#!/usr/bin/env python3
"""
Single-process Edge AI Brain for Jetson Orin Nano 8GB.

Design goals:
- Preload all models once at startup.
- Route STT text into 3 branches (CMD / CHAT / VISION) via KoSimCSE + SQLite.
- Never run LLM and VLM inference at the same time (shared inference lock).
- Keep audio I/O and inference in one Python process.
"""

from __future__ import annotations

import importlib.util
import json
import os
import queue
import re
import socket
import sqlite3
import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import sounddevice as sd
import torch
import torchaudio.functional as F_audio

# ── torch < 2.4.0 호환성 패치 ────────────────────────────────────────────────
# torch.serialization.add_safe_globals 는 2.4.0 에서 추가됨.
# 구버전에서 transformers/torchao 가 호출하면 AttributeError 발생 → no-op 으로 패치.
if not hasattr(torch.serialization, "add_safe_globals"):
    torch.serialization.add_safe_globals = lambda *a, **kw: None  # type: ignore[attr-defined]


# -----------------------------
# Config / Data models
# -----------------------------


@dataclass
class BrainConfig:
    # Audio
    stt_sample_rate: int = 16000
    tts_sample_rate: int = 22050
    channels: int = 1
    listen_seconds: float = 2.0

    # STT
    stt_engine: str = "moonshine"          # whisper | moonshine  (whisper: tokenizers 충돌 없음)
    moonshine_script_path: str = "/home/ubuntu/rover_ws/src/stt_cpp/scripts/moonshine_stt.py"
    moonshine_model_name: str = "moonshine-tiny-ko"
    whisper_model_name: str = "tiny"

    # Intent Router
    intent_db_path: str = "/app/models/intent_router/intents.db"
    intent_embed_model: str = "BM-K/KoSimCSE-roberta"
    cmd_threshold: float = 0.82
    vision_threshold: float = 0.70

    # LLM (GGUF + llama.cpp)
    llm_model_path: str = "/app/models/llm/Qwen1.5-1.5B-Chat.gguf"
    llm_ctx: int = 1024
    llm_max_tokens: int = 96

    # VLM (Moondream2)
    vlm_model_name: str = "vikhyatk/moondream2"
    vlm_revision: str = "2024-08-26"     # torchao 의존 없는 마지막 안전 revision
    camera_index: int = 0

    # Action bridge
    action_host: str = "127.0.0.1"
    action_port: int = 8765
    action_queue_size: int = 64

    warmup_enabled: bool = True


@dataclass
class IntentDecision:
    route: str  # CMD | CHAT | VISION
    text: str
    score: float
    response_hint: str
    action: dict[str, Any]


# -----------------------------
# Action sender
# -----------------------------


class ActionSenderThread(threading.Thread):
    """Non-blocking action sender for host ROS2 bridge."""

    def __init__(self, host: str, port: int, q: "queue.Queue[dict[str, Any]]") -> None:
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.q = q
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        backoff = 1.0
        sock: socket.socket | None = None
        while not self._stop_event.is_set():
            try:
                if sock is None:
                    sock = socket.create_connection((self.host, self.port), timeout=2.0)
                    sock.settimeout(2.0)
                    backoff = 1.0

                action = self.q.get(timeout=0.2)
                payload = (json.dumps(action, ensure_ascii=False) + "\n").encode("utf-8")
                sock.sendall(payload)
            except queue.Empty:
                continue
            except Exception:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass
                sock = None
                time.sleep(backoff)
                backoff = min(backoff * 1.5, 5.0)

        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass


# -----------------------------
# STT
# -----------------------------


class STTEngine:
    def __init__(self, cfg: BrainConfig, device: torch.device) -> None:
        self.cfg = cfg
        self.device = device
        self.engine = cfg.stt_engine.lower().strip()

        if self.engine == "moonshine":
            self._moonshine = self._load_moonshine_module(cfg.moonshine_script_path)
            self.dtype = torch.float32
            self.state = self._moonshine.load_ko_model_state(
                cfg.moonshine_model_name, self.device, self.dtype
            )
            self.max_tokens = 128
        elif self.engine == "whisper":
            import whisper
            self.whisper_model = whisper.load_model(cfg.whisper_model_name, device=str(device))
        else:
            raise ValueError(f"unsupported_stt_engine:{cfg.stt_engine}")

    @staticmethod
    def _load_moonshine_module(script_path: str):
        if not os.path.exists(script_path):
            raise FileNotFoundError(f"moonshine_script_not_found:{script_path}")
        spec = importlib.util.spec_from_file_location("moonshine_stt_runtime", script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("moonshine_import_spec_failed")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    @torch.no_grad()
    def transcribe(self, audio_f32: np.ndarray, sample_rate: int) -> str:
        if audio_f32.size == 0:
            return ""
        try:
            mono = np.asarray(audio_f32, dtype=np.float32).reshape(-1)
            mono = np.clip(mono, -1.0, 1.0)

            if sample_rate != 16000:
                x = torch.from_numpy(mono).unsqueeze(0)
                x = F_audio.resample(x, orig_freq=sample_rate, new_freq=16000)
                mono = x.squeeze(0).cpu().numpy().astype(np.float32, copy=False)

            if self.engine == "moonshine":
                audio_tensor = torch.from_numpy(mono).to(
                    device=self.state["device"], dtype=self.state["dtype"], non_blocking=True
                )
                decoder = self._moonshine.MoonshineDecoder(self.state)
                decoder.encode(audio_tensor)
                duration_sec = max(len(mono) / 16000.0, 0.2)
                max_len = min(max(int(duration_sec * 80), 32), self.max_tokens)
                text = decoder.generate(max_len=max_len, early_stop=True)
                return self._moonshine.sanitize_transcript(text)

            # whisper
            out = self.whisper_model.transcribe(mono, language="ko", fp16=self.device.type == "cuda")
            return str(out.get("text", "")).strip()
        except Exception as e:
            print(f"[stt error] {e}")
            return ""


# -----------------------------
# Intent Router (SQLite + KoSimCSE)
# -----------------------------


class IntentRouter:
    def __init__(self, cfg: BrainConfig) -> None:
        self.cfg = cfg
        os.makedirs(os.path.dirname(cfg.intent_db_path), exist_ok=True)
        self.conn = sqlite3.connect(cfg.intent_db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row

        self._init_db()
        self._load_encoder()
        self._refresh_cache()

        self.vision_keywords = (
            "뭐", "무엇", "보여", "앞", "물건", "상자", "컵", "병", "색", "카메라", "보이나"
        )

    def _init_db(self) -> None:
        cur = self.conn.cursor()
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS intent_templates (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                phrase TEXT NOT NULL,
                route TEXT NOT NULL,
                action_json TEXT NOT NULL,
                response_hint TEXT NOT NULL
            )
            """
        )
        self.conn.commit()

        cur.execute("SELECT COUNT(*) AS n FROM intent_templates")
        n = int(cur.fetchone()["n"])
        if n > 0:
            return

        seeds = [
            ("앞으로 가", "CMD", '{"cmd":"move_forward"}', "앞으로 이동할게요."),
            ("뒤로 가", "CMD", '{"cmd":"move_backward"}', "뒤로 이동할게요."),
            ("왼쪽으로 돌아", "CMD", '{"cmd":"turn_left"}', "왼쪽으로 회전할게요."),
            ("오른쪽으로 돌아", "CMD", '{"cmd":"turn_right"}', "오른쪽으로 회전할게요."),
            ("멈춰", "CMD", '{"cmd":"stop"}', "즉시 정지할게요."),
            ("정지", "CMD", '{"cmd":"stop"}', "즉시 정지할게요."),
            ("나한테 와", "CMD", '{"cmd":"move_forward"}', "당신 쪽으로 이동할게요."),
            ("저 상자 집어와", "VISION", '{"cmd":"pick_and_place"}', "카메라로 상자를 확인해볼게요."),
            ("앞에 무슨 물건이 있어", "VISION", '{"cmd":"none"}', "앞쪽 물체를 확인해볼게요."),
            ("이게 뭐야", "VISION", '{"cmd":"none"}', "화면을 보고 설명해볼게요."),
            ("안녕", "CHAT", '{"cmd":"none"}', "안녕하세요."),
        ]
        cur.executemany(
            "INSERT INTO intent_templates(phrase, route, action_json, response_hint) VALUES(?,?,?,?)",
            seeds,
        )
        self.conn.commit()

    def _load_encoder(self) -> None:
        from sentence_transformers import SentenceTransformer
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.encoder = SentenceTransformer(self.cfg.intent_embed_model, device=device)

    def _refresh_cache(self) -> None:
        rows = self.conn.execute(
            "SELECT id, phrase, route, action_json, response_hint FROM intent_templates"
        ).fetchall()
        self.rows = list(rows)
        phrases = [r["phrase"] for r in rows]
        if phrases:
            emb = self.encoder.encode(phrases, normalize_embeddings=True, convert_to_numpy=True)
            self.template_emb = emb.astype(np.float32, copy=False)
        else:
            self.template_emb = np.zeros((0, 1), dtype=np.float32)

    def route(self, text: str) -> IntentDecision:
        q = (text or "").strip()
        if not q:
            return IntentDecision("CHAT", "", 0.0, "", {"cmd": "none"})

        if self.template_emb.shape[0] == 0:
            return IntentDecision("CHAT", q, 0.0, "", {"cmd": "none"})

        q_emb = self.encoder.encode([q], normalize_embeddings=True, convert_to_numpy=True)[0].astype(
            np.float32, copy=False
        )
        sims = self.template_emb @ q_emb
        idx = int(np.argmax(sims))
        score = float(sims[idx])
        row = self.rows[idx]

        try:
            action = json.loads(row["action_json"])
            if not isinstance(action, dict):
                action = {"cmd": "none"}
        except Exception:
            action = {"cmd": "none"}
        action.setdefault("cmd", "none")

        route = str(row["route"]).upper().strip()
        hint = str(row["response_hint"])

        if route == "CMD" and score >= self.cfg.cmd_threshold:
            return IntentDecision("CMD", q, score, hint, action)

        if route == "VISION" and score >= self.cfg.vision_threshold:
            return IntentDecision("VISION", q, score, hint, action)
        if any(k in q for k in self.vision_keywords):
            return IntentDecision("VISION", q, score, hint or "시각 분석을 진행할게요.", {"cmd": "none"})

        return IntentDecision("CHAT", q, score, "", {"cmd": "none"})


# -----------------------------
# LLM (llama.cpp / GGUF)
# -----------------------------


class LLMEngine:
    def __init__(self, cfg: BrainConfig) -> None:
        from llama_cpp import Llama

        self.system_prompt = (
            "너는 한국어 로봇 비서다. 짧고 정확하게 답한다. "
            "로봇 명령이 아니면 일반 대화로 응답하라."
        )
        self.llm = Llama(
            model_path=cfg.llm_model_path,
            n_gpu_layers=-1,
            n_ctx=cfg.llm_ctx,
            n_batch=512,
            n_threads=max(4, (os.cpu_count() or 8) // 2),
            chat_format="chatml",
            verbose=False,
        )
        self.max_tokens = cfg.llm_max_tokens

    def generate(self, text: str) -> tuple[str, dict[str, Any]]:
        """LLM 추론. (response_text, action_dict) 튜플 반환."""
        try:
            out = self.llm.create_chat_completion(
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": text},
                ],
                temperature=0.2,
                top_p=0.9,
                max_tokens=self.max_tokens,
            )
            response = (
                out.get("choices", [{}])[0]
                .get("message", {})
                .get("content", "")
                .strip()
                or "잘 들었어요."
            )
        except Exception as e:
            print(f"[llm error] {e}")
            response = "답변을 생성하지 못했어요."
        return response, {"cmd": "none"}


# -----------------------------
# VLM (Moondream2)
# -----------------------------


class VLMEngine:
    def __init__(self, cfg: BrainConfig, device: torch.device) -> None:
        self.device = device
        self.model_name = cfg.vlm_model_name
        self._ready = False
        self.model = None
        self.tokenizer = None

        dtype = torch.float16 if device.type == "cuda" else torch.float32

        # 1차 시도: 최신 버전 (캐시된 모델 사용)
        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer
            self.tokenizer = AutoTokenizer.from_pretrained(
                self.model_name, trust_remote_code=True)
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name, trust_remote_code=True, torch_dtype=dtype)
            self.model.to(device)
            self.model.eval()
            self._ready = True
            print(f"[vlm] loaded: {self.model_name} (latest)")
        except Exception as e:
            print(f"[vlm] latest load failed ({e}), retrying with revision={cfg.vlm_revision}")
            # 2차 시도: torchao 없는 안전 revision
            try:
                from transformers import AutoModelForCausalLM, AutoTokenizer
                self.tokenizer = AutoTokenizer.from_pretrained(
                    self.model_name, trust_remote_code=True, revision=cfg.vlm_revision)
                self.model = AutoModelForCausalLM.from_pretrained(
                    self.model_name, trust_remote_code=True, torch_dtype=dtype,
                    revision=cfg.vlm_revision)
                self.model.to(device)
                self.model.eval()
                self._ready = True
                print(f"[vlm] loaded: {self.model_name} revision={cfg.vlm_revision}")
            except Exception as e2:
                print(f"[vlm] load failed entirely: {e2}. VLM branch will be disabled.")

    @staticmethod
    def _to_pil(image_bgr: np.ndarray):
        from PIL import Image
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb)

    @torch.no_grad()
    def analyze(self, image_bgr: np.ndarray, text: str) -> tuple[str, dict[str, Any]]:
        if not self._ready or self.model is None:
            return "VLM을 사용할 수 없습니다.", {"cmd": "none"}
        try:
            image = self._to_pil(image_bgr)

            if hasattr(self.model, "query"):
                result = self.model.query(image, text)
            elif hasattr(self.model, "chat"):
                result = self.model.chat(
                    image=image,
                    msgs=[{"role": "user", "content": text}],
                    tokenizer=self.tokenizer)
            else:
                result = "이미지 분석 API를 찾지 못했습니다."

            if isinstance(result, dict):
                response = str(result.get("answer") or result.get("text") or result)
            else:
                response = str(result)
            response = response.strip() or "이미지를 분석했어요."

            lower = text.lower()
            if any(k in lower for k in ("집어", "가져", "상자", "물건")):
                action: dict[str, Any] = {"cmd": "pick_and_place"}
            else:
                action = {"cmd": "none"}

            return response, action
        except Exception as e:
            print(f"[vlm error] {e}")
            return "이미지 분석에 실패했어요.", {"cmd": "none"}


# -----------------------------
# TTS (MeloTTS)
# -----------------------------


class TTSEngine:
    def __init__(self, cfg: BrainConfig, device: torch.device) -> None:
        from melo import utils as melo_utils
        from melo.api import TTS

        self.cfg = cfg
        self.device = device
        self.speed = 1.0
        self.melo_utils = melo_utils
        self.tts = TTS(language="KR", device=str(self.device))
        self.language = self.tts.language

        spk2id = dict(getattr(getattr(self.tts.hps, "data"), "spk2id", {}))
        self.speaker_id = int(spk2id.get("KR", next(iter(spk2id.values()), 0)))

    @torch.no_grad()
    def synthesize_stream(self, text: str):
        """문장 단위 스트리밍 생성기 (np.ndarray chunk 를 yield)."""
        clean = (text or "").strip()
        if not clean:
            yield np.zeros(1, dtype=np.float32)
            return

        sentences = self.tts.split_sentences_into_pieces(clean, self.language, quiet=True)
        for sentence in sentences:
            s = sentence
            if self.language in ["EN", "ZH_MIX_EN"]:
                s = re.sub(r"([a-z])([A-Z])", r"\1 \2", s)

            bert, ja_bert, phones, tones, lang_ids = self.melo_utils.get_text_for_tts_infer(
                s, self.language, self.tts.hps, self.tts.device, self.tts.symbol_to_id
            )

            x_tst = phones.to(self.device).unsqueeze(0)
            tones_t = tones.to(self.device).unsqueeze(0)
            lang_ids_t = lang_ids.to(self.device).unsqueeze(0)
            bert_t = bert.to(self.device).unsqueeze(0)
            ja_bert_t = ja_bert.to(self.device).unsqueeze(0)
            x_tst_lengths = torch.LongTensor([phones.size(0)]).to(self.device)
            speakers = torch.LongTensor([self.speaker_id]).to(self.device)

            audio_t = self.tts.model.infer(
                x_tst, x_tst_lengths, speakers, tones_t, lang_ids_t, bert_t, ja_bert_t,
                sdp_ratio=0.2, noise_scale=0.6, noise_scale_w=0.8,
                length_scale=1.0 / self.speed,
            )[0][0, 0]
            audio = audio_t.detach().float().cpu().numpy().astype(np.float32, copy=False)

            sr = int(self.tts.hps.data.sampling_rate)
            if sr != self.cfg.tts_sample_rate:
                t = torch.from_numpy(audio).unsqueeze(0)
                t = F_audio.resample(t, orig_freq=sr, new_freq=self.cfg.tts_sample_rate)
                audio = t.squeeze(0).cpu().numpy().astype(np.float32, copy=False)
            yield audio

    def synthesize(self, text: str) -> np.ndarray:
        """전체 텍스트를 합성해 단일 np.ndarray 반환 (bench 호환용)."""
        chunks = list(self.synthesize_stream(text))
        if not chunks:
            return np.zeros(1, dtype=np.float32)
        return np.concatenate(chunks).astype(np.float32, copy=False)


# -----------------------------
# Main Brain
# -----------------------------


class AIBrain:
    def __init__(self, cfg: BrainConfig) -> None:
        self.cfg = cfg
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        torch.set_grad_enabled(False)

        self.stt = STTEngine(cfg, self.device)
        self.intent = IntentRouter(cfg)
        self.llm = LLMEngine(cfg)
        self.vlm = VLMEngine(cfg, self.device)
        self.tts = TTSEngine(cfg, self.device)

        # LLM 과 VLM 은 동시에 추론하지 않는다.
        self.infer_lock = threading.Lock()

        self.camera = cv2.VideoCapture(cfg.camera_index)

        self.action_q: "queue.Queue[dict[str, Any]]" = queue.Queue(maxsize=cfg.action_queue_size)
        self.sender = ActionSenderThread(cfg.action_host, cfg.action_port, self.action_q)

    def start(self) -> None:
        self.sender.start()
        if self.cfg.warmup_enabled:
            self.warmup_once()
        print("[ai-brain] started")

    def stop(self) -> None:
        self.sender.stop()
        self.sender.join(timeout=2.0)
        try:
            self.camera.release()
        except Exception:
            pass

    @torch.no_grad()
    def warmup_once(self) -> None:
        print("[warmup] start")
        t0 = time.perf_counter()
        try:
            dummy_audio = np.zeros(int(self.cfg.stt_sample_rate * 0.25), dtype=np.float32)
            _ = self.stt.transcribe(dummy_audio, self.cfg.stt_sample_rate)

            with self.infer_lock:
                _, _ = self.llm.generate("시스템 워밍업")

            if self.vlm._ready:
                dummy_img = np.full((480, 640, 3), 128, dtype=np.uint8)
                with self.infer_lock:
                    _ = self.vlm.analyze(dummy_img, "무엇이 보이나요?")

            _ = self.intent.route("워밍업")
            _ = next(self.tts.synthesize_stream("준비되었습니다"))

            if self.device.type == "cuda":
                torch.cuda.synchronize()
            print(f"[warmup] done in {(time.perf_counter() - t0)*1000:.1f}ms")
        except Exception as e:
            print(f"[warmup] skipped: {e}")

    def listen_once(self, seconds: float | None = None) -> np.ndarray:
        sec = float(seconds if seconds is not None else self.cfg.listen_seconds)
        frames = int(self.cfg.stt_sample_rate * sec)
        print("[ai-brain] listening...")
        audio = sd.rec(
            frames,
            samplerate=self.cfg.stt_sample_rate,
            channels=self.cfg.channels,
            dtype="float32",
            blocking=True,
        )
        mono = audio.reshape(-1).astype(np.float32, copy=False)
        return np.clip(mono, -1.0, 1.0)

    def play_audio(
        self, audio_f32: np.ndarray, orig_sample_rate: int, target_sample_rate: int = 48000
    ) -> None:
        try:
            out = audio_f32
            if orig_sample_rate != target_sample_rate:
                t = torch.from_numpy(audio_f32).unsqueeze(0)
                t = F_audio.resample(t, orig_freq=orig_sample_rate, new_freq=target_sample_rate)
                out = t.squeeze(0).numpy()
            sd.play(out, samplerate=target_sample_rate, blocking=True)
        except Exception as e:
            print(f"[audio play error] {e}")

    def capture_frame(self) -> np.ndarray | None:
        ok, frame = self.camera.read()
        if not ok:
            return None
        return frame

    def _play_tts_gapless(self, response: str) -> None:
        """Producer-consumer gapless TTS 재생."""
        audio_q: "queue.Queue[np.ndarray | None]" = queue.Queue(maxsize=4)

        def _producer() -> None:
            try:
                for chunk in self.tts.synthesize_stream(response):
                    audio_q.put(chunk)
            finally:
                audio_q.put(None)

        producer = threading.Thread(target=_producer, daemon=True)
        producer.start()
        while True:
            chunk = audio_q.get()
            if chunk is None:
                break
            self.play_audio(chunk, self.cfg.tts_sample_rate, target_sample_rate=48000)
        producer.join(timeout=5.0)

    def _run_branch_chat(self, text: str) -> tuple[str, dict[str, Any]]:
        with self.infer_lock:
            response, action = self.llm.generate(text)
        return response, action

    def _run_branch_vision(self, text: str) -> tuple[str, dict[str, Any]]:
        frame = self.capture_frame()
        if frame is None:
            return "카메라 프레임을 가져오지 못했어요.", {"cmd": "none"}
        with self.infer_lock:
            return self.vlm.analyze(frame, text)

    def run_turn(self, audio: np.ndarray) -> dict[str, Any]:
        t0 = time.perf_counter()

        text = self.stt.transcribe(audio, self.cfg.stt_sample_rate)
        t_stt = time.perf_counter()

        decision = self.intent.route(text)
        t_route = time.perf_counter()

        if decision.route == "CMD":
            response = decision.response_hint or "명령을 수행할게요."
            action = decision.action
        elif decision.route == "VISION":
            response, action = self._run_branch_vision(decision.text)
        else:
            response, action = self._run_branch_chat(decision.text)
        t_infer = time.perf_counter()

        try:
            self.action_q.put_nowait(action)
        except queue.Full:
            print("[action] queue full, dropping action")

        self._play_tts_gapless(response)
        t_tts = time.perf_counter()

        if self.device.type == "cuda":
            torch.cuda.empty_cache()

        return {
            "text": text,
            "route": decision.route,
            "score": decision.score,
            "response": response,
            "action": action,
            "stt_ms": (t_stt - t0) * 1000.0,
            "route_ms": (t_route - t_stt) * 1000.0,
            "infer_ms": (t_infer - t_route) * 1000.0,
            "tts_ms": (t_tts - t_infer) * 1000.0,
            "total_ms": (t_tts - t0) * 1000.0,
        }

    def run_turn_streaming(
        self, text: str
    ) -> tuple[str, dict[str, Any], np.ndarray, dict[str, float]]:
        """bench 스크립트 호환용: LLM → TTS 순차 실행, 타이밍 딕셔너리 반환.

        Returns:
            (response, action, tts_audio_f32, stats_dict)
            stats keys: llm_done_ms, resp_extracted_ms, tts_done_ms, first_sound_ms
        """
        t0 = time.perf_counter()
        with self.infer_lock:
            response, action = self.llm.generate(text)
        t_llm = time.perf_counter()
        llm_ms = (t_llm - t0) * 1000.0

        tts_audio = self.tts.synthesize(response)
        t_tts = time.perf_counter()
        tts_ms = (t_tts - t_llm) * 1000.0

        stats = {
            "llm_done_ms": llm_ms,
            "resp_extracted_ms": llm_ms,
            "tts_done_ms": llm_ms + tts_ms,
            "first_sound_ms": llm_ms,
        }
        return response, action, tts_audio, stats

    def run_forever(self) -> None:
        self.start()
        try:
            while True:
                audio = self.listen_once()
                m = self.run_turn(audio)
                print(
                    f"[turn] text={m['text']!r} route={m['route']} score={m['score']:.3f} "
                    f"action={m['action']} response={m['response']!r}"
                )
                print(
                    f"[latency] stt={m['stt_ms']:.1f}ms route={m['route_ms']:.1f}ms "
                    f"infer={m['infer_ms']:.1f}ms tts={m['tts_ms']:.1f}ms total={m['total_ms']:.1f}ms"
                )
        finally:
            self.stop()


def main() -> None:
    cfg = BrainConfig()
    brain = AIBrain(cfg)
    brain.run_forever()


if __name__ == "__main__":
    main()
