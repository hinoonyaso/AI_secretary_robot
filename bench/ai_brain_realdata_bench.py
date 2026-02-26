#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any
import wave

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from main_ai_brain import AIBrain, BrainConfig


@dataclass
class TurnResult:
    source: str
    stt_text: str
    response: str
    action: dict[str, Any]
    stt_ms: float
    llm_ms: float
    tts_ms: float
    first_sound_ms: float   # stt+max(llm, resp_extracted+tts) – time until audio ready
    play_ms: float
    total_ms: float
    total_e2e_ms: float
    tts_samples: int
    stage_metrics: dict[str, Any]


@dataclass
class TegraSample:
    ts: float
    w_mw: int
    ram_used_mb: int
    ram_total_mb: int


class TegraMonitor:
    pattern_w = re.compile(r"VDD_IN (\d+)mW/(\d+)mW")
    pattern_ram = re.compile(r"RAM (\d+)/(\d+)MB")

    def __init__(self, interval_ms: int = 200) -> None:
        self.interval_ms = interval_ms
        self.proc: subprocess.Popen[str] | None = None
        self.samples: list[TegraSample] = []
        self._lock = threading.Lock()
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        try:
            self.proc = subprocess.Popen(
                ["tegrastats", "--interval", str(self.interval_ms)],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
            self.samples = []
            self._stop_event.clear()
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()
        except FileNotFoundError:
            self.proc = None

    def _reader_loop(self) -> None:
        if self.proc is None or self.proc.stdout is None:
            return
        for line in self.proc.stdout:
            if self._stop_event.is_set():
                break
            m_w = self.pattern_w.search(line)
            m_r = self.pattern_ram.search(line)
            if not m_w:
                continue
            w_mw = int(m_w.group(1))
            ram_used_mb = int(m_r.group(1)) if m_r else 0
            ram_total_mb = int(m_r.group(2)) if m_r else 0
            s = TegraSample(
                ts=time.perf_counter(),
                w_mw=w_mw,
                ram_used_mb=ram_used_mb,
                ram_total_mb=ram_total_mb,
            )
            with self._lock:
                self.samples.append(s)

    def stop(self) -> list[TegraSample]:
        if self.proc is None:
            return []
        self._stop_event.set()
        self.proc.terminate()
        try:
            self.proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.communicate()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        with self._lock:
            return list(self.samples)

    def slice(self, ts_start: float, ts_end: float) -> list[TegraSample]:
        with self._lock:
            return [s for s in self.samples if ts_start <= s.ts <= ts_end]


def read_wav_mono_f32(path: Path) -> tuple[np.ndarray, int]:
    with wave.open(str(path), "rb") as wf:
        n_channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        sr = wf.getframerate()
        frames = wf.getnframes()
        raw = wf.readframes(frames)

    if sampwidth == 1:
        x = np.frombuffer(raw, dtype=np.uint8).astype(np.float32)
        x = (x - 128.0) / 128.0
    elif sampwidth == 2:
        x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    elif sampwidth == 4:
        x = np.frombuffer(raw, dtype="<i4").astype(np.float32) / 2147483648.0
    else:
        raise ValueError(f"unsupported_sample_width:{sampwidth}")

    if n_channels > 1:
        x = x.reshape(-1, n_channels).mean(axis=1)
    return np.clip(x.astype(np.float32, copy=False), -1.0, 1.0), sr


def summarize_power(vals: list[int]) -> dict[str, float]:
    if not vals:
        return {"n": 0, "avg_w": 0.0, "max_w": 0.0}
    return {
        "n": len(vals),
        "avg_w": float(sum(vals) / len(vals) / 1000.0),
        "max_w": float(max(vals) / 1000.0),
    }


def summarize_samples(samples: list[TegraSample]) -> dict[str, float]:
    if not samples:
        return {
            "n": 0,
            "avg_w": 0.0,
            "max_w": 0.0,
            "peak_ram_mb": 0.0,
            "peak_ram_pct": 0.0,
        }
    ws = [s.w_mw for s in samples]
    rams = [s.ram_used_mb for s in samples if s.ram_used_mb > 0]
    ram_total = max((s.ram_total_mb for s in samples if s.ram_total_mb > 0), default=0)
    peak_ram = float(max(rams) if rams else 0.0)
    peak_pct = float((peak_ram / ram_total) * 100.0) if ram_total > 0 else 0.0
    return {
        "n": len(samples),
        "avg_w": float(sum(ws) / len(ws) / 1000.0),
        "max_w": float(max(ws) / 1000.0),
        "peak_ram_mb": peak_ram,
        "peak_ram_pct": peak_pct,
    }


def process_rss_mb() -> float:
    try:
        with open("/proc/self/status", "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    kb = float(line.split()[1])
                    return kb / 1024.0
    except Exception:
        pass
    return 0.0


def cuda_stats_mb() -> dict[str, float]:
    if not torch.cuda.is_available():
        return {"allocated_mb": 0.0, "reserved_mb": 0.0, "max_allocated_mb": 0.0}
    return {
        "allocated_mb": float(torch.cuda.memory_allocated() / (1024 ** 2)),
        "reserved_mb": float(torch.cuda.memory_reserved() / (1024 ** 2)),
        "max_allocated_mb": float(torch.cuda.max_memory_allocated() / (1024 ** 2)),
    }


def pick_wavs(dir_path: Path, count: int) -> list[Path]:
    wavs = sorted(dir_path.glob("*.wav"), key=lambda p: p.stat().st_mtime, reverse=True)
    if len(wavs) < count:
        raise FileNotFoundError(f"not_enough_wavs:{dir_path} need={count} found={len(wavs)}")
    return wavs[:count]


def run_turn(
    brain: AIBrain,
    active_mon: TegraMonitor,
    wav_path: Path | None = None,
    input_text: str | None = None,
    playback: bool = False,
    streaming: bool = False,
) -> TurnResult:
    stage_metrics: dict[str, Any] = {}

    def begin_stage() -> tuple[float, float]:
        if torch.cuda.is_available():
            torch.cuda.reset_peak_memory_stats()
        return time.perf_counter(), process_rss_mb()

    def end_stage(name: str, t_start: float, rss_start: float) -> tuple[float, float]:
        t_end = time.perf_counter()
        rss_end = process_rss_mb()
        pw = summarize_samples(active_mon.slice(t_start, t_end))
        cuda = cuda_stats_mb()
        stage_metrics[name] = {
            "latency_ms": float((t_end - t_start) * 1000.0),
            "process_rss_mb_start": float(rss_start),
            "process_rss_mb_end": float(rss_end),
            "process_rss_mb_peak_est": float(max(rss_start, rss_end)),
            "cuda": cuda,
            "power": pw,
        }
        return t_end, float((t_end - t_start) * 1000.0)

    t0 = time.perf_counter()
    if input_text is not None:
        t_stt_start, stt_rss_start = begin_stage()
        stt_text = input_text
        source = f"text:{input_text}"
        t1, stt_ms = end_stage("stt", t_stt_start, stt_rss_start)
    else:
        if wav_path is None:
            raise ValueError("wav_path_or_input_text_required")
        audio, sr = read_wav_mono_f32(wav_path)
        t_stt_start, stt_rss_start = begin_stage()
        stt_text = brain.stt.transcribe(audio, sr)
        t1, stt_ms = end_stage("stt", t_stt_start, stt_rss_start)
        source = str(wav_path)

    if streaming:
        t_llm_start, llm_rss_start = begin_stage()
        response, action, tts_audio, st = brain.run_turn_streaming(stt_text)
        t2, _ = end_stage("llm_tts_streaming", t_llm_start, llm_rss_start)
        llm_ms = st.get("llm_done_ms", 0.0)
        resp_ms = st.get("resp_extracted_ms", llm_ms)
        tts_ms = st.get("tts_done_ms", 0.0) - resp_ms
        first_sound_ms = stt_ms + st.get("first_sound_ms", 0.0)
        total_ms = (t2 - t0) * 1000.0
    else:
        t_llm_start, llm_rss_start = begin_stage()
        response, action = brain.llm.generate(stt_text)
        t_llm, llm_ms = end_stage("llm", t_llm_start, llm_rss_start)

        t_tts_start, tts_rss_start = begin_stage()
        tts_audio = brain.tts.synthesize(response)
        t2, tts_ms = end_stage("tts", t_tts_start, tts_rss_start)
        first_sound_ms = (t2 - t0) * 1000.0
        total_ms = (t2 - t0) * 1000.0

    t_play_start, play_rss_start = begin_stage()
    if playback:
        brain.play_audio(tts_audio, brain.cfg.tts_sample_rate, target_sample_rate=48000)
    t4, play_ms = end_stage("play", t_play_start, play_rss_start)
    return TurnResult(
        source=source,
        stt_text=stt_text,
        response=response,
        action=action,
        stt_ms=stt_ms,
        llm_ms=llm_ms,
        tts_ms=tts_ms,
        first_sound_ms=first_sound_ms,
        play_ms=play_ms,
        total_ms=total_ms,
        total_e2e_ms=(t4 - t0) * 1000.0,
        tts_samples=int(tts_audio.shape[0]),
        stage_metrics=stage_metrics,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--wav-dir",
        default="/home/ubuntu/rover_ws/install/wake_vad_cpp/share/wake_vad_cpp/record_sound",
    )
    parser.add_argument("--turns", type=int, default=3)
    parser.add_argument("--idle-secs", type=float, default=5.0)
    parser.add_argument("--interval-ms", type=int, default=200)
    parser.add_argument("--out-prefix", default="/tmp/ai_brain_realdata")
    parser.add_argument(
        "--input-texts",
        default="",
        help="use direct text inputs separated by '|' (bypasses STT audio decode)",
    )
    parser.add_argument(
        "--playback",
        action="store_true",
        help="play synthesized audio to speaker and include play_ms in e2e timing",
    )
    parser.add_argument(
        "--streaming",
        action="store_true",
        help="use overlapped LLM→TTS streaming pipeline (TTS starts before LLM finishes)",
    )
    args = parser.parse_args()

    input_texts = [x.strip() for x in args.input_texts.split("|") if x.strip()]
    use_text_mode = len(input_texts) > 0
    if use_text_mode:
        wavs: list[Path] = []
    else:
        wav_dir = Path(args.wav_dir)
        wavs = pick_wavs(wav_dir, args.turns)

    stamp = int(time.time())
    out_json = Path(f"{args.out_prefix}_summary_{stamp}.json")
    out_txt = Path(f"{args.out_prefix}_latency_{stamp}.log")

    cfg = BrainConfig()
    brain = AIBrain(cfg)
    brain.start()

    idle_mon = TegraMonitor(interval_ms=args.interval_ms)
    idle_mon.start()
    time.sleep(args.idle_secs)
    idle_samples = idle_mon.stop()

    active_mon = TegraMonitor(interval_ms=args.interval_ms)
    active_mon.start()

    results: list[TurnResult] = []
    if use_text_mode:
        for text in input_texts:
            r = run_turn(
                brain,
                active_mon=active_mon,
                input_text=text,
                playback=args.playback,
                streaming=args.streaming,
            )
            results.append(r)
    else:
        for wav_path in wavs:
            r = run_turn(
                brain,
                active_mon=active_mon,
                wav_path=wav_path,
                playback=args.playback,
                streaming=args.streaming,
            )
            results.append(r)

    active_samples = active_mon.stop()
    brain.stop()

    with out_txt.open("w", encoding="utf-8") as f:
        for i, r in enumerate(results, start=1):
            f.write(
                f"[turn{i}] source={r.source} stt_text={r.stt_text!r} "
                f"resp={r.response!r} action={r.action} tts_samples={r.tts_samples}\n"
            )
            f.write(
                f"[latency] turn={i} stt_ms={r.stt_ms:.1f} llm_ms={r.llm_ms:.1f} "
                f"tts_ms={r.tts_ms:.1f} play_ms={r.play_ms:.1f} "
                f"total_ms={r.total_ms:.1f} total_e2e_ms={r.total_e2e_ms:.1f}\n"
            )
            f.write(f"[stage_metrics] turn={i} {json.dumps(r.stage_metrics, ensure_ascii=False)}\n")

    def stage_mean() -> dict[str, Any]:
        stage_names: set[str] = set()
        for r in results:
            stage_names.update(r.stage_metrics.keys())
        out: dict[str, Any] = {}
        for s in sorted(stage_names):
            rows = [r.stage_metrics[s] for r in results if s in r.stage_metrics]
            if not rows:
                continue
            out[s] = {
                "latency_ms": float(sum(x["latency_ms"] for x in rows) / len(rows)),
                "process_rss_mb_peak_est": float(max(x["process_rss_mb_peak_est"] for x in rows)),
                "cuda_allocated_mb_peak": float(max(x["cuda"]["allocated_mb"] for x in rows)),
                "cuda_reserved_mb_peak": float(max(x["cuda"]["reserved_mb"] for x in rows)),
                "cuda_max_allocated_mb_peak": float(max(x["cuda"]["max_allocated_mb"] for x in rows)),
                "power_avg_w": float(sum(x["power"]["avg_w"] for x in rows) / len(rows)),
                "power_max_w_peak": float(max(x["power"]["max_w"] for x in rows)),
                "system_ram_peak_mb": float(max(x["power"]["peak_ram_mb"] for x in rows)),
            }
        return out

    mean = lambda key: float(sum(getattr(r, key) for r in results) / len(results))
    summary = {
        "measured_at": datetime.now().isoformat(timespec="seconds"),
        "input_mode": "text" if use_text_mode else "wav",
        "inputs": input_texts if use_text_mode else [str(x) for x in wavs],
        "playback": bool(args.playback),
        "streaming": bool(args.streaming),
        "turns": [asdict(r) for r in results],
        "mean_ms": {
            "stt": mean("stt_ms"),
            "llm": mean("llm_ms"),
            "tts": mean("tts_ms"),
            "first_sound": mean("first_sound_ms"),
            "play": mean("play_ms"),
            "total": mean("total_ms"),
            "total_e2e": mean("total_e2e_ms"),
        },
        "model_stage_mean": stage_mean(),
        "power": {
            "idle": summarize_samples(idle_samples),
            "active": summarize_samples(active_samples),
        },
        "artifacts": {
            "latency_log": str(out_txt),
            "summary_json": str(out_json),
        },
    }
    out_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
