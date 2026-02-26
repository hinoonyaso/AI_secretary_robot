#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import wave
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from main_ai_brain import (
    BrainConfig,
    IntentRouter,
    LLMEngine,
    STTEngine,
    TTSEngine,
    VLMEngine,
)


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


def rss_mb() -> float:
    try:
        with open("/proc/self/status", "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return float(line.split()[1]) / 1024.0
    except Exception:
        return 0.0
    return 0.0


def cuda_mb() -> dict[str, float]:
    if not torch.cuda.is_available():
        return {"allocated_mb": 0.0, "reserved_mb": 0.0, "max_allocated_mb": 0.0}
    return {
        "allocated_mb": float(torch.cuda.memory_allocated() / (1024 ** 2)),
        "reserved_mb": float(torch.cuda.memory_reserved() / (1024 ** 2)),
        "max_allocated_mb": float(torch.cuda.max_memory_allocated() / (1024 ** 2)),
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--case", required=True, choices=["stt", "intent", "llm", "vlm", "tts"])
    p.add_argument(
        "--wav-dir",
        default="/app/install/wake_vad_cpp/share/wake_vad_cpp/record_sound",
    )
    args = p.parse_args()

    cfg = BrainConfig()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.set_grad_enabled(False)
    if torch.cuda.is_available():
        torch.cuda.reset_peak_memory_stats()

    out: dict[str, object] = {
        "case": args.case,
        "device": str(device),
        "rss_mb_before": rss_mb(),
    }

    if args.case == "stt":
        wavs = sorted(Path(args.wav_dir).glob("*.wav"), key=lambda x: x.stat().st_mtime, reverse=True)
        if not wavs:
            raise FileNotFoundError(f"no_wavs:{args.wav_dir}")
        audio, sr = read_wav_mono_f32(wavs[0])
        stt = STTEngine(cfg, device)
        t0 = time.perf_counter()
        text = stt.transcribe(audio, sr)
        dt = (time.perf_counter() - t0) * 1000.0
        out["latency_ms"] = float(dt)
        out["text"] = text
        out["input"] = str(wavs[0])

    elif args.case == "intent":
        router = IntentRouter(cfg)
        texts = ["앞으로 가", "현재시간 알려줘", "앞에 뭐가 있어"]
        lat = []
        routes = []
        for q in texts:
            t0 = time.perf_counter()
            d = router.route(q)
            lat.append((time.perf_counter() - t0) * 1000.0)
            routes.append({"q": q, "route": d.route, "score": d.score})
        out["latency_ms"] = float(sum(lat) / len(lat))
        out["routes"] = routes

    elif args.case == "llm":
        llm = LLMEngine(cfg)
        t0 = time.perf_counter()
        resp, action = llm.generate("현재시간 알려줘")
        dt = (time.perf_counter() - t0) * 1000.0
        out["latency_ms"] = float(dt)
        out["response"] = resp
        out["action"] = action

    elif args.case == "vlm":
        vlm = VLMEngine(cfg, device)
        img = np.full((480, 640, 3), 127, dtype=np.uint8)
        t0 = time.perf_counter()
        resp, action = vlm.analyze(img, "앞에 무슨 물건이 있어?")
        dt = (time.perf_counter() - t0) * 1000.0
        out["latency_ms"] = float(dt)
        out["response"] = resp
        out["action"] = action
        out["vlm_ready"] = bool(getattr(vlm, "_ready", False))

    else:
        tts = TTSEngine(cfg, device)
        t0 = time.perf_counter()
        audio = tts.synthesize("안녕하세요. 테스트 중입니다.")
        dt = (time.perf_counter() - t0) * 1000.0
        out["latency_ms"] = float(dt)
        out["samples"] = int(audio.shape[0])

    if torch.cuda.is_available():
        torch.cuda.synchronize()
    out["rss_mb_after"] = rss_mb()
    out["cuda"] = cuda_mb()
    print(json.dumps(out, ensure_ascii=False))


if __name__ == "__main__":
    main()
