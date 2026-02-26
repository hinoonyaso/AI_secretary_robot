#!/usr/bin/env python3
"""
Jarvis 파이프라인 지연시간 + 전력 종합 벤치마크
측정 항목:
  1) 아이들 전력 기준선
  2) STT  : ONNX Runtime C++ (stt_node via ROS2 topic)
  3) LLM  : ollama HTTP API
  4) TTS  : melo_tts_server HTTP API  (PCM 수신까지)
  5) 엔드-투-엔드 : LLM 응답 → TTS PCM 수신
측정 도구: tegrastats (VDD_IN, VDD_CPU_GPU_CV, VDD_SOC)
"""
import argparse
import json
import math
import os
import re
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

# ──────────────────────────────────────────────────
# 설정
# ──────────────────────────────────────────────────
WORKSPACE   = Path("/home/ubuntu/rover_ws")
INSTALL_DIR = WORKSPACE / "install"
ONNX_DIR    = Path("/home/ubuntu/models/moonshine")
MELO_SCRIPT = INSTALL_DIR / "tts_cpp/share/tts_cpp/scripts/melo_tts_server.py"
MELO_URL    = "http://127.0.0.1:5500"
OLLAMA_URL  = "http://127.0.0.1:11434"
OLLAMA_MODEL = "qwen2.5:1.5b"
SAMPLE_RATE  = 16000           # STT 입력 Hz
BENCH_AUDIO_SECS = 3.0         # 합성 오디오 길이 (초)
LLM_PROMPT  = "안녕하세요. 오늘 날씨 어때요?"
TTS_TEXT    = "안녕하세요. 오늘은 맑고 따뜻한 날씨가 예상됩니다."
TEGRA_MS    = 200              # tegrastats 샘플 간격 (ms)
STT_NODE_TIMEOUT = 15.0        # stt_node 응답 대기 최대 시간 (초)
LLM_RUNS    = 3
TTS_RUNS    = 5
IDLE_SECS   = 5.0              # 아이들 전력 측정 시간

# ──────────────────────────────────────────────────
# tegrastats 전력 모니터
# ──────────────────────────────────────────────────
class PowerMonitor:
    _PATTERN = re.compile(
        r"VDD_IN\s+(\d+)mW.*?VDD_CPU_GPU_CV\s+(\d+)mW.*?VDD_SOC\s+(\d+)mW"
    )

    def __init__(self, interval_ms: int = 200):
        self._interval = interval_ms
        self._samples: list[dict] = []
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._running = False

    def start(self):
        self._samples.clear()
        self._running = True
        self._proc = subprocess.Popen(
            ["tegrastats", "--interval", str(self._interval)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def stop(self) -> dict:
        self._running = False
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._thread:
            self._thread.join(timeout=3)
        return self._summarize()

    def _reader(self):
        for line in self._proc.stdout:
            if not self._running:
                break
            m = self._PATTERN.search(line)
            if m:
                with self._lock:
                    self._samples.append({
                        "vdd_in":  int(m.group(1)),
                        "cpu_gpu": int(m.group(2)),
                        "soc":     int(m.group(3)),
                    })

    def _summarize(self) -> dict:
        with self._lock:
            s = self._samples.copy()
        if not s:
            return {"n": 0, "vdd_in_mean": 0, "vdd_in_max": 0,
                    "cpu_gpu_mean": 0, "soc_mean": 0}
        return {
            "n":           len(s),
            "vdd_in_mean": int(sum(x["vdd_in"]  for x in s) / len(s)),
            "vdd_in_max":  max(x["vdd_in"]       for x in s),
            "cpu_gpu_mean":int(sum(x["cpu_gpu"]  for x in s) / len(s)),
            "soc_mean":    int(sum(x["soc"]      for x in s) / len(s)),
        }


# ──────────────────────────────────────────────────
# 합성 16kHz WAV 생성 (무음 + 짧은 tone)
# ──────────────────────────────────────────────────
def make_test_wav(path: Path, duration_sec: float = BENCH_AUDIO_SECS):
    """400Hz 사인파 WAV (16bit, mono, 16kHz) 생성"""
    n_samples = int(SAMPLE_RATE * duration_sec)
    pcm = bytearray()
    for i in range(n_samples):
        v = int(8000 * math.sin(2 * math.pi * 400 * i / SAMPLE_RATE))
        v = max(-32768, min(32767, v))
        pcm.extend(struct.pack("<h", v))

    with open(path, "wb") as f:
        data_size = len(pcm)
        # RIFF header
        f.write(b"RIFF")
        f.write(struct.pack("<I", 36 + data_size))
        f.write(b"WAVE")
        # fmt chunk
        f.write(b"fmt ")
        f.write(struct.pack("<I", 16))
        f.write(struct.pack("<H", 1))           # PCM
        f.write(struct.pack("<H", 1))           # mono
        f.write(struct.pack("<I", SAMPLE_RATE))
        f.write(struct.pack("<I", SAMPLE_RATE * 2))
        f.write(struct.pack("<H", 2))           # block align
        f.write(struct.pack("<H", 16))          # bits per sample
        # data chunk
        f.write(b"data")
        f.write(struct.pack("<I", data_size))
        f.write(pcm)


# ──────────────────────────────────────────────────
# HTTP 헬퍼
# ──────────────────────────────────────────────────
def http_post_json(url: str, payload: dict, timeout: float = 30.0) -> tuple[int, bytes, dict]:
    body = json.dumps(payload).encode()
    req = urllib.request.Request(
        url, data=body,
        headers={"Content-Type": "application/json", "Content-Length": str(len(body))},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read(), dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), {}
    except Exception as e:
        return 0, str(e).encode(), {}


def http_get(url: str, timeout: float = 5.0) -> int:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status
    except Exception:
        return 0


# ──────────────────────────────────────────────────
# MeloTTS 서버 관리
# ──────────────────────────────────────────────────
class MeloServer:
    def __init__(self, script: Path):
        self._script = script
        self._proc: subprocess.Popen | None = None

    def start(self, wait_secs: float = 20.0) -> bool:
        if http_get(f"{MELO_URL}/health") == 200:
            print("  [melo] 이미 실행 중")
            return True
        print(f"  [melo] 서버 기동 중... ({self._script})")
        self._proc = subprocess.Popen(
            ["python3", str(self._script),
             "--host", "127.0.0.1", "--port", "5500",
             "--language", "KR", "--device", "auto"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True,
        )
        deadline = time.time() + wait_secs
        while time.time() < deadline:
            time.sleep(0.5)
            if http_get(f"{MELO_URL}/health") == 200:
                print("  [melo] 준비 완료")
                return True
            if self._proc.poll() is not None:
                out = self._proc.stdout.read()
                print(f"  [melo] 프로세스 종료됨:\n{out}")
                return False
        print("  [melo] 타임아웃")
        return False

    def stop(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()


# ──────────────────────────────────────────────────
# 벤치마크 섹션
# ──────────────────────────────────────────────────
def bench_idle(pm: PowerMonitor) -> dict:
    print(f"\n[1] 아이들 전력 측정 ({IDLE_SECS}초)...")
    pm.start()
    time.sleep(IDLE_SECS)
    pwr = pm.stop()
    print(f"    VDD_IN: {pwr['vdd_in_mean']}mW (avg), {pwr['vdd_in_max']}mW (peak)")
    return pwr


def bench_stt(pm: PowerMonitor, wav_path: Path) -> dict:
    """
    stt_node를 ROS2로 기동 후 /wake_vad/audio_path 토픽을 발행하여
    /stt/result 를 수신할 때까지의 지연시간 측정.
    ONNX 경로 파라미터를 직접 전달.
    """
    print("\n[2] STT (ONNX Runtime C++) 지연시간 측정...")

    ros_setup = f"source {INSTALL_DIR}/setup.bash"
    encoder = ONNX_DIR / "encoder.onnx"
    decoder = ONNX_DIR / "decoder.onnx"
    vocab   = ONNX_DIR / "vocab.json"

    if not encoder.exists():
        print(f"  [stt] ONNX 파일 없음: {encoder}")
        return {"error": "onnx_missing"}

    # stt_node 실행 + 결과 캡처
    stt_cmd = (
        f"{ros_setup} && "
        f"ros2 run stt_cpp stt_node "
        f"--ros-args "
        f"-p onnx_encoder_path:={encoder} "
        f"-p onnx_decoder_path:={decoder} "
        f"-p vocab_json_path:={vocab} "
        f"-p onnx_use_cuda:=false"  # 초기 테스트: CPU
    )

    latencies = []
    errors = []
    RUNS = 2  # stt_node 기동 오버헤드 있어 2회

    for i in range(RUNS):
        print(f"  run {i+1}/{RUNS} ...", end=" ", flush=True)

        # stt_node 기동
        stt_proc = subprocess.Popen(
            ["bash", "-c", stt_cmd],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        time.sleep(2.5)  # 노드 초기화 대기

        # /stt/result 리스너 기동
        echo_proc = subprocess.Popen(
            ["bash", "-c",
             f"{ros_setup} && ros2 topic echo /stt/result std_msgs/msg/String --once"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )

        # /wake_vad/audio_path 발행
        t0 = time.perf_counter()
        pm.start()
        pub_proc = subprocess.Popen(
            ["bash", "-c",
             f"{ros_setup} && ros2 topic pub --once /wake_vad/audio_path "
             f"std_msgs/msg/String '{{data: \"{wav_path}\"}}'"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

        # 결과 대기
        try:
            stdout, _ = echo_proc.communicate(timeout=STT_NODE_TIMEOUT)
            elapsed = time.perf_counter() - t0
            pwr = pm.stop()
            if "data:" in stdout:
                text = stdout.strip()
                print(f"{elapsed*1000:.0f}ms  →  {text[:60]}")
                latencies.append(elapsed * 1000)
            else:
                print(f"응답 없음 ({elapsed*1000:.0f}ms)")
                errors.append("no_result")
                pm.stop()
        except subprocess.TimeoutExpired:
            pwr = pm.stop()
            print(f"타임아웃")
            errors.append("timeout")
            echo_proc.kill()

        pub_proc.wait(timeout=3)
        stt_proc.terminate()
        try:
            stt_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            stt_proc.kill()
        time.sleep(1)

    result = {"runs": RUNS, "latency_ms": latencies, "errors": errors,
              "power": pwr if latencies else {}}
    if latencies:
        result["mean_ms"] = sum(latencies) / len(latencies)
        result["min_ms"]  = min(latencies)
    return result


def bench_llm(pm: PowerMonitor) -> dict:
    print(f"\n[3] LLM (ollama / {OLLAMA_MODEL}) 지연시간 측정 ({LLM_RUNS}회)...")

    latencies_first = []
    latencies_total = []
    powers = []

    for i in range(LLM_RUNS):
        print(f"  run {i+1}/{LLM_RUNS} ...", end=" ", flush=True)
        pm.start()
        t0 = time.perf_counter()

        status, body, _ = http_post_json(
            f"{OLLAMA_URL}/api/generate",
            {"model": OLLAMA_MODEL, "prompt": LLM_PROMPT, "stream": False},
            timeout=60.0,
        )
        t_total = time.perf_counter() - t0
        pwr = pm.stop()
        powers.append(pwr)

        if status == 200:
            try:
                resp = json.loads(body)
                answer = resp.get("response", "")[:50]
                eval_dur = resp.get("eval_duration", 0) / 1e9      # ns→s
                prompt_dur = resp.get("prompt_eval_duration", 0) / 1e9
                eval_count = resp.get("eval_count", 0)
                tok_per_sec = eval_count / eval_dur if eval_dur > 0 else 0
                print(f"total={t_total*1000:.0f}ms  tok/s={tok_per_sec:.1f}  →  {answer}")
                latencies_total.append(t_total * 1000)
                latencies_first.append(prompt_dur * 1000)
            except Exception as e:
                print(f"파싱 오류: {e}")
        else:
            print(f"HTTP {status}")
        time.sleep(1)

    result = {"runs": LLM_RUNS, "latency_total_ms": latencies_total,
              "prompt_ms": latencies_first}
    if latencies_total:
        result["mean_total_ms"] = sum(latencies_total) / len(latencies_total)
        result["min_total_ms"]  = min(latencies_total)
        result["mean_vdd_in_mw"] = int(
            sum(p["vdd_in_mean"] for p in powers) / len(powers))
        result["peak_vdd_in_mw"] = max(p["vdd_in_max"] for p in powers)
    return result


def bench_tts(pm: PowerMonitor) -> dict:
    print(f"\n[4] TTS (melo_tts_server PCM) 지연시간 측정 ({TTS_RUNS}회)...")

    latencies = []
    pcm_bytes_list = []
    powers = []

    for i in range(TTS_RUNS):
        print(f"  run {i+1}/{TTS_RUNS} ...", end=" ", flush=True)
        pm.start()
        t0 = time.perf_counter()

        status, body, headers = http_post_json(
            f"{MELO_URL}/synthesize",
            {"text": TTS_TEXT, "speed": 1.0},
            timeout=30.0,
        )
        elapsed = time.perf_counter() - t0
        pwr = pm.stop()
        powers.append(pwr)

        if status == 200:
            sr = int(headers.get("X-Sample-Rate", "44100"))
            n_samples = len(body) // 2
            duration_ms = n_samples / sr * 1000
            print(f"{elapsed*1000:.0f}ms  →  {n_samples}샘플 @ {sr}Hz ({duration_ms:.0f}ms 오디오)")
            latencies.append(elapsed * 1000)
            pcm_bytes_list.append(len(body))
        else:
            print(f"HTTP {status}: {body[:80]}")
        time.sleep(0.5)

    result = {"runs": TTS_RUNS, "latency_ms": latencies}
    if latencies:
        result["mean_ms"]  = sum(latencies) / len(latencies)
        result["min_ms"]   = min(latencies)
        result["max_ms"]   = max(latencies)
        result["mean_pcm_bytes"] = int(sum(pcm_bytes_list) / len(pcm_bytes_list))
        result["mean_vdd_in_mw"] = int(
            sum(p["vdd_in_mean"] for p in powers) / len(powers))
        result["peak_vdd_in_mw"] = max(p["vdd_in_max"] for p in powers)
    return result


def bench_llm_tts_pipeline(pm: PowerMonitor) -> dict:
    """LLM 응답 → TTS PCM 수신 엔드-투-엔드 지연시간"""
    print(f"\n[5] LLM→TTS 엔드-투-엔드 지연시간 측정 (3회)...")

    latencies = []
    powers = []

    for i in range(3):
        print(f"  run {i+1}/3 ...", end=" ", flush=True)
        pm.start()
        t0 = time.perf_counter()

        # LLM
        status, body, _ = http_post_json(
            f"{OLLAMA_URL}/api/generate",
            {"model": OLLAMA_MODEL, "prompt": LLM_PROMPT, "stream": False},
            timeout=60.0,
        )
        t_llm = time.perf_counter()
        if status != 200:
            print(f"LLM HTTP {status}")
            pm.stop()
            continue
        llm_resp = json.loads(body).get("response", "")[:100]

        # TTS
        status2, body2, headers2 = http_post_json(
            f"{MELO_URL}/synthesize",
            {"text": llm_resp, "speed": 1.0},
            timeout=30.0,
        )
        t_end = time.perf_counter()
        pwr = pm.stop()
        powers.append(pwr)

        if status2 == 200:
            total_ms = (t_end - t0) * 1000
            llm_ms   = (t_llm - t0) * 1000
            tts_ms   = (t_end - t_llm) * 1000
            print(f"총={total_ms:.0f}ms  (LLM={llm_ms:.0f}ms + TTS={tts_ms:.0f}ms)")
            latencies.append({"total": total_ms, "llm": llm_ms, "tts": tts_ms})
        else:
            print(f"TTS HTTP {status2}")
        time.sleep(1)

    result = {"latencies": latencies}
    if latencies:
        result["mean_total_ms"] = sum(x["total"] for x in latencies) / len(latencies)
        result["mean_llm_ms"]   = sum(x["llm"]   for x in latencies) / len(latencies)
        result["mean_tts_ms"]   = sum(x["tts"]   for x in latencies) / len(latencies)
        result["mean_vdd_in_mw"] = int(
            sum(p["vdd_in_mean"] for p in powers) / len(powers)) if powers else 0
        result["peak_vdd_in_mw"] = max(p["vdd_in_max"] for p in powers) if powers else 0
    return result


# ──────────────────────────────────────────────────
# 리포트 출력
# ──────────────────────────────────────────────────
def print_report(idle, stt, llm, tts, e2e):
    print("\n" + "=" * 60)
    print("  Jarvis 파이프라인 벤치마크 결과")
    print("=" * 60)

    print(f"\n[전력 - 아이들]")
    print(f"  VDD_IN    : {idle['vdd_in_mean']:5d} mW (avg)  {idle['vdd_in_max']:5d} mW (peak)")
    print(f"  CPU/GPU/CV: {idle['cpu_gpu_mean']:5d} mW (avg)")
    print(f"  SOC       : {idle['soc_mean']:5d} mW (avg)")

    print(f"\n[STT - ONNX Runtime C++]")
    if "error" in stt:
        print(f"  오류: {stt['error']}")
    elif stt.get("latency_ms"):
        for j, ms in enumerate(stt["latency_ms"]):
            print(f"  run{j+1}: {ms:.0f} ms")
        print(f"  평균 : {stt['mean_ms']:.0f} ms")
        if stt.get("power"):
            p = stt["power"]
            print(f"  전력  : {p.get('vdd_in_mean',0)} mW (avg)  {p.get('vdd_in_max',0)} mW (peak)")
    else:
        print(f"  측정 실패 (errors: {stt.get('errors', [])})")
        print(f"  ※ ROS2 환경이 소싱되지 않았거나 stt_node 초기화 실패")
        print(f"     → 아래 수동 측정 방법으로 확인하세요")

    print(f"\n[LLM - ollama {OLLAMA_MODEL}]")
    if stt_latencies := llm.get("latency_total_ms"):
        print(f"  총 응답 시간 : {llm['mean_total_ms']:.0f} ms (avg)  {llm['min_total_ms']:.0f} ms (min)")
        for j, ms in enumerate(stt_latencies):
            print(f"  run{j+1}: {ms:.0f} ms")
        print(f"  전력: {llm.get('mean_vdd_in_mw',0)} mW (avg)  {llm.get('peak_vdd_in_mw',0)} mW (peak)")
    else:
        print("  측정 실패")

    print(f"\n[TTS - MeloTTS 상주 서버 (PCM)]")
    if tts.get("latency_ms"):
        print(f"  평균 : {tts['mean_ms']:.0f} ms   최소: {tts['min_ms']:.0f} ms   최대: {tts['max_ms']:.0f} ms")
        print(f"  PCM  : 평균 {tts.get('mean_pcm_bytes',0)//1024} KB")
        print(f"  전력  : {tts.get('mean_vdd_in_mw',0)} mW (avg)  {tts.get('peak_vdd_in_mw',0)} mW (peak)")
        for j, ms in enumerate(tts["latency_ms"]):
            print(f"  run{j+1}: {ms:.0f} ms")
    else:
        print("  측정 실패")

    print(f"\n[LLM→TTS 엔드-투-엔드]")
    if e2e.get("latencies"):
        print(f"  총 평균  : {e2e['mean_total_ms']:.0f} ms")
        print(f"    LLM   : {e2e['mean_llm_ms']:.0f} ms")
        print(f"    TTS   : {e2e['mean_tts_ms']:.0f} ms")
        print(f"  전력     : {e2e.get('mean_vdd_in_mw',0)} mW (avg)  {e2e.get('peak_vdd_in_mw',0)} mW (peak)")
    else:
        print("  측정 실패")

    print(f"\n[STT 수동 측정 가이드]")
    print(f"  # 터미널 A:")
    print(f"  source {INSTALL_DIR}/setup.bash")
    print(f"  ros2 run stt_cpp stt_node --ros-args \\")
    print(f"    -p onnx_encoder_path:={ONNX_DIR}/encoder.onnx \\")
    print(f"    -p onnx_decoder_path:={ONNX_DIR}/decoder.onnx \\")
    print(f"    -p vocab_json_path:={ONNX_DIR}/vocab.json \\")
    print(f"    -p onnx_use_cuda:=false")
    print(f"  # 터미널 B (결과 확인):")
    print(f"  source {INSTALL_DIR}/setup.bash && ros2 topic echo /stt/result")
    print(f"  # 터미널 C (오디오 발행):")
    print(f"  source {INSTALL_DIR}/setup.bash && \\")
    print(f"  ros2 topic pub --once /wake_vad/audio_path std_msgs/msg/String \\")
    print(f"    '{{data: \"/tmp/bench_test.wav\"}}'")

    print("\n" + "=" * 60)

    # JSON 저장
    out = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "idle_power": idle,
        "stt": stt,
        "llm": llm,
        "tts": tts,
        "e2e_llm_tts": e2e,
    }
    out_path = Path("/tmp/jarvis_bench_result.json")
    out_path.write_text(json.dumps(out, indent=2, ensure_ascii=False))
    print(f"  JSON 결과 저장: {out_path}")


# ──────────────────────────────────────────────────
# main
# ──────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-stt",  action="store_true", help="STT 측정 건너뜀")
    parser.add_argument("--skip-llm",  action="store_true", help="LLM 측정 건너뜀")
    parser.add_argument("--skip-tts",  action="store_true", help="TTS 측정 건너뜀")
    parser.add_argument("--no-melo-start", action="store_true", help="MeloTTS 서버 자동 기동 안 함")
    args = parser.parse_args()

    pm = PowerMonitor(TEGRA_MS)
    melo_server = MeloServer(MELO_SCRIPT)

    # 테스트 WAV 생성
    wav_path = Path("/tmp/bench_test.wav")
    make_test_wav(wav_path)
    print(f"테스트 WAV: {wav_path} ({BENCH_AUDIO_SECS}초, 16kHz mono)")

    # MeloTTS 서버 기동
    melo_ok = False
    if not args.skip_tts and not args.no_melo_start:
        print("\n[MeloTTS 서버 준비]")
        melo_ok = melo_server.start(wait_secs=30)

    # 측정
    idle = bench_idle(pm)
    stt  = bench_stt(pm, wav_path) if not args.skip_stt else {"skipped": True}
    llm  = bench_llm(pm)           if not args.skip_llm else {"skipped": True}
    tts  = bench_tts(pm)           if (not args.skip_tts and melo_ok) else {"skipped": True, "reason": "melo_not_running" if not melo_ok else "skip_flag"}
    e2e  = bench_llm_tts_pipeline(pm) if (not args.skip_tts and not args.skip_llm and melo_ok) else {"skipped": True}

    # MeloTTS 서버 종료
    if not args.no_melo_start:
        melo_server.stop()

    print_report(idle, stt, llm, tts, e2e)


if __name__ == "__main__":
    main()
