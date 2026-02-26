#!/usr/bin/env bash
# STT ONNX 단독 지연시간 측정 (ROS2 환경 필요)
# 사용법: bash bench/stt_bench.sh
set -euo pipefail

INSTALL_DIR="$(cd "$(dirname "$0")/.." && pwd)/install"
ONNX_DIR="/home/ubuntu/models/moonshine"
WAV="/tmp/bench_test.wav"
RUNS=3

source "${INSTALL_DIR}/setup.bash"

echo "=== STT ONNX 지연시간 측정 ==="
echo "ONNX: ${ONNX_DIR}"
echo "WAV : ${WAV}"
echo ""

# stt_node 백그라운드 기동
ros2 run stt_cpp stt_node \
  --ros-args \
  -p onnx_encoder_path:="${ONNX_DIR}/encoder.onnx" \
  -p onnx_decoder_path:="${ONNX_DIR}/decoder.onnx" \
  -p vocab_json_path:="${ONNX_DIR}/vocab.json" \
  -p onnx_use_cuda:=false \
  2>&1 | grep -v "^\[INFO\].*param" &
STT_PID=$!

echo "stt_node PID: ${STT_PID}, 초기화 대기 (3초)..."
sleep 3

for i in $(seq 1 ${RUNS}); do
  echo -n "run ${i}/${RUNS}: "

  # /stt/result 리스너를 백그라운드에서 실행
  RESULT_FILE="$(mktemp /tmp/stt_result.XXXXXX)"
  ros2 topic echo /stt/result std_msgs/msg/String --once \
    > "${RESULT_FILE}" 2>/dev/null &
  ECHO_PID=$!

  # 타이밍 시작
  T_START=$(date +%s%N)

  # 오디오 경로 발행
  ros2 topic pub --once /wake_vad/audio_path std_msgs/msg/String \
    "{data: \"${WAV}\"}" > /dev/null 2>&1

  # 결과 대기 (최대 15초)
  WAITED=0
  while kill -0 "${ECHO_PID}" 2>/dev/null && [ "${WAITED}" -lt 150 ]; do
    sleep 0.1
    WAITED=$((WAITED + 1))
  done

  T_END=$(date +%s%N)
  ELAPSED_MS=$(( (T_END - T_START) / 1000000 ))

  kill "${ECHO_PID}" 2>/dev/null || true
  wait "${ECHO_PID}" 2>/dev/null || true

  RESULT=$(cat "${RESULT_FILE}" 2>/dev/null || echo "(없음)")
  rm -f "${RESULT_FILE}"

  # 전력 스냅샷
  POWER=$(tegrastats --interval 500 2>/dev/null | head -1 | \
    grep -oP 'VDD_IN \K\d+' || echo "N/A")

  echo "${ELAPSED_MS} ms  |  VDD_IN=${POWER}mW  |  ${RESULT:0:60}"
  sleep 1
done

kill "${STT_PID}" 2>/dev/null || true
wait "${STT_PID}" 2>/dev/null || true
echo ""
echo "STT 측정 완료"
