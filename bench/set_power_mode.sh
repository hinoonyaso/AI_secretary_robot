#!/usr/bin/env bash
# Jarvis 전력 모드 전환 스크립트
# 사용법: bash bench/set_power_mode.sh [performance|balanced|economy|restore]
set -euo pipefail

MODE="${1:-balanced}"

set_cpu_max_freq() {
  local freq="$1"
  for i in 0 1 2 3 4 5; do
    local path="/sys/devices/system/cpu/cpu${i}/cpufreq/scaling_max_freq"
    if [ -f "$path" ]; then
      echo "$freq" | sudo tee "$path" > /dev/null
    fi
  done
  echo "CPU max_freq → ${freq} Hz ($(echo "$freq / 1000" | bc) MHz)"
}

show_current() {
  local freq
  freq=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo "N/A")
  local nvmode
  nvmode=$(nvpmodel -q 2>/dev/null | head -1 || echo "N/A")
  echo "현재: ${nvmode} | CPU max ${freq} Hz"
}

case "$MODE" in
  performance)
    echo "[performance] 1728MHz, 6코어 - 최고 성능"
    set_cpu_max_freq 1728000
    ;;
  balanced)
    echo "[balanced] 1190MHz, 6코어 - 전력/성능 균형 (권장)"
    set_cpu_max_freq 1190400
    ;;
  economy)
    echo "[economy] 960MHz, 6코어 - 저전력"
    set_cpu_max_freq 960000
    ;;
  jetson_7w)
    echo "[jetson_7w] nvpmodel 7W 모드 (4코어 960MHz)"
    sudo nvpmodel -m 1
    ;;
  jetson_15w)
    echo "[jetson_15w] nvpmodel 15W 모드 (6코어 1728MHz)"
    sudo nvpmodel -m 0
    ;;
  restore)
    echo "[restore] 1728MHz 복원"
    set_cpu_max_freq 1728000
    ;;
  *)
    echo "사용법: $0 [performance|balanced|economy|jetson_7w|jetson_15w|restore]"
    exit 1
    ;;
esac

show_current

# 전력 측정 (2초)
echo "전력 측정 중..."
timeout 3 tegrastats --interval 500 2>/dev/null | head -3 | \
  grep -oP 'VDD_IN \K\d+' | \
  awk '{sum+=$1; n++} END{if(n>0)printf "VDD_IN idle avg: %dmW\n", sum/n}'
