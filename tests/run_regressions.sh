#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/tests/logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_LOG="${LOG_DIR}/regression_${STAMP}.log"

mkdir -p "${LOG_DIR}"
touch "${RUN_LOG}"

log() {
  echo "[$(date +'%F %T')] $*" | tee -a "${RUN_LOG}"
}

run_show_args() {
  local name="$1"
  shift
  log "START: ${name}"
  if "$@" >>"${RUN_LOG}" 2>&1; then
    log "PASS:  ${name}"
  else
    log "FAIL:  ${name}"
    return 1
  fi
}

if ! command -v ros2 >/dev/null 2>&1; then
  log "FAIL: ros2 command not found. Source ROS2 environment first."
  exit 1
fi

log "Regression run started"
log "Log file: ${RUN_LOG}"

run_show_args "Case1 ros_robot_controller launch syntax" \
  ros2 launch ros_robot_controller_cpp ros_robot_controller_cpp.launch.py --show-args

run_show_args "Case2 moveit launch syntax" \
  ros2 launch jetrover_arm_moveit moveit_demo.launch.py --show-args

run_show_args "Case3 voice pipeline launch syntax" \
  ros2 launch tts_cpp voice_pipeline_with_tts.launch.py --show-args

log "All regression syntax checks passed"
