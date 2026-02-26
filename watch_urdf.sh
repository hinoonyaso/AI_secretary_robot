#!/bin/bash
# xacro 파일 변경 감지 → moveit_demo 자동 재시작

WORKSPACE="/home/ubuntu/rover_ws"
URDF_DIR="$WORKSPACE/src/jetrover_arm_moveit/urdf"
LAUNCH_PKG="jetrover_arm_moveit"
LAUNCH_FILE="moveit_demo.launch.py"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "xacro 파일 감시 시작: $URDF_DIR"
echo "변경 감지 시 $LAUNCH_FILE 자동 재시작"

# 초기 실행
ros2 launch $LAUNCH_PKG $LAUNCH_FILE &
LAUNCH_PID=$!

while inotifywait -e modify -r "$URDF_DIR"; do
  echo ""
  echo "변경 감지! 재시작 중..."
  kill $LAUNCH_PID 2>/dev/null
  sleep 1
  ros2 launch $LAUNCH_PKG $LAUNCH_FILE &
  LAUNCH_PID=$!
  echo "재시작 완료 (PID: $LAUNCH_PID)"
done
