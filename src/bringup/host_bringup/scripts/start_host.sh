#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-mapping}"  # mapping | localization
USE_LOCALIZATION="false"
if [ "${MODE}" = "localization" ]; then
  USE_LOCALIZATION="true"
fi

source /opt/ros/humble/setup.bash
source /home/ubuntu/AI_secretary_robot/install/setup.bash

ros2 launch host_bringup host_bringup_main.launch.py \
  use_slam:=true \
  use_localization:=${USE_LOCALIZATION} \
  use_nav2:=true \
  use_arm:=false \
  use_camera:=true \
  use_monitor:=true
