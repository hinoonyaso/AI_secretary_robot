# Robotic Arm UI + C++ Bridge

## 1) Web UI 실행

```bash
cd /home/sang/dev_ws/robotic-arm-ui
corepack enable
corepack prepare pnpm@latest --activate
pnpm install
pnpm dev
```

브라우저: `http://localhost:3000`

## 2) C++ ROS2 브리지 빌드/실행

```bash
cd /home/sang/dev_ws/robotic-arm-ui/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetrover_web_bridge
source install/setup.bash
ros2 run jetrover_web_bridge udp_servo_bridge_node
```

기본 동작:
- UDP `0.0.0.0:9999` 수신
- ROS2 토픽 `/ros_robot_controller/bus_servo/set_position` 발행 (`ros_robot_controller_msgs/msg/ServosPosition`)
- 매핑:
  - `servo_id` -> `position[].id` (0~255 clamp)
  - `value` -> `position[].position` (0~1000 clamp)
  - `duration_ms` -> `duration` (초 단위 float64로 변환)

## 3) UI ↔ 브리지 연결 설정 (옵션)

필요 시 환경변수로 UDP 목적지 변경:

```bash
export ROBOT_UDP_HOST=127.0.0.1
export ROBOT_UDP_PORT=9999
pnpm dev
```

## 4) 명령 흐름

- 사용자가 슬라이더를 조작하면 UI가 `/api/servo-command`로 POST
- Next API가 UDP JSON 전송
- C++ ROS2 노드가 수신 후 `/ros_robot_controller/bus_servo/set_position`로 publish
