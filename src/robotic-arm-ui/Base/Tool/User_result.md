# jetrover_arm_moveit RViz-실로봇 동기화 가이드

`command.md` 기준으로, 바로 실행하기 쉽게 정리한 버전입니다.

## 빠른 실행 순서

1. 빌드
2. 터미널 A에서 `ros_robot_controller_cpp` 실행
3. 터미널 B에서 `jetrover_arm_moveit` 실행
4. 동기화 체크 명령으로 확인

## 사전 주의

- 작업 경로는 항상 `/home/ubuntu/rover_ws`
- `cd /home/ubuntu/rover_ws/src`에서 build/source 하지 않음
- RViz 동기화 점검 시 `use_fake_joint_states:=false` 고정

## 1) 빌드 (한 번만)

```bash
cd /home/ubuntu/rover_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select ros_robot_controller_msgs ros_robot_controller_cpp jetrover_arm_moveit
source /home/ubuntu/rover_ws/install/setup.bash
```

## 2) 터미널 A (하드웨어 컨트롤러)

```bash
cd /home/ubuntu/rover_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/rover_ws/install/setup.bash
ros2 launch ros_robot_controller_cpp ros_robot_controller_cpp.launch.py
```

## 3) 터미널 B (MoveIt + RViz)

```bash
cd /home/ubuntu/rover_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/rover_ws/install/setup.bash
ros2 launch jetrover_arm_moveit moveit_demo.launch.py use_fake_joint_states:=false
```

## 4) 동기화 체크 (새 터미널)

```bash
cd /home/ubuntu/rover_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/rover_ws/install/setup.bash
ros2 service list | grep /ros_robot_controller/bus_servo/get_state
ros2 topic hz /moveit_joint_states
ros2 topic echo /moveit_joint_states --once
```

정상 판정:
- `/ros_robot_controller/bus_servo/get_state` 서비스가 조회됨
- `/moveit_joint_states`가 주기적으로 publish 됨

## 5) 안 맞을 때 바로 보는 점검

```bash
cd /home/ubuntu/rover_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/rover_ws/install/setup.bash
ros2 node list | grep ros_robot_controller
ros2 node list | grep arm_servo_state_bridge
```

대표 경고 로그:
- `service not ready: /ros_robot_controller/bus_servo/get_state`
- `missing servo states for ids=...`

## 참고

- RViz 상태 동기화와 MoveIt `Execute`는 별개
- 실제 팔 `Execute`까지 하려면 `FollowJointTrajectory` action 서버가 추가로 필요
