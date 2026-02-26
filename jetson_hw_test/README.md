# Jetson Orin Nano 하드웨어 기능 테스트 (JetRover 기준)

문서 기준으로 다음 하드웨어 테스트를 자동화했습니다.

- 센서/보드: STM32 보드 연결, I2C(MPU6050), RPLIDAR A1(시리얼/ROS), 카메라(UVC/Orbbec USB), 마이크 장치 탐지
- 액추에이터: ROS2 토픽 기반 `cmd_vel`, 그리퍼, 암 조인트 테스트 (기본값은 안전한 dry-run, gripper/arm optional)

## 파일

- `sensor_board_test.py`: 비침습 센서/보드 점검
- `actuator_ros2_test.py`: ROS2 액추에이터 점검

## 사전 준비

```bash
sudo apt update
sudo apt install -y i2c-tools python3-pip
pip3 install pyserial opencv-python
```

ROS2 액추에이터 테스트 시:

```bash
source /opt/ros/humble/setup.bash
# 필요하면 프로젝트 setup.bash도 source
```

## 1) 센서/보드 테스트 실행

```bash
cd /home/ubuntu/rover_ws/jetson_hw_test
python3 sensor_board_test.py
```

옵션:

```bash
python3 sensor_board_test.py --i2c-buses 0,1,7
python3 sensor_board_test.py --lidar-port /dev/ttyUSB0
python3 sensor_board_test.py --skip-camera
python3 sensor_board_test.py --skip-i2c --lidar-mode ros
```

JetRover 권장:

```bash
# STM32 경유 IMU/라이다를 사용하는 경우
python3 sensor_board_test.py --skip-i2c --lidar-mode auto
```

## 2) 액추에이터(ROS2) 테스트 실행

기본(dry-run, 실제 구동 안 함):

```bash
cd /home/ubuntu/rover_ws/jetson_hw_test
python3 actuator_ros2_test.py
```

실제 구동(저속, 짧은 시간):

```bash
python3 actuator_ros2_test.py --enable-motion --linear-x 0.08 --seconds 1.0
```

토픽명이 다른 경우:

```bash
python3 actuator_ros2_test.py \
  --enable-motion \
  --cmd-vel-topic /cmd_vel \
  --gripper-topic /gripper/command \
  --arm-joint-topic /arm/joint1/command
```

gripper/arm 토픽을 필수 검증하려면:

```bash
python3 actuator_ros2_test.py --require-gripper --require-arm
```

## 주의

- `--enable-motion`은 실제로 차체/암/그리퍼를 움직일 수 있습니다.
- 반드시 바퀴 공회전 가능한 안전 환경에서 실행하세요.
- 테스트 종료 시 스크립트가 정지 명령(`cmd_vel=0`)을 전송합니다.
