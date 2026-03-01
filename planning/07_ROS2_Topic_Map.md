# ROS2 Topic Map

## JetRover AI 아키텍처

**최종 통합 기술 설계 문서**  
버전 3.0 | 2026년 2월

---

## 센서 토픽

| 토픽명 | 타입 | 발행자 | 주파수 | 설명 |
|--------|------|--------|--------|------|
| `/scan` | sensor_msgs/LaserScan | rplidar_node | 15Hz | LiDAR 스캔 데이터 |
| `/camera/color/image_raw` | sensor_msgs/Image | dabai_dcw_driver | 30Hz | RGB 이미지 |
| `/camera/depth/image_raw` | sensor_msgs/Image | dabai_dcw_driver | 30Hz | 깊이 이미지 |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | dabai_dcw_driver | 30Hz | 칩메라 캘리브레이션 |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | dabai_dcw_driver | 30Hz | 깊이 캘리브레이션 |
| `/imu` | sensor_msgs/Imu | stm32_bridge | 50Hz | IMU 데이터 (가속도, 자이로) |
| `/imu/data_raw` | sensor_msgs/Imu | stm32_bridge | 50Hz | 보정되지 않은 IMU 데이터 |
| `/odom` | nav_msgs/Odometry | stm32_bridge | 50Hz | 휠 오도메트리 |
| `/odom/unfiltered` | nav_msgs/Odometry | stm32_bridge | 50Hz | EKF 전 오도메트리 |

### 센서 토픽 상세

#### /scan (LaserScan)
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 0}
  frame_id: "laser"
angle_min: 0.0
angle_max: 6.28318530718
angle_increment: 0.01745329252
time_increment: 0.0
scan_time: 0.0666667
range_min: 0.15
range_max: 12.0
ranges: [1.2, 1.25, 1.3, ...]  # 360개
intensities: [47.0, 47.0, ...]
```

#### /camera/color/image_raw (Image)
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 0}
  frame_id: "camera_color_optical_frame"
height: 480
width: 640
encoding: "rgb8"
is_bigendian: 0
step: 1920
data: [255, 0, 0, ...]  # 921600 bytes
```

---

## 상태 토픽

| 토픽명 | 타입 | 발행자 | 주파수 | 설명 |
|--------|------|--------|--------|------|
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster | 50Hz | 전체 관절 상태 |
| `/tf` | tf2_msgs/TFMessage | robot_state_publisher | 50Hz | 동적 좌표 변환 |
| `/tf_static` | tf2_msgs/TFMessage | robot_state_publisher | 정적 | 고정 좌표 변환 |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | slam_toolbox | 20Hz | 추정 위치 |
| `/robot_description` | std_msgs/String | robot_state_publisher | 정적 | URDF XML |

### 상태 토픽 상세

#### /joint_states (JointState)
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 0}
  frame_id: ""
name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_joint", "pan_joint", "tilt_joint"]
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

---

## 제어 토픽

| 토픽명 | 타입 | 발행자/구독자 | 주파수 | 설명 |
|--------|------|---------------|--------|------|
| `/cmd_vel` | geometry_msgs/Twist | Nav2 → ros2_control | 50Hz | 속도 명령 (linear.x, angular.z) |
| `/cmd_vel_smoothed` | geometry_msgs/Twist | velocity_smoother | 50Hz | 스물딩된 속도 |
| `/arm_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | MoveIt → controller | 50Hz | 팔 궤적 명령 |
| `/gripper_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | MoveIt → controller | 50Hz | 그리퍼 명령 |
| `/pan_tilt_cmd` | std_msgs/Float64MultiArray | pan_tilt_controller | 50Hz | 팬틸트 각도 명령 [pan, tilt] |
| `/bus_servo/set_position` | ros_robot_controller_msgs/ServosPosition | hiwonder_bridge → STM32 | 50Hz | 서보 위치 명령 |
| `/bus_servo/set_speed` | ros_robot_controller_msgs/ServosSpeed | hiwonder_bridge → STM32 | 50Hz | 서보 속도 명령 |

### 제어 토픽 상세

#### /cmd_vel (Twist)
```yaml
linear:
  x: 0.5   # 전진/후진 속도 (m/s)
  y: 0.0   # 좌/우 속도 (m/s) - Mecanum 전용
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3   # 회전 속도 (rad/s)
```

#### /pan_tilt_cmd (Float64MultiArray)
```yaml
layout:
  dim: [{label: "", size: 2, stride: 2}]
  data_offset: 0
data: [0.0, 0.0]  # [pan (rad), tilt (rad)]
```

#### /bus_servo/set_position (ServosPosition)
```yaml
servos:
  - id: 1
    position: 500
  - id: 2
    position: 500
  ...
```

---

## AI 토픽

| 토픽명 | 타입 | 발행자 | 주파수 | 설명 |
|--------|------|--------|--------|------|
| `/yolo/detections` | vision_msgs/Detection2DArray | yolo11n_trt_node | 15Hz | 객체 감지 결과 |
| `/yolo/annotated_image` | sensor_msgs/Image | yolo11n_trt_node | 15Hz | 바울딩 박스 표시 이미지 |
| `/intent` | std_msgs/String | intent_router | 이벤트 | 분류된 의도 |
| `/intent_confidence` | std_msgs/Float32 | intent_router | 이벤트 | 의도 신뢰도 |
| `/wake_word` | std_msgs/Bool | porcupine_node | 이벤트 | 웨이크워드 감지 |
| `/stt_result` | std_msgs/String | sherpa_onnx_node | 이벤트 | STT 결과 |
| `/stt_partial` | std_msgs/String | sherpa_onnx_node | 이벤트 | STT 중간 결과 |
| `/tts_request` | std_msgs/String | llama_cpp_server | 이벤트 | TTS 요청 |
| `/tts_status` | std_msgs/String | sherpa_onnx_node | 이벤트 | TTS 상태 (playing/done) |
| `/vlm_result` | std_msgs/String | moondream2_node | 이벤트 | VLM 분석 결과 |
| `/ocr_result` | std_msgs/String | rapidocr_node | 이벤트 | OCR 인식 결과 |

### AI 토픽 상세

#### /yolo/detections (Detection2DArray)
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 0}
  frame_id: "camera_color_optical_frame"
detections:
  - results:
      - hypothesis:
          class_id: 0
          class_name: "person"
          score: 0.95
    bbox:
      center: {x: 320.0, y: 240.0}
      size: {x: 100.0, y: 150.0}
```

#### /intent (String)
```yaml
data: "MOVE_TO"  # 또는 "PICK", "STOP", "SCAN_SCENE", 등
```

---

## 네비게이션 토픽

| 토픽명 | 타입 | 발행자/구독자 | 설명 |
|--------|------|---------------|------|
| `/goal_pose` | geometry_msgs/PoseStamped | 사용자 → Nav2 | 목표 위치 |
| `/plan` | nav_msgs/Path | Nav2 | 계획된 경로 |
| `/local_plan` | nav_msgs/Path | DWB | 로컬 경로 |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | 로컬 코스트맵 |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | 글로벌 코스트맵 |
| `/local_costmap/costmap_updates` | map_msgs/OccupancyGridUpdate | Nav2 | 로컬 코스트맵 업데이트 |
| `/global_costmap/costmap_updates` | map_msgs/OccupancyGridUpdate | Nav2 | 글로벌 코스트맵 업데이트 |
| `/particlecloud` | geometry_msgs/PoseArray | slam_toolbox | 파티클 클라우드 |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox | 전체 맵 |
| `/map_metadata` | nav_msgs/MapMetaData | slam_toolbox | 맵 메타데이터 |

### 네비게이션 토픽 상세

#### /plan (Path)
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 0}
  frame_id: "map"
poses:
  - header: {stamp: ..., frame_id: "map"}
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  - header: {stamp: ..., frame_id: "map"}
    pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  ...
```

#### /global_costmap/costmap (OccupancyGrid)
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 0}
  frame_id: "map"
info:
  map_load_time: {sec: 0, nanosec: 0}
  resolution: 0.05
  width: 384
  height: 384
  origin: {position: {x: -10.0, y: -10.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
data: [0, 0, 0, 100, 100, ...]  # -1: unknown, 0: free, 100: occupied
```

---

## 진단 토픽

| 토픽명 | 타입 | 발행자 | 주파수 | 설명 |
|--------|------|--------|--------|------|
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | 다양한 노드 | 1Hz | 시스템 진단 정보 |
| `/system_status` | std_msgs/String | heartbeat_monitor | 1Hz | 시스템 상태 요약 |
| `/cpu_usage` | std_msgs/Float32 | system_monitor | 1Hz | CPU 사용률 |
| `/memory_usage` | std_msgs/Float32 | system_monitor | 1Hz | 메모리 사용률 |
| `/gpu_usage` | std_msgs/Float32 | system_monitor | 1Hz | GPU 사용률 |
| `/temperature` | sensor_msgs/Temperature | system_monitor | 1Hz | SoC 온도 |

---

## 토픽 흐름도

```
[센서 레이어]
    │
    ├──> /scan ────────────┐
    ├──> /camera/... ──────┤
    ├──> /imu ─────────────┤
    └──> /odom ────────────┤
                           ▼
[처리 레이어]
    │
    ├──> slam_toolbox ───> /amcl_pose, /map
    ├──> yolo11n ────────> /yolo/detections
    ├──> porcupine ──────> /wake_word
    └──> sherpa-onnx ────> /stt_result
                           │
                           ▼
[의도 레이어]
    │
    └──> intent_router ──> /intent
                           │
                           ▼
[액션 레이어]
    │
    ├──> Nav2 ───────────> /cmd_vel
    ├──> MoveIt2 ────────> /arm_controller/joint_trajectory
    └──> llama.cpp ──────> /tts_request
                           │
                           ▼
[출력 레이어]
    │
    ├──> ros2_control ───> /joint_states
    ├──> hiwonder_bridge -> /bus_servo/set_position
    └──> sherpa-onnx ────> TTS 오디오
```

---

*JetRover AI Architecture v3.0 | Jetson Orin Nano 8GB + ROS2 Humble + 6DOF Mecanum*
