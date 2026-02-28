# 🖥️ JetRover Host 부분 구현 현황 분석

> **날짜**: 2026-02-27
> **분석 대상**: Host (Ubuntu 22.04 네이티브) 환경의 ROS2 패키지 및 시스템 구성
> **참조**: plan.md section 6.1 Host-Brain 분리 구조

---

## 📋 Host의 역할 (plan.md 기준)

| 영역 | 실행 환경 | 역할 |
|:---|:---|:---|
| **Host** | Ubuntu 22.04 (네이티브) | ROS2, Nav2, SLAM, 모터/서보 제어, E-Stop |
| **Brain** | Docker Container (GPU) | AI 추론 (LLM, VLM, STT, TTS, YOLO, OCR) |

> **설계 원칙**: AI 컨테이너(Brain)가 OOM/크래시되어도 로봇 제어(Host)는 절대 멈추지 않는다.

---

## ✅ Host에 구현된 부분

### **1. ROS2 Humble 기본 패키지**
- ✅ `ros-humble-ros-base` 설치됨
- ✅ `ros-humble-ament-cmake` (빌드 시스템)
- ✅ `ros-humble-action-msgs`, `actionlib-msgs`
- ✅ 기본 메시지 타입들 (geometry_msgs, sensor_msgs 등)

### **2. 하드웨어 제어 패키지 (L1)**
Host의 `src/` 디렉토리 구조:
```
src/
├── ai/                  ← Brain 컨테이너에서 실행
├── common/              ✅ 공통 라이브러리
├── control/             ✅ 구현됨
│   ├── jetrover_arm_moveit/        ✅ MoveIt2 (5-DOF 로봇 암)
│   ├── ros_robot_controller_cpp/   ✅ 모터/서보 UART 제어
│   ├── ros_robot_controller_msgs/  ✅ 커스텀 메시지
│   └── rover_autonomy_cpp/         ✅ 키보드 제어 (개발용)
├── external/            ✅ TRAC-IK (역기구학)
├── hardware/            ✅ 구현됨
│   ├── battery_cpp/                ✅ 배터리 모니터
│   ├── depth_camera_cpp/           ✅ Orbbec Dabai DCW
│   ├── imu_bridge_cpp/             ✅ IMU (MPU6050/9250)
│   ├── lidar_cpp/                  ✅ RPLIDAR A1
│   ├── orbbec_camera/              ✅ 카메라 드라이버
│   └── orbbec_camera_msgs/         ✅ 카메라 메시지
└── ui/                  ✅ 로봇 암 웹 UI
```

### **3. MoveIt2 통합**
- ✅ `jetrover_arm_moveit` 패키지 존재
- ✅ TRAC-IK 역기구학 솔버 설치됨
- ✅ Launch 파일: `moveit_demo.launch.py`
- ✅ URDF, SRDF 설정 완료

### **4. 하드웨어 드라이버**
모든 센서/액추에이터 드라이버 구현됨:
- ✅ 모터 제어 (4WD 메카넘 휠)
- ✅ 서보 제어 (5-DOF 로봇 암 + 그리퍼)
- ✅ LiDAR (RPLIDAR A1)
- ✅ 카메라 (Orbbec Dabai DCW - RGB + Depth)
- ✅ IMU (MPU6050/9250)
- ✅ 배터리 모니터

---

## ❌ Host에 미구현된 부분 (Critical)

### **1. Nav2 (자율 주행 스택) — 설치 + 설정 패키지 구현 완료 (실기 주행 검증 대기)**

#### **필요한 패키지**
```bash
# APT에서 설치 가능 (확인됨)
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-amcl \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-collision-monitor \
  ros-humble-nav2-controller \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-dwb-controller \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-map-server \
  ros-humble-nav2-planner \
  ros-humble-nav2-recoveries \
  ros-humble-nav2-rviz-plugins \
  ros-humble-nav2-util \
  ros-humble-nav2-waypoint-follower
```

#### **구현 현황**
- ✅ Nav2 APT 패키지 설치 완료 (`ros-humble-nav2-*`, `ros-humble-navigation2` 확인)
- ✅ `costmap` 설정 파일 생성 완료 (global/local costmap)
- ✅ `dwb_local_planner` (holonomic/mecanum) 파라미터 구성 완료
- ✅ Navigation launch 파일 생성 완료 (`src/navigation/nav2_config/launch/nav2_bringup.launch.py`)
- ✅ 지도 파일 디렉토리 및 기본 맵 생성 완료 (`default_map.pgm`, `default_map.yaml`)
- ✅ AMCL 설정 추가 완료 (`amcl` 파라미터 블록)
- ✅ `ros2 launch nav2_config nav2_bringup.launch.py --show-args` 검증 완료

#### **신규 생성 파일**
- `src/navigation/nav2_config/package.xml`
- `src/navigation/nav2_config/CMakeLists.txt`
- `src/navigation/nav2_config/launch/nav2_bringup.launch.py`
- `src/navigation/nav2_config/params/nav2_params.yaml`
- `src/navigation/nav2_config/maps/default_map.yaml`
- `src/navigation/nav2_config/maps/default_map.pgm`

#### **영향**
- 현재는 "설정 파일 부재" 이슈는 해소됨
- 단, 실제 주행 검증(/scan, /odom, TF, 목표점 이동) 전까지는 완전한 자율주행 보장 불가

---

### **2. SLAM (지도 작성) — 설치 + 설정 패키지 구현 완료 (실기 맵핑 검증 대기)**

#### **필요한 패키지**
```bash
# APT에서 설치 가능 (확인됨)
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-cartographer \
  ros-humble-cartographer-ros
```

#### **구현 현황**
- ✅ `slam_toolbox` 설치 완료
- ✅ `cartographer`, `cartographer_ros` 설치 완료
- ✅ SLAM 설정 파일 생성 완료 (`src/navigation/slam_config/config/slam_params.yaml`)
- ✅ SLAM launch 파일 생성 완료
  - `online_async_launch.py` (매핑)
  - `localization_launch.py` (로컬라이제이션)
- ✅ `/map` 토픽 발행 노드 구동 가능한 launch 구성 완료 (`async_slam_toolbox_node`)
- ✅ 지도 저장 스크립트 생성 완료 (`src/navigation/slam_config/scripts/save_map.sh`)
- ✅ LiDAR 데이터(`/scan`) 연동 파라미터/리매핑 구성 완료
- ✅ `colcon build --packages-select slam_config` 빌드 성공
- ✅ `ros2 launch slam_config ... --show-args` 검증 성공

#### **신규 생성 파일**
- `src/navigation/slam_config/package.xml`
- `src/navigation/slam_config/CMakeLists.txt`
- `src/navigation/slam_config/config/slam_params.yaml`
- `src/navigation/slam_config/launch/online_async_launch.py`
- `src/navigation/slam_config/launch/localization_launch.py`
- `src/navigation/slam_config/scripts/save_map.sh`
- `src/navigation/slam_config/maps/.gitkeep`

#### **영향**
- 현재는 "SLAM 설정/런치 부재" 이슈는 해소됨
- 단, 실제 로봇에서 맵 생성/저장/재로컬라이제이션 실기 검증 전까지 완전한 운용 보장 불가

---

### **3. ROS2 Launch 통합 시스템 — 기본 통합 기동 구현 완료**

#### **구현 현황**
- ✅ `host_bringup` 패키지 생성 완료
- ✅ 전체 시스템 시작 launch 파일 생성 완료:
  - `src/bringup/host_bringup/launch/host_bringup_main.launch.py`
  - 포함 구성:
    - 모터 컨트롤러 (`ros_robot_controller_cpp.launch.py`)
    - 센서 드라이버 (RPLIDAR A1, IMU, 카메라, 배터리)
    - SLAM (`slam_config` mapping/localization 선택)
    - Nav2 (`nav2_config`)
    - MoveIt2 (옵션)
- ✅ 단계별 시작 순서 적용 (TimerAction): 하드웨어 → SLAM(2s) → Nav2(5s) → MoveIt2(7s)
- ✅ 시스템 모니터 노드 추가 (`host_system_monitor.py`)
  - `/host/system_status`, `/host/system_ok` 발행
- ✅ 간단한 시작 스크립트 추가 (`scripts/start_host.sh`)
- ⚠️ Nav2 lifecycle manager는 Nav2 내부에서 동작, 시스템 전체 lifecycle 통합 관리자(단일 supervisor)는 아직 없음

#### **신규 생성 파일**
- `src/bringup/host_bringup/package.xml`
- `src/bringup/host_bringup/CMakeLists.txt`
- `src/bringup/host_bringup/launch/host_bringup_main.launch.py`
- `src/bringup/host_bringup/scripts/host_system_monitor.py`
- `src/bringup/host_bringup/scripts/start_host.sh`
- `src/bringup/host_bringup/config/.gitkeep`

#### **영향**
- 수동 개별 실행 부담 감소 (단일 launch로 기동 가능)
- 노드 시작 순서가 통제되어 초기화 안정성 향상
- 단, 프로세스 크래시 자동복구/강제 재기동까지는 아직 제한적

---

### **4. E-Stop (비상 정지) 시스템**

#### **미구현 사항**
- ❌ 하드웨어 E-Stop 버튼 연동 코드 없음
- ❌ `/emergency_stop` 토픽 발행자 없음
- ❌ 모터 제어 노드에 E-Stop 핸들러 없음
- ❌ 소프트웨어 E-Stop API 없음

#### **필요 구현**
```cpp
// ros_robot_controller_cpp에 추가 필요
void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        // 모든 모터 즉시 정지
        stop_all_motors();
        // 서보 홀드
        hold_servos();
        RCLCPP_ERROR(get_logger(), "EMERGENCY STOP ACTIVATED");
    }
}
```

---

### **5. TF (Transform) 브로드캐스터**

#### **미구현 사항**
- ❌ `robot_state_publisher` 설정 불완전
- ❌ Static TF (base_link → 센서들) 일부 누락
- ❌ SLAM/Nav2용 TF tree 연결 없음
  ```
  map → odom → base_link → lidar_link
                         → camera_link
                         → imu_link
                         → arm_base_link → ...
  ```

---

### **6. 파라미터 서버 & 설정 파일**

#### **미구현 사항**
- ❌ `/config` 디렉토리 없음
- ❌ `params.yaml` (시스템 전역 파라미터)
- ❌ `robot_description.yaml` (URDF 경로 등)
- ❌ ROS2 parameter bridge (런타임 재설정) 없음

---

### **7. 시스템 서비스 & 자동 시작**

#### **미구현 사항**
- ❌ systemd 서비스 파일 없음
  ```bash
  # 예상 파일: /etc/systemd/system/jetrover-host.service
  # 부팅 시 자동 시작
  ```
- ❌ udev rules (하드웨어 권한 자동 설정) 불완전
- ❌ 로그 로테이션 설정 없음

---

### **8. 진단 & 모니터링**

#### **미구현 사항**
- ❌ `ros2_control` diagnostics aggregator 없음
- ❌ `/diagnostics` 토픽 발행자 없음
- ❌ 하드웨어 상태 모니터링 노드 없음
- ❌ CPU/GPU/메모리 사용률 추적 없음

---

## 📊 Host 구현 진척도 요약

| 카테고리 | 구현률 | 상태 |
|:---|---:|:---|
| **L1: 하드웨어 드라이버** | **100%** | 모든 센서/액추에이터 완료 |
| **L2: MoveIt2 (로봇 암)** | **90%** | 기본 동작 OK, 최적화 필요 |
| **L2: Nav2 (자율주행)** | **75%** | 설치+설정 완료, 실기 주행 검증 대기 ⚠️ |
| **L2: SLAM (지도작성)** | **75%** | 설치+설정 완료, 실기 맵핑 검증 대기 ⚠️ |
| **통합 Launch 시스템** | **80%** | host_bringup 구현 완료, 고급 supervisor/자동복구 고도화 필요 ⚠️ |
| **E-Stop** | **0%** | 미구현 ❌ |
| **TF Tree** | **40%** | 부분적 구현 |
| **시스템 서비스** | **0%** | 미구현 |
| **진단/모니터링** | **0%** | 미구현 |

---

## 🎯 Host 미구현 부분 우선순위

### **Phase 1: 필수 로보틱스 기능 (2~3주)**
1. ⭐⭐⭐ **Nav2 실기 주행 검증**
   - 실제 센서(/scan), 오도메트리(/odom), TF 확인 후 목표점 주행 테스트
   - 예상 작업: 8시간

2. ⭐⭐⭐ **SLAM 실기 맵핑 검증**
   - 실주행 맵 생성 → `save_map.sh` 저장 → localization 모드 재기동 검증
   - 예상 작업: 10시간

3. ⭐⭐ **통합 Launch 고도화**
   - 프로세스 자동복구(supervisor/systemd) 및 상태대시보드 연계
   - 예상 작업: 8시간

### **Phase 2: 안전 & 안정성 (1~2주)**
4. ⭐⭐⭐ **E-Stop 시스템**
   - 하드웨어 버튼 + 소프트웨어 API
   - 예상 작업: 16시간

5. ⭐⭐ **TF Tree 완성**
   - 모든 좌표계 연결
   - 예상 작업: 8시간

### **Phase 3: 운영 자동화 (1주)**
6. ⭐ **systemd 서비스**
   - 부팅 시 자동 시작
   - 예상 작업: 8시간

7. ⭐ **진단/모니터링**
   - diagnostics aggregator
   - 예상 작업: 12시간

---

## 💡 다음 단계 제안

### **즉시 실행 가능한 명령어**

1. **Nav2 런타임 확인**:
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ubuntu/AI_secretary_robot/install/setup.bash
   ros2 launch nav2_config nav2_bringup.launch.py --show-args
   ```

2. **SLAM 런타임 확인**:
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ubuntu/AI_secretary_robot/install/setup.bash
   ros2 launch slam_config online_async_launch.py --show-args
   ```

3. **통합 Host 런치 확인**:
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ubuntu/AI_secretary_robot/install/setup.bash
   ros2 launch host_bringup host_bringup_main.launch.py --show-args
   ```

---

## 📝 Host Codex 프롬프트 필요 여부

**Yes!** Host 미구현 부분도 별도 Codex 프롬프트 세트가 필요합니다:

1. **Prompt H1**: Nav2 설정 및 Launch 작성
2. **Prompt H2**: SLAM Toolbox 설정 및 지도 관리
3. **Prompt H3**: 통합 Bringup 시스템
4. **Prompt H4**: E-Stop 및 안전 시스템
5. **Prompt H5**: TF Tree 완성
6. **Prompt H6**: systemd 서비스 & 자동화

작성 필요 시 요청 주세요!
