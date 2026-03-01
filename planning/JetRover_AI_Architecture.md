# JetRover AI 아키텍처

**최종 통합 기술 설계 문서**  
Jetson Orin Nano 8GB • ROS2 Humble • 6DOF Mecanum Robot  
버전 3.0 | 2026년 2월

---

## 프로젝트 개요

### 프로젝트 목표
JetRover는 Jetson Orin Nano 8GB를 두뇌로 사용하는 6DOF 로봇암 탑재 Mecanum 휠 플랫폼으로, 음성 기반 자율 이동 및 조작이 가능한 AI 로봇 시스템을 구축하는 것을 목표로 합니다.

### 하드웨어 플랫폼
| 구분 | 구성요소 | 사양 | 비고 |
|------|----------|------|------|
| 컴퓨터 | Jetson Orin Nano 8GB | 6코어 Cortex-A78AE, 1024 CUDA, 8GB LPDDR5 | CPU/GPU 메모리 공유 |
| OS | Ubuntu 22.04 + ROS2 Humble | JetPack 6.x, CUDA 12.6, cuDNN 9 | LTS 배포판 |
| 메칸움 구동 | MC520 DC 기어드 모터 × 4 | 12V, 1024PPR 자기 엔코더, 메탈 기어 | STM32F407 제어 |
| 로봇암 | HTD-35H3 버스 서보 × 6 | 35kg·cm, 12~14.8V, 반이중 UART TTL | 6DOF, 데이지체인 |
| 그리퍼 | HTS-21H 버스 서보 × 1 | 같은 TTL 버스 공유 | ID 7 |
| 팬틸트 | HTS-20H1 버스 서보 × 2 | 58g, 9~12.6V, 0.18s/60° | 치메라 마운트, ID 8-9 |
| MCU | STM32F407VET6 | Cortex-M4, 168MHz, FPU 내장 | USB 단일케이블 → Jetson |
| LiDAR | RPLIDAR A1 | 360°, 15Hz | slam_toolbox 입력 |
| 치메라 | DaBai DCW RGB-D | RGB + 깊이 | 팬틸트에 탑재 |
| 마이크 | 6마이크 어레이 | ALSA 멀티채널 | Porcupine 웨이크워드 |

### 핵심 제약 조건
- **메모리**: CPU와 GPU가 8GB LPDDR5 메모리를 공유하는 구조가 전체 자원 예산의 핵심 제약
- **전력**: Jetson 15W TDP 기준 운영
- **PyTorch-Free**: PyTorch 런타임 완전 제거로 경량화

---

## 기능 요구사항

### 음성 인터랙션
| ID | 기능 | 설명 | 우선순위 |
|----|------|------|----------|
| F-001 | 웨이크워드 감지 | "JetRover" 웨이크워드 상시 감지 | 필수 |
| F-002 | 음성 활동 감지 (VAD) | 발화 시작/종료 자동 감지 | 필수 |
| F-003 | 한국어 음성 인식 (STT) | moonshine-tiny-ko 기반 실시간 STT | 필수 |
| F-004 | 의도 분류 | KoSimCSE 기반 사용자 의도 분류 | 필수 |
| F-005 | 자연어 이해/응답 | Qwen2.5-1.5B 기반 대화 및 명령 해석 | 필수 |
| F-006 | 한국어 TTS | Piper TTS 기반 음성 출력 | 필수 |
| F-007 | 음성 응답 지연 | 발화 → 첫 음절 < 1.5초 | 필수 |

### 자율 이동
| ID | 기능 | 설명 | 우선순위 |
|----|------|------|----------|
| F-101 | SLAM | RPLIDAR A1 기반 맵핑 및 위치 추정 | 필수 |
| F-102 | 경로 계획 | A* 전역 계획 + DWB 로컬 제어 | 필수 |
| F-103 | 장애물 회피 | 실시간 장애물 감지 및 회피 | 필수 |
| F-104 | 목표 지점 이동 | 음성/토픽 기반 목표 지점 이동 | 필수 |
| F-105 | 객체 추적 | YOLO11n 기반 객체 감지 및 추적 | 필수 |

### 로봇암 조작
| ID | 기능 | 설명 | 우선순위 |
|----|------|------|----------|
| F-201 | 역기구학 계산 | trac-ik 기반 6DOF IK 계산 | 필수 |
| F-202 | 경로 계획 | MoveIt2 OMPL 기반 충돌 회피 경로 | 필수 |
| F-203 | 그리퍼 제어 | 개폐 동작 제어 | 필수 |
| F-204 | 팬틸트 제어 | 치메라 실시간 시선 제어 | 필수 |
| F-205 | 집기 동작 | 객체 인식 → 접근 → 집기 시퀀스 | 선택 |

### 비전 분석
| ID | 기능 | 설명 | 우선순위 |
|----|------|------|----------|
| F-301 | 객체 감지 | YOLO11n 기반 실시간 객체 감지 | 필수 |
| F-302 | 장면 분석 (VLM) | moondream2 기반 이미지 캡셔닝/VQA | 선택 |
| F-303 | OCR | RapidOCR 기반 텍스트 인식 | 선택 |
| F-304 | 깊이 추정 | RGB-D 기반 거리 측정 | 필수 |

---

## 비기능 요구사항

### 성능 요구사항
| ID | 요구사항 | 목표값 | 측정 방법 |
|----|----------|--------|-----------|
| NF-001 | 음성 응답 지연 | < 1.5초 | 발화 종료 → TTS 첫 음절 |
| NF-002 | STT 처리 지연 | < 300ms | 3초 발화 기준 |
| NF-003 | LLM TTFT | < 500ms | 첫 토큰 생성 시간 |
| NF-004 | VLM 응답 시간 | < 4초 | 이미지 → 텍스트 완료 |
| NF-005 | IK 계산 시간 | < 10ms | 6DOF IK 해결 |
| NF-006 | 경로 계획 시간 | < 500ms | 일반 환경 기준 |
| NF-007 | 객체 감지 FPS | > 15fps | YOLO11n 기준 |
| NF-008 | 로봇암 제어 주파수 | 50Hz | 서보 명령 주기 |

### 자원 요구사항
| ID | 요구사항 | 목표값 | 비고 |
|----|----------|--------|------|
| NF-101 | 메모리 사용량 | < 6GB | 상시 적재 기준 |
| NF-102 | CPU 사용량 | < 80% | 6코어 기준 |
| NF-103 | GPU 사용량 | < 90% | 추론 시 |
| NF-104 | 전력 소비 | < 15W | Jetson TDP 준수 |
| NF-105 | 디스크 사용량 | < 16GB | 시스템 전체 |

### 안정성 요구사항
| ID | 요구사항 | 목표값 | 비고 |
|----|----------|--------|------|
| NF-201 | 시스템 가동률 | > 99% | 24/7 상시 대기 |
| NF-202 | 음성 인식 정확도 | > 90% | 정숙한 환경 기준 |
| NF-203 | 의도 분류 정확도 | > 85% | KoSimCSE 기준 |
| NF-204 | 납비 실패율 | < 1% | 정상 동작 시 |
| NF-205 | 충돌 방지 | 100% | 장애물 감지 시 |

### 보안 요구사항
| ID | 요구사항 | 설명 |
|----|----------|------|
| NF-301 | 오프라인 우선 | 네트워크 단절 시 완전 동작 |
| NF-302 | 로컬 처리 | 모든 AI 추론 Edge에서 수행 |
| NF-303 | 선택적 클라우드 | 클라우드는 품질 향상 수단으로만 사용 |

---

## 전체 시스템 아키텍처

### 아키텍처 개요
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              JetRover AI System                              │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │   음성 처리   │  │   비전 처리   │  │   자연어    │  │   로봇 제어   │    │
│  │   Pipeline   │  │   Pipeline   │  │   처리      │  │   Pipeline   │    │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘    │
│         │                 │                 │                 │            │
│  ┌──────▼─────────────────▼─────────────────▼─────────────────▼───────┐    │
│  │                      Intent Router (KoSimCSE)                      │    │
│  └──────┬────────────────────────────────────────────────────────────┘    │
│         │                                                                  │
│  ┌──────▼────────────────────────────────────────────────────────────┐    │
│  │                      ROS2 Humble (CycloneDDS)                      │    │
│  └──────┬────────────────────────────────────────────────────────────┘    │
│         │                                                                  │
│  ┌──────▼──────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────┐   │
│  │   SLAM/Nav2    │  │   MoveIt2    │  │ ros2_control │  │ 센서 드라이버│   │
│  └──────┬──────────┘  └──────┬───────┘  └──────┬───────┘  └────┬─────┘   │
│         │                    │                 │               │          │
│  ┌──────▼────────────────────▼─────────────────▼───────────────▼──────┐  │
│  │                         Hardware Interface                          │  │
│  │  (STM32F407 → Mecanum/서보/LiDAR/칩메라/마이크)                      │  │
│  └─────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 통신 토폴로지
```
Jetson Orin Nano ─── USB ─── STM32F407 ┬── PWM+ENC ── MC520×4 (Mecanum)
                                        └── TTL 반이중 ── HTD-35H3×6 + HTS-21H×1 + HTS-20H1×2
```

| 구간 | 프로토콜 | 속도 | 비고 |
|------|----------|------|------|
| Jetson ↔ STM32 | USB CDC | Full Speed 12Mbps | 단일 케이블 (모터+서보 통합) |
| STM32 ↔ DC모터 | PWM + 엔코더 ABZ | 100Hz 제어 | 4채널 독립 PID |
| STM32 ↔ 버스서보 | 반이중 UART TTL | 115200~500000bps | 9개 데이지체인, ~50Hz 안정 |
| STM32 → Jetson | odom + IMU 토픽 | 50Hz | robot_localization EKF 입력 |

---

## 컴포넌트 설계서

### AI 모델 스택

| 모델 | 실행환경 | RAM | 지연 | 전력 | 핵심 결정 |
|------|----------|-----|------|------|-----------|
| Porcupine | CPU (C SDK) | 30MB | ~10ms | ~0.1W | 웨이크워드. 상시 실행. PyTorch 불필요 |
| Silero VAD | CPU (sherpa-onnx) | 80MB | ~50ms | ~0.2W | GPU 전환 시 50~60% 오히려 느려짐 확인. CPU 확정 |
| moonshine-tiny-ko | GPU (sherpa-onnx) | ~150MB | ~250ms | ~1W burst | BF16 convolution frontend → CPU ORT에서 FP32 강등 → 정확도 저하 |
| YOLO11n TRT | GPU FP16 | 400MB | ~50ms | ~4W | FP16 확정. INT8 금지 (Orin Nano에서 이득 없고 정확도 저하) |
| KoSimCSE ONNX | CPU FP32 | 450MB | ~80ms | ~0.5W | 인텐트 분류. FP32 고정 (FP16 변환 시 CPU ORT 동작 불가) |
| Piper TTS 한국어 | CPU FP32 | 180MB | ~120ms | ~0.5W | RTF 0.1~0.3으로 GPU 불필요. VITS ONNX |
| RapidOCR ONNX | CPU FP32 | 300MB peak | ~400ms | ~1W burst | GPU 지원 공식 포기 (메인테이너 확인) |
| Qwen2.5-1.5B GGUF | GPU (-ngl 99) | 1.35GB | TTFT ~450ms | ~5W | 정지 시만 실행 (이동 중 15W 초과). MAX_HISTORY=3턴 |
| moondream2 GGUF | GPU (-ngl 99) | 1.75GB | TTFT 2.5~3.5s | ~7W | mmproj f16 고정 필수 (q4 양자화 시 VQA 정확도 저하) |

### 음성 처리 파이프라인
```
Porcupine (C SDK) → sherpa-onnx VAD → sherpa-onnx STT (moonshine GPU) 
→ KoSimCSE → Intent Router → llama.cpp LLM → sherpa-onnx TTS (Piper)
```

### ROS2 패키지 구성

| 패키지 | 역할 | RAM | CPU | 설정 요점 |
|--------|------|-----|-----|-----------|
| slam_toolbox | 맵핑/로컬라이제이션 | 200~300MB | 0.5~2코어 | 운용 시 localization 모드 |
| Nav2 (Composition) | A* 전역계획 + DWB 제어 | 150~180MB | 0.5~2코어 | use_composition:=True 필수 |
| MoveIt2 + trac-ik | 6DOF IK + 경로계획 | ~450MB | 1~2코어 | solve_type: Speed, timeout: 0.05s |
| ros2_control | 관절 상태 관리 | 150MB | 0.5코어 | update_rate: 50Hz |
| hiwonder_bridge | MoveIt ↔ Hiwonder 변환 | 40MB | 0.1코어 | JointTrajectory → ServosPosition 변환 |
| robot_state_publisher | URDF → TF | 50MB | 0.2코어 | joint_state_broadcaster 연동 |
| robot_localization | EKF (IMU + odom 융합) | 60MB | 0.2코어 | 50Hz |
| pan_tilt_controller | 칩메라 팬틸트 독립 제어 | 30MB | 0.1코어 | MoveIt 제외, 토픽 기반 |
| RPLIDAR A1 드라이버 | LaserScan 발행 | 70MB | 0.3코어 | 15Hz, /scan 토픽 |
| DaBai DCW 드라이버 | RGB-D 스트림 | 100MB | 0.3코어 | YOLO 입력 + 깊이 추정용 |

### MoveIt2 구성

**플래닝 그룹**
- arm 그룹: joint1~joint6 (HTD-35H3 ×6), end-effector: gripper_link
- gripper 그룹: gripper_joint (HTS-21H ×1), 독립 액션 서버
- pan_tilt: MoveIt2 제외 → 독립 토픽 제어

**trac-ik 설정**
```yaml
solve_type: Speed
kinematics_solver_timeout: 0.05  # 50ms
kinematics_solver_attempts: 3
```

**hiwonder_bridge 변환 로직**
```python
# 서보 포지션 범위: 0-1000 = 0°~240°, 중앙 500 = 120°
servo_pos = int(500 + math.degrees(radian) * (1000 / 240))
```

---

## 인터페이스 명세서

### 하드웨어 인터페이스

| 구간 | 인터페이스 타입 | 프로토콜 | 데이터 레이트 |
|------|-----------------|----------|---------------|
| Jetson ↔ STM32 | USB CDC ACM | 커스텀 바이너리 | 12 Mbps |
| STM32 ↔ DC 모터 | PWM + ABZ 엔코더 | 하드웨어 타이머 | 100Hz |
| STM32 ↔ 버스 서보 | UART TTL 반이중 | Hiwonder 프로토콜 | 115200~500000bps |
| Jetson ↔ LiDAR | USB | RPLIDAR 프로토콜 | 15Hz |
| Jetson ↔ 칩메라 | USB 3.0 | UVC + 커스텀 깊이 | 30fps |
| Jetson ↔ 마이크 | USB Audio | ALSA 멀티채널 | 16kHz |

### 소프트웨어 API 인터페이스

**LLM 서버 (llama.cpp)**
| 엔드포인트 | 메서드 | 설명 |
|------------|--------|------|
| `/completion` | POST | 텍스트 생성 |
| `/tokenize` | POST | 토큰화 |
| `/detokenize` | POST | 디토큰화 |

**TTS 서버 (sherpa-onnx)**
| 엔드포인트 | 메서드 | 설명 |
|------------|--------|------|
| `/tts` | POST | 텍스트 → 음성 |

**STT 서버 (sherpa-onnx)**
| 엔드포인트 | 메서드 | 설명 |
|------------|--------|------|
| `/recognize` | POST | 음성 → 텍스트 (스트리밍) |

---

## ROS2 Topic Map

### 센서 토픽
| 토픽명 | 타입 | 발행자 | 주파수 | 설명 |
|--------|------|--------|--------|------|
| `/scan` | sensor_msgs/LaserScan | rplidar_node | 15Hz | LiDAR 스캔 데이터 |
| `/camera/color/image_raw` | sensor_msgs/Image | dabai_dcw_driver | 30Hz | RGB 이미지 |
| `/camera/depth/image_raw` | sensor_msgs/Image | dabai_dcw_driver | 30Hz | 깊이 이미지 |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | dabai_dcw_driver | 30Hz | 칩메라 캘리브레이션 |
| `/imu` | sensor_msgs/Imu | stm32_bridge | 50Hz | IMU 데이터 |
| `/odom` | nav_msgs/Odometry | stm32_bridge | 50Hz | 휠 오도메트리 |

### 상태 토픽
| 토픽명 | 타입 | 발행자 | 주파수 | 설명 |
|--------|------|--------|--------|------|
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster | 50Hz | 전체 관절 상태 |
| `/tf` | tf2_msgs/TFMessage | robot_state_publisher | 50Hz | 좌표 변환 |
| `/tf_static` | tf2_msgs/TFMessage | robot_state_publisher | 정적 | 고정 좌표 변환 |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | slam_toolbox | 20Hz | 추정 위치 |

### 제어 토픽
| 토픽명 | 타입 | 발행자/구독자 | 주파수 | 설명 |
|--------|------|---------------|--------|------|
| `/cmd_vel` | geometry_msgs/Twist | Nav2 → ros2_control | 50Hz | 속도 명령 |
| `/arm_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | MoveIt → controller | 50Hz | 팔 궤적 명령 |
| `/gripper_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | MoveIt → controller | 50Hz | 그리퍼 명령 |
| `/pan_tilt_cmd` | std_msgs/Float64MultiArray | pan_tilt_controller | 50Hz | 팬틸트 각도 명령 |
| `/bus_servo/set_position` | ros_robot_controller_msgs/ServosPosition | hiwonder_bridge → STM32 | 50Hz | 서보 위치 명령 |

### AI 토픽
| 토픽명 | 타입 | 발행자 | 주파수 | 설명 |
|--------|------|--------|--------|------|
| `/yolo/detections` | vision_msgs/Detection2DArray | yolo11n_trt_node | 15Hz | 객체 감지 결과 |
| `/intent` | std_msgs/String | intent_router | 이벤트 | 분류된 의도 |
| `/wake_word` | std_msgs/Bool | porcupine_node | 이벤트 | 웨이크워드 감지 |
| `/stt_result` | std_msgs/String | sherpa_onnx_node | 이벤트 | STT 결과 |
| `/tts_request` | std_msgs/String | llama_cpp_server | 이벤트 | TTS 요청 |

### 네비게이션 토픽
| 토픽명 | 타입 | 발행자/구독자 | 설명 |
|--------|------|---------------|------|
| `/goal_pose` | geometry_msgs/PoseStamped | 사용자 → Nav2 | 목표 위치 |
| `/plan` | nav_msgs/Path | Nav2 | 계획된 경로 |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | 로컬 코스트맵 |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | 글로벌 코스트맵 |

---

## GPU/메모리 예산표

### 상시 적재 구성요소

| 구성요소 (상시 적재) | 최소 | 최대 | 비고 |
|----------------------|------|------|------|
| Ubuntu 22.04 + 커널 | 500MB | 700MB | |
| CycloneDDS + ROS2 미들웨어 | 200MB | 300MB | /dev/shm 포함 |
| AI: Porcupine + sherpa-onnx 세션 | 300MB | 380MB | VAD+STT 상시 |
| AI: YOLO11n TRT FP16 | 380MB | 450MB | VLM 시 언로드 |
| AI: KoSimCSE FP32 + Piper TTS | 580MB | 650MB | 상시 적재 |
| ROS2: slam_toolbox (localization) | 200MB | 300MB | 맵핑 시 300MB |
| ROS2: Nav2 Composition | 150MB | 200MB | |
| ROS2: MoveIt2 + trac-ik | 420MB | 500MB | move_group 독립 프로세스 |
| ROS2: ros2_control + 브리지 | 150MB | 200MB | hiwonder_bridge 포함 |
| ROS2: 센서 드라이버 (RPLIDAR + DCW) | 150MB | 200MB | robot_state_publisher 포함 |
| **상시 적재 합계** | **3.03GB** | **3.68GB** | **8GB 기준 여유 4.32GB** |

### 시나리오별 피크

| 시나리오 | 점유 RAM | 여유 | 판정 |
|----------|----------|------|------|
| 자율이동 + 음성 명령 (LLM 활성) | 5.0~5.1GB | ~2.9GB | ✅ 정상 |
| VLM 장멸분석 (YOLO 언로드, 정지) | 4.6~4.9GB | ~3.1GB | ✅ YOLO 언로드 전제 |
| 맵핑 모드 (SLAM sync) + LLM | 5.1~5.3GB | ~2.7GB | ⚠️ LLM MAX_HISTORY 2턴으로 축소 |
| MoveIt 경로계획 + LLM | 5.0~5.2GB | ~2.8GB | ✅ 정상 |
| 맵핑 + VLM 동시 (금지) | 5.7~6.0GB | ~2.0GB | ❌ 정책으로 차단 |

### CPU 코어 점유 분석 (6코어 Cortex-A78AE)

| 코어 | 할당 | 점유율 | 설명 |
|------|------|--------|------|
| C0~C1 | ros2_control + CycloneDDS | ~1.0코어 | 100Hz 실시간 제어 |
| C2 | slam_toolbox | ~0.5코어 | localization 모드 |
| C3 | Nav2 A* + DWB | ~0.5~2코어 | 경로 재계획 시 급증 |
| C4 | Silero VAD + Piper TTS + KoSimCSE | ~0.5~1코어 | 음성 처리 |
| C5 | llama.cpp 스케줄러 | ~0.3코어 | GPU 보조, nice +10 |

### 전력 예산 (Jetson 15W TDP)

| 운용 시나리오 | CPU | GPU+추론 | 합계 | 판정 |
|---------------|-----|----------|------|------|
| 대기 (Wake word 감지) | ~1.5W | ~0.8W | ~2.3W | ✅ |
| 자율이동 + YOLO 연속 | ~3.0W | ~5.0W | ~8.0W | ✅ |
| 정지 + STT + LLM | ~3.5W | ~7.0W | ~10.5W | ✅ |
| 정지 + VLM (YOLO OFF) | ~3.0W | ~9.0W | ~12.0W | ✅ |
| 이동 + LLM 동시 | ~4.5W | ~12.0W | ~16.5W | ❌ TDP 초과 |
| 맵핑 + LLM 동시 | ~5.5W | ~7.0W | ~12.5W | ⚠️ 스로틀링 위험 |

---

## Failure Mode 정의 (FMEA)

| ID | 구성요소 | 고장 모드 | 원인 | 영향 | 현재 통제 | RPN | 조치 |
|----|----------|-----------|------|------|-----------|-----|------|
| F-001 | Porcupine | 웨이크워드 미감지 | 마이크 노이즈, 잡음 환경 | 음성 명령 불가 | 6마이크 어레이, SNR 필터 | 6 | 잡음 환경 알림, 버튼 백업 |
| F-002 | STT | 인식 실패/오인식 | 발음 불분명, 방언 | 잘못된 명령 실행 | VAD 품질 게이트, 재시도 | 8 | "다시 말씀해 주세요" 요청 |
| F-003 | LLM | 응답 지연/타임아웃 | 복잡한 쿼리, 과부하 | 사용자 대기 | MAX_HISTORY 제한, 타임아웃 | 6 | 타임아웃 시 TTS 안내 |
| F-004 | YOLO | 객체 미감지 | 조명, occlusion | 충돌 위험 | 깊이 센서 융합, Nav2 코스트맵 | 9 | LiDAR 우선 회피 |
| F-005 | Nav2 | 경로 계획 실패 | 좁은 공간, 동적 장애물 | 이동 불가 | DWB 로컬 플래너, 복구 행동 | 7 | 사용자 알림, 수동 조작 |
| F-006 | MoveIt | IK 실패 | 특이 자세, 범위 초과 | 조작 불가 | trac-ik 다중 시도, 제한 설정 | 5 | 대체 자세 제안 |
| F-007 | STM32 | 통신 단절 | USB 케이블 분리 | 전체 제어 불가 | 하트비트 모니터링, 자동 재접속 | 10 | watchdog 재시작 |
| F-008 | LiDAR | 스캔 데이터 유실 | 모터 고장, 통신 오류 | SLAM 실패 | 데이터 타임스탬프 검증 | 8 | 오도메트리만으로 주행 |
| F-009 | 서보 | 위치 편차 | 부하 과다, 전압 저하 | 정밀도 저하 | 전류 모니터링, 위치 피드백 | 6 | 부하 경고, 속도 제한 |
| F-010 | Jetson | 과열 | 환경 온도, 연속 고부하 | 스로틀링, 성능 저하 | 온도 모니터링, 팬 제어 | 7 | 자동 성능 저하 모드 |
| F-011 | 메모리 | 부족 | 메모리 누수, 과다 적재 | OOM, 프로세스 종료 | 메모리 모니터링, 리소스 관리 | 9 | 비필수 프로세스 종료 |
| F-012 | 전원 | TDP 초과 | 이동+LLM 동시 실행 | 시스템 불안정 | 정책 기반 실행 제한 | 10 | 자동 LLM 중단 |

### 위험 등급 정의
- **RPN 1-3**: 낮음 - 모니터링
- **RPN 4-6**: 중간 - 주기적 점검
- **RPN 7-10**: 높음 - 즉시 조치 필요

---

## Intent→Action 매핑 테이블

| 의도 (Intent) | 키워드/패턴 | 액션 | 실행 조건 | 파라미터 |
|---------------|-------------|------|-----------|----------|
| WAKE_WORD | "JetRover" | 마이크 STT 모드 전환 | 상시 | - |
| MOVE_TO | "가줘", "이동해", "가서" | Nav2 goal 전송 | 정지 상태 | target_location |
| FOLLOW | "따라와", "따라가", "팔로우" | 객체 추적 모드 | YOLO 감지 중 | target_object |
| STOP | "정지", "멈춰", "그만" | Emergency Stop | 항상 | - |
| PICK | "집어", "줘", "가져와" | 집기 시퀀스 | 객체 감지 + 접근 완료 | target_object |
| PLACE | "놓아", "날려", "두고 와" | 놓기 시퀀스 | 그리퍼 보유 중 | target_location |
| SCAN_SCENE | "뭐 있어", "보여줘", "설명해" | VLM 호출 | 정지 상태 | - |
| READ_TEXT | "읽어줘", "뭐라고 써 있어" | OCR 호출 | 정지 상태 | - |
| GREETING | "안녕", "반가워" | 인사 응답 | 항상 | - |
| QUESTION | "뭐야", "어디", "언제" | LLM 일반 응답 | 항상 | query |
| ARM_MOVE | "팔을", "손을" | MoveIt 궤적 실행 | 정지 상태 | joint_positions |
| GRIPPER_OPEN | "펼쳐", "열어" | 그리퍼 개방 | 항상 | - |
| GRIPPER_CLOSE | "잡아", "닫아" | 그리퍼 폐쇄 | 항상 | - |
| PAN_TILT | "여기 봐", "저기 봐" | 팬틸트 제어 | 항상 | pan_angle, tilt_angle |
| MAPPING_START | "맵핑", "지도 만들어" | SLAM 모드 전환 | 정지 상태 | - |
| MAPPING_STOP | "맵핑 끝", "저장해" | 맵 저장 | 맵핑 중 | map_name |
| LOCALIZE | "여기가 어디야" | 현재 위치 응답 | localization 중 | - |

### Intent 분류 신뢰도 임계값
| 의도 유형 | 신뢰도 임계값 | 낮은 신뢰도 시 동작 |
|-----------|---------------|---------------------|
| 안전 관련 (STOP, Emergency) | 0.5 | 즉시 실행 |
| 이동/조작 | 0.75 | 확인 질문 |
| 일반 대화 | 0.6 | LLM 일반 응답 |
| VLM/OCR | 0.7 | 확인 질문 |

---

## 배포/운영 문서

### 시스템 요구사항

**하드웨어**
- Jetson Orin Nano 8GB
- 64GB 이상 microSD 또는 NVMe SSD
- 5V 4A 이상 전원 공급

**소프트웨어**
- JetPack 6.x
- Ubuntu 22.04
- ROS2 Humble Hawksbill
- CUDA 12.6, cuDNN 9

### 설치 절차

```bash
# 1. JetPack 설치
sudo apt update && sudo apt upgrade -y

# 2. ROS2 Humble 설치
sudo apt install ros-humble-desktop

# 3. 필수 패키지 설치
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-nav2-bringup \
                 ros-humble-moveit \
                 ros-humble-trac-ik

# 4. Python 패키지 설치
pip install sherpa-onnx onnxruntime-gpu rapidocr-onnxruntime

# 5. llama.cpp 빌드 (GPU 지원)
git clone https://github.com/ggerganov/llama.cpp
cd llama.cpp
mkdir build && cd build
cmake .. -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87
make -j$(nproc)

# 6. onnxruntime 중복 방지
pip uninstall onnxruntime
pip install onnxruntime-gpu

# 7. 전력 설정
sudo nvpmodel -m 0  # 15W MAXN 모드
sudo jetson_clocks
```

### systemd 서비스 구성

**그룹 1: 하드웨어 인터페이스 (최우선 시작)**
- ros_robot_controller: Mecanum 모터 + IMU + 버스서보
- rplidar_ros: RPLIDAR A1 LaserScan 15Hz
- dabai_dcw_driver: RGB-D 스트림

**그룹 2: 상태 추정**
- robot_state_publisher: URDF → TF 발행
- joint_state_broadcaster: /joint_states 집계
- ekf_filter_node: IMU + odom EKF 융합 50Hz

**그룹 3: SLAM & 네비게이션**
- slam_toolbox: localization 모드
- nav2: use_composition:=True, A* + DWB

**그룹 4: 팔 제어**
- controller_manager: ros2_control, 50Hz
- arm_trajectory_controller: 6DOF FollowJointTrajectory
- hiwonder_bridge: MoveIt ↔ Hiwonder 변환
- pan_tilt_controller: HTS-20H1 독립 토픽 제어

**그룹 5: AI 스택 (마지막 시작)**
- porcupine_node: 웨이크워드 상시 대기
- sherpa_onnx_node: VAD + STT + TTS 통합
- yolo11n_trt_node: 객체 탐지 연속 추론
- intent_router: KoSimCSE 인텐트 분류
- llama_cpp_server: on-demand LLM 서버
- heartbeat_monitor: 노드 상태 감시

### 운용 정책

**인터락 및 안전 정책**
| 정책 | 트리거 | 조치 |
|------|--------|------|
| LLM 실행 전 정지 | 음성 명령 수신 (속도 > 0.05m/s) | Nav2 velocity smoother 0 → 완전 정지 확인 → LLM 시작 |
| VLM 실행 조건 | "장면 분석" 인텐트 감지 | YOLO 언로드 → LLM KV 캐시 해제 → VLM 적재 → 완료 후 YOLO 재적재 |
| 맵핑 중 AI 제한 | online_sync 모드 활성 | VLM 비활성, LLM MAX_HISTORY 2턴, TTS 우선 처리 |
| A* 재계획 중 LLM 금지 | planner_server 액션 실행 중 | LLM 요청 큐잉 (최대 2.0s), DWB는 기존 경로로 계속 주행 |
| 열 제한 (5s 지속) | 온도 ≥ 70°C 5초 연속 or RAM < 2.5GB | Level1: VLM 비활성, Level2: LLM 비활성, Level3: YOLO만 유지 |
| 긴급 팔 정지 | "정지" 인텐트 or 충돌 감지 | STM32 Emergency Stop 패킷 즉시 전송 → MoveIt 목표 취소 |

**음성 명령 수신 시퀀스**
1. Porcupine 웨이크워드 감지 → 마이크 핸들 STT로 전환
2. Nav2 velocity smoother 속도 0 명령 발행
3. odom 속도 < 0.05m/s 확인 (완전 정지)
4. llama.cpp 추론 시작 (nice +10, GPU -ngl 99)
5. 응답 스트리밍 → Piper TTS 첫 문장부터 재생
6. 명령 실행 (Nav2 goal or MoveIt plan)
7. Porcupine 재활성

**VLM 호출 시퀀스**
1. "장면 분석" 인텐트 감지
2. TTS "분석 중입니다..." 브리징 출력
3. YOLO TRT 엔진 unload() → ~400MB 회수
4. LLM KV 캐시 초기화
5. moondream2 GGUF 로드 → 이미지 인코딩 → 추론
6. 결과 TTS 출력
7. YOLO TRT 엔진 재적재 (~200ms)

### 모니터링 및 로깅

```bash
# 전력 모니터링 (13W 초과 시 경보)
tegrastats --interval 500 | grep VDD_CPU_GPU_CV

# 메모리 모니터링
free -h && cat /proc/meminfo | grep MemAvailable

# 온도 모니터링
cat /sys/class/thermal/thermal_zone*/temp

# ROS2 토픽 모니터링
ros2 topic hz /scan /joint_states /odom
ros2 topic echo /intent

# 노드 상태 확인
ros2 node list
ros2 service list
```

### 백업 및 복구

**설정 파일 백업**
```bash
# ROS2 설정
~/.ros/
/opt/ros/humble/share/

# 커스텀 패키지
~/ros2_ws/src/

# AI 모델
~/models/

# 시스템 설정
/etc/systemd/system/ros-*
```

**복구 절차**
1. 시스템 부팅 실패 시: 복구 모드로 진입
2. ROS2 노드 충돌 시: heartbeat_monitor 자동 재시작
3. AI 모델 오류 시: 캐시 삭제 후 재적재
4. 심각한 오류 시: SD 카드 이미지 복원

---

## 핵심 결정 사항 요약

| 결정 항목 | 선택 | 근거 |
|-----------|------|------|
| Brain 런타임 | systemd native | GPU/SHM 직접 접근. Level3 강등 시 재시작 1~3s 빠름 |
| 클라우드 연동 | Offline-First (선택적 가속) | 네트워크 단절 시 완전 동작 보장 |
| STT 정밀도 | sherpa-onnx GPU (moonshine) | BF16 frontend CPU에서 FP32 강등 → 정확도 저하 확인 |
| YOLO 정밀도 | FP16 TRT (INT8 금지) | Orin Nano에서 INT8 실익 없음 |
| PyTorch 제거 | 완전 제거 가능 | sherpa-onnx + llama.cpp + ORT 조합으로 torch import 0 |
| Nav2 메모리 | dynamic composition | 다중 프로세스 대비 메모리 70% 절감 |
| 팬틸트 | MoveIt 제외, 독립 토픽 | 실시간 칩메라 시선 제어에 action 인터페이스 부적합 |
| LLM 실행 조건 | 로봇 정지 후만 허용 | 이동 중 LLM → Jetson 16.5W → 15W TDP 초과 |
| 서보 제어 Hz | 50Hz (ros2_control) | TTL 버스 9개 데이지체인 레이턴시 현실적 한계 |
| MoveIt ↔ Hiwonder | hiwonder_bridge (신규 개발) | Hiwonder 토픽 기반 API ↔ 표준 ros2_control 간 변환 필요 |

---

*JetRover AI Architecture v3.0 | Jetson Orin Nano 8GB + ROS2 Humble + 6DOF Mecanum*
