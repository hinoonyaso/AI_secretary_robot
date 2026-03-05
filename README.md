# 🤖 AI Secretary Robot (JetRover)

> **완전 오프라인 음성 비서 로봇** | ROS2 기반 자율주행 & 매니퓰레이터 제어  
> 🔴 **현재 개발 진행중 (WIP)** | 오프라인 모드 최우선 구현 중

---

## 🎬 Current Working Demo

현재 동작하는 핵심 기능:

- Wake word detection (Porcupine)
- Voice Activity Detection (Silero)
- STT pipeline (Moonshine)
- ROS2 Navigation (Nav2 + SLAM Toolbox)
- Robot arm trajectory execution (MoveIt2)

검증 방법 (Quick Proof):

- `ros2 topic echo /wake_vad/transcript` 로 STT 출력 확인
- `rviz2` 에서 Nav2 goal 전송 후 경로 계획/주행 확인
- MoveIt2 trajectory 실행으로 팔 동작 확인

---

## 🏗️ System Architecture

```text
Voice (Offline)
Wake Word(Porcupine) -> VAD(Silero) -> STT(Moonshine) -> Intent(KoSimCSE) -> LLM(Qwen2.5) -> Action -> TTS(Piper)
                                                                                                  |         |
                                                                                                  v         v
Robot (ROS2)                                                                                  MoveIt2      Nav2
                                                                                                  \\        /
                                                                                                   -> Robot HW(STM32 bridge)

Vision
Camera -> YOLO11n(TRT) -> Nav2
Camera -> Moondream2
Camera -> RapidOCR

Localization
SLAM Toolbox(RPLIDAR) -> Nav2

Fallback
STT -> Groq Whisper
LLM -> Cloud LLM
Intent -> Groq Intent
TTS -> Edge TTS
```

아래는 현재 전체 파이프라인 요약입니다.

- Voice: Porcupine -> Silero -> Moonshine -> KoSimCSE -> Qwen -> Piper
- Vision: Camera -> YOLO11n / Moondream2 / RapidOCR
- Robot: Nav2 + SLAM Toolbox, MoveIt2, STM32 bridge
- Fallback: Groq Whisper, Cloud LLM, Edge TTS

## 🛠️ Tech Stack

| Category | Technologies |
|---|---|
| AI/ML | ![ONNX Runtime](https://img.shields.io/badge/ONNX-Runtime-blue) ![TensorRT](https://img.shields.io/badge/TensorRT-10.x-green) ![GGUF](https://img.shields.io/badge/GGUF-llama.cpp-orange) |
| STT | Moonshine-tiny-ko (sherpa-onnx) • Whisper (Groq Fallback) |
| TTS | Piper TTS (VITS ONNX) • Edge TTS (Cloud Fallback) |
| LLM | Qwen2.5-1.5B (local) • GPT-4o/Gemini (Hybrid) |
| Vision | YOLO11n (TRT) • Moondream2 • RapidOCR |
| NLP | KoSimCSE (Embedding) |
| ROS2 | ![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue) MoveIt2 • Nav2 • SLAM Toolbox |
| HW | Jetson Orin Nano • JetRover • Hiwonder Bus Servo • RPLIDAR |
| Runtime | Python 3.10 • C++ • CUDA 12.x |

## 📊 Implementation Status

### ✅ Implemented (Working)

Evidence Links:

- Wake/VAD/STT nodes: `src/wake_vad_detector`, `src/stt_engine` (repo path)
- Nav2/MoveIt launch-config: `src/ai_secretary/launch` (repo path)

- [x] Wake Word Detection - Porcupine C SDK (Local)
- [x] Voice Activity Detection - Silero VAD ONNX
- [x] STT Pipeline - Moonshine ONNX GPU + ONNX Runtime
- [x] Hybrid STT Fallback - Groq Whisper API 연동
- [x] Robot Hardware Interface - STM32 USB CDC 브리지 (모터/IMU/배터리/부저)
- [x] ROS2 Control - Hiwonder follow_joint_trajectory_bridge
- [x] Navigation Stack - SLAM Toolbox + Nav2 (Composition)
- [x] MoveIt2 Integration - URDF + trac-ik
- [x] Custom Messages - ROS2 msg/srv 정의 완료
- [x] Dual Mode Launcher - Offline/Hybrid 모드 런치 구분

### 🚧 In Progress (Priority Order)

- [ ] Local Intent Router - KoSimCSE ONNX (Cosine Similarity) 🔴 P0
- [ ] Vision Stack - YOLO11n TRT FP16 상시 적재 🔴 P0
- [ ] Safety Interlock - LLM 실행 시 로봇 속도 0.05m/s 이상 차단 🔴 P0
- [ ] LLM Context Limit - MAX_HISTORY=3 히스토리 관리 🟡 P1
- [ ] VLM Integration - Moondream2 GGUF 로컬 실행 🟢 P2
- [ ] OCR - RapidOCR ONNX 🟢 P2
- [ ] EKF Localization - robot_localization 퓨전 🟡 P1
- [ ] Pan/Tilt Control - 카메라 포즈 컨트롤러 🟢 P2
- [ ] Safety Monitor - Heartbeat monitor + 열 제한 정책 🟡 P1
- [ ] Memory Swap - YOLO↔VLM GPU 메모리 스왑 (08 설계) 🟢 P2

## 🎯 Key Features

### 🔒 Full Offline Mode (Primary)

모든 음성 처리가 로컬 Jetson Orin Nano에서 실행됩니다. 인터넷 연결 없이도 작동합니다.

```bash
# 오프라인 모드 실행
ros2 launch ai_secretary offline_mode.launch.py
```

### ☁️ Hybrid Fallback

로컬 모델 실패 시 클라우드 API로 자동 폴백됩니다.

| Local | Fallback |
|---|---|
| Moonshine STT | Groq Whisper |
| Qwen2.5-1.5B | GPT-4o → Groq → Gemini → Ollama |
| KoSimCSE Intent | Groq LLM 분류 |
| Piper TTS | Edge TTS |

```bash
# 하이브리드 모드 실행 (폴백 활성화)
ros2 launch ai_secretary hybrid_mode.launch.py
```

## 🚀 Quick Start

### Prerequisites

- Jetson Orin Nano (JetPack 6.x)
- ROS2 Jazzy Jalisco
- CUDA 12.x + TensorRT 10.x
- Python 3.10+

### Installation

```bash
# 저장소 클론
git clone https://github.com/hinoonyaso/AI_secretary_robot.git
cd AI_secretary_robot

# 의존성 설치
rosdep install --from-paths src --ignore-src -y

# 빌드
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 환경 설정
source install/setup.bash
```

### Execution

```bash
# 오프라인 모드 (권장)
ros2 launch ai_secretary offline_mode.launch.py

# 하이브리드 모드
ros2 launch ai_secretary hybrid_mode.launch.py

# 개별 컴포넌트 테스트
ros2 run wake_vad_detector porcupine_node
ros2 run stt_engine moonshine_node --ros-args -p model_path:=/path/to/moonshine
```

## 📋 Detailed Planning Review

<details>
<summary>🔍 Planning 01~11 문서 기준 상세 검토 보고서 (펼쳐보기)</summary>

### 오프라인 모드 검토 결과

| 컴포넌트 | Planning 기준 | 실제 구현 | 상태 |
|---|---|---|---|
| Wake Word | Porcupine C SDK | Porcupine C SDK | ✅ 일치 |
| VAD | Silero VAD (sherpa-onnx) | Silero VAD ONNX | ✅ 일치 |
| STT | moonshine ONNX GPU | ONNX Runtime GPU + moonshine | ✅ 일치 (래퍼 미사용) |
| TTS | sherpa-onnx Piper | 외부 piper CLI 호출 | ⚠️ 프로세스 구조 차이 |
| LLM | llama.cpp HTTP API | Ollama 경유 | ⚠️ 런타임 구조 차이 |
| Intent | KoSimCSE ONNX | 미구현 | ❌ 미구현 (P0) |
| Vision | YOLO11n TRT | 미구현 | ❌ 미구현 (P0) |

### 토픽 네이밍 현황

| Planning (07_Topic_Map) | 실제 구현 |
|---|---|
| /stt_result | /wake_vad/transcript |
| /tts_request | /llm/response |
| /intent | /intent_router/category |
| /bus_servo/set_position | /ros_robot_controller/bus_servo/set_position |

</details>

## 📝 License

MIT License - 개인 및 상업적 사용 가능

⚠️ Note: 이 프로젝트는 현재 활발히 개발 중입니다. 주요 기능(P0 항목)은 계속해서 업데이트될 예정입니다.
