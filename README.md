# AI_secretary_robot
## 아직 진행중인 프로젝트 입니다 ##

# JetRover Planning 엄격 대조 검토 보고서  
**범위:** Planning 01~11 문서 기준 모델·엔진·정책과 실제 구현 비교  
**우선순위:** ★ 완전 오프라인 모드 최우선  
**분석일:** 2026-03-01  

---

# 1️⃣ Planning 기준 모델 스택 (05_컴포넌트_설계서 기준)

| 단계 | Planning 지정 모델 | 런타임 | 로컬/클라우드 |
|------|-------------------|--------|---------------|
| Wake | Porcupine (C SDK) | CPU | 로컬 |
| VAD | Silero VAD (sherpa-onnx) | CPU ONNX | 로컬 |
| STT | moonshine-tiny-ko (sherpa-onnx GPU) | GPU ONNX | 로컬 |
| Intent | KoSimCSE ONNX | CPU FP32 | 로컬 |
| LLM | Qwen2.5-1.5B GGUF (llama.cpp -ngl 99) | GPU | 로컬 |
| TTS | Piper TTS 한국어 (VITS ONNX) | CPU FP32 | 로컬 |
| Vision | YOLO11n TRT FP16 | GPU | 로컬 |
| VLM | moondream2 GGUF (-ngl 99) | GPU | 로컬 |
| OCR | RapidOCR ONNX | CPU FP32 | 로컬 |

---

# 🔵 오프라인 모드 (최우선)

## ✅ Planning 기준과 일치

| # | 컴포넌트 | Planning 지정 | 실제 구현 | 판정 |
|---|-----------|--------------|------------|------|
| 1 | Wake Word | Porcupine C SDK | Porcupine C SDK | ✅ |
| 2 | VAD | Silero VAD (sherpa-onnx) | Silero VAD ONNX | ✅ |
| 3 | STT | moonshine ONNX GPU | ONNX Runtime GPU + moonshine | ✅ |
| 4 | 음성 상태 머신 | Wake→VAD→STT→Intent→LLM→TTS | 동일 흐름 구현 | ✅ |
| 5 | Hiwonder Bridge | follow_joint_trajectory_bridge | Python Action Server 구현 | ✅ |
| 6 | ROS Robot Controller | STM32 USB CDC 브리지 | 모터/IMU/배터리/부저 등 구현 | ✅ |
| 7 | MoveIt2 | URDF + trac-ik | 패키지 완비 | ✅ |
| 8 | 커스텀 메시지 | msg/srv 정의 | 정의 완료 | ✅ |
| 9 | HW 드라이버 | RPLIDAR, DCW, IMU 등 | 5개 드라이버 구현 | ✅ |
|10 | SLAM | slam_toolbox | launch + params | ✅ |
|11 | Nav2 | Composition | launch + params | ✅ |
|12 | 공통 유틸 | json/curl/string 등 | 구현 완료 | ✅ |

---

## ⚠️ 부분 구현 (Planning과 차이 존재)

| # | 컴포넌트 | Planning 기준 | 실제 구현 | 차이점 |
|---|-----------|--------------|------------|--------|
| 13 | STT 엔진 | sherpa-onnx 래퍼 사용 | ONNX Runtime 직접 호출 | 래퍼 미사용 |
| 14 | TTS | sherpa-onnx Piper | 외부 piper CLI 호출 | 프로세스 구조 차이 |
| 15 | LLM | llama.cpp HTTP API | Ollama 경유 | 런타임 구조 차이 |

---

## ❌ 오프라인 모드 미구현

| # | 항목 | Planning 근거 | 우선순위 |
|---|-------|---------------|-----------|
|16| KoSimCSE Intent Router | 05:L20 | 🔴 P0 |
|17| LLM 인터락 (속도>0.05m/s 차단) | 05:L23 | 🔴 P0 |
|18| LLM MAX_HISTORY=3 | 05:L23 | 🟡 P1 |
|19| YOLO11n TRT FP16 | 05:L19 | 🔴 P0 |
|20| moondream2 VLM | 05 | 🟢 P2 |
|21| RapidOCR ONNX | 05 | 🟢 P2 |
|22| YOLO↔VLM 메모리 스왑 | 08 | 이후 |
|23| robot_localization EKF | 06 | 🟡 P1 |
|24| pan_tilt_controller | 06 | 🟢 P2 |
|25| heartbeat_monitor | 09 | 🟡 P1 |
|26| 열 제한 정책 | 09 | 🟡 P1 |
|27| Emergency Stop 시퀀스 | 09 | 🟡 P1 |
|28| systemd 서비스 그룹화 | 11 | 🟢 P2 |

---

# 🟣 하이브리드 모드

## ✅ 구현 완료

| # | 항목 | 상태 |
|---|------|------|
|1| STT → Groq Whisper 폴백 | ✅ |
|2| LLM 클라우드 체인 (OpenAI→Groq→Gemini→Ollama) | ✅ |
|3| Intent → Groq LLM 분류 | ✅ |
|4| 듀얼 모드 런치 | ✅ |

---

## ⚠️ Planning 문서에 없는 클라우드 API

| 컴포넌트 | 실제 구현 |
|-----------|------------|
| STT | Groq Whisper API |
| LLM | OpenAI GPT-4o, Groq llama-3.1, Gemini 2.5 |
| Intent | Groq LLM 분류 |
| TTS | Edge TTS (MS 클라우드) |

> 위 항목은 **하이브리드 모드 전용**이며 Planning 문서에는 명시되지 않음.

---

# 📋 토픽 네이밍 불일치

| Planning (07_Topic_Map) | 실제 구현 |
|--------------------------|------------|
| /stt_result | /wake_vad/transcript |
| /tts_request | /llm/response |
| /intent | /intent_router/category |
| /bus_servo/set_position | /ros_robot_controller/bus_servo/set_position |

---

# 🎯 오프라인 모드 구현 우선순위 정리

## 🔴 P0 (즉시 구현)

- KoSimCSE Intent Router (코사인 유사도 + 임계값)
- YOLO11n TRT FP16 상시 적재
- LLM 인터락 (정지 시만 실행)

## 🟡 P1 (안정성/안전성)

- llama.cpp 직접 HTTP 서버 연동
- LLM 히스토리 3턴 제한
- robot_localization EKF
- heartbeat_monitor
- 열 제한 정책
- Emergency Stop 연동

## 🟢 P2 (확장 기능)

- moondream2 VLM
- RapidOCR
- pan_tilt_controller
- systemd 그룹 관리

---

# 📌 결론

현재 구현은 **음성 파이프라인과 로봇 제어 기반은 완성도 높음**.  
그러나 Planning 문서 기준에서 보면:

- 핵심 로컬 Intent 아키텍처 미완성 (KoSimCSE)
- 비전 스택 미구현 (YOLO/VLM/OCR)
- 안전 정책 계층 부재

따라서 **오프라인 모드 완전 충족까지는 P0 3개 항목이 절대 우선**이다.

---
