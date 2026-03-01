# GPU/메모리 예산표

## JetRover AI 아키텍처

**최종 통합 기술 설계 문서**  
버전 3.0 | 2026년 2월

---

## 시스템 사양

| 항목 | 사양 |
|------|------|
| 프로세서 | Jetson Orin Nano 8GB |
| CPU | 6코어 Cortex-A78AE |
| GPU | 1024 CUDA 코어 |
| 메모리 | 8GB LPDDR5 (CPU/GPU 공유) |
| TDP | 15W |

---

## 상시 적재 구성요소

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

---

## 시나리오별 피크

| 시나리오 | 점유 RAM | 여유 | 판정 |
|----------|----------|------|------|
| 자율이동 + 음성 명령 (LLM 활성) | 5.0~5.1GB | ~2.9GB | ✅ 정상 |
| VLM 장멸분석 (YOLO 언로드, 정지) | 4.6~4.9GB | ~3.1GB | ✅ YOLO 언로드 전제 |
| 맵핑 모드 (SLAM sync) + LLM | 5.1~5.3GB | ~2.7GB | ⚠️ LLM MAX_HISTORY 2턴으로 축소 |
| MoveIt 경로계획 + LLM | 5.0~5.2GB | ~2.8GB | ✅ 정상 |
| 맵핑 + VLM 동시 (금지) | 5.7~6.0GB | ~2.0GB | ❌ 정책으로 차단 |

ℹ️ 맵핑 + VLM 동시 실행은 정책으로 차단. 개별 실행은 모두 안전 여유 내.

---

## AI 모델별 메모리 사용량

| 모델 | RAM | GPU 메모리 | 실행 조건 |
|------|-----|------------|-----------|
| Porcupine | 30MB | - | 상시 |
| Silero VAD | 80MB | - | 상시 |
| moonshine-tiny-ko | ~150MB | ~100MB | STT 활성 시 |
| YOLO11n TRT FP16 | 400MB | 350MB | 상시 (VLM 시 언로드) |
| KoSimCSE ONNX | 450MB | - | 상시 |
| Piper TTS | 180MB | - | TTS 활성 시 |
| RapidOCR ONNX | 300MB peak | - | 요청 시 |
| Qwen2.5-1.5B GGUF | 1.35GB | 1.2GB | LLM 활성 시 |
| moondream2 GGUF | 1.75GB | 1.5GB | VLM 활성 시 |

---

## CPU 코어 점유 분석 (6코어 Cortex-A78AE)

### 상시 동작 기준

| 코어 | 할당 | 점유율 | 설명 |
|------|------|--------|------|
| C0~C1 | ros2_control + CycloneDDS | ~1.0코어 | 100Hz 실시간 제어 |
| C2 | slam_toolbox | ~0.5코어 | localization 모드 |
| C3 | Nav2 A* + DWB | ~0.5~2코어 | 경로 재계획 시 급증 |
| C4 | Silero VAD + Piper TTS + KoSimCSE | ~0.5~1코어 | 음성 처리 |
| C5 | llama.cpp 스케줄러 | ~0.3코어 | GPU 보조, nice +10 |

### CPU 사용량 요약

| 상태 | 점유 코어 | 사용률 | 비고 |
|------|-----------|--------|------|
| 일반 상태 | 2.8~3.5코어 | ~58% | 대기/이동 중 |
| 피크 (A* + OCR + TTS 동시) | 4.5~5.5코어 | ~92% | 순간 최대 |

⚠️ ros2_control nice -20으로 최우선 보장 필요

---

## GPU 사용량 분석

| 모델/작업 | GPU 메모리 | GPU 사용률 | 비고 |
|-----------|------------|------------|------|
| YOLO11n TRT FP16 | 350MB | 30-40% | 연속 추론 |
| moonshine-tiny-ko | 100MB | 20-30% | STT 시 |
| Qwen2.5-1.5B GGUF | 1.2GB | 60-70% | LLM 시 |
| moondream2 GGUF | 1.5GB | 80-90% | VLM 시 |
| CUDA 컨텍스트 | ~200MB | - | 기본 오버헤드 |

### GPU 사용 시나리오

| 시나리오 | 총 GPU 메모리 | GPU 사용률 | 판정 |
|----------|---------------|------------|------|
| 대기 (YOLO만) | 550MB | 30-40% | ✅ |
| 이동 + YOLO | 550MB | 30-40% | ✅ |
| 정지 + STT + LLM | 1.65GB | 80-90% | ✅ |
| 정지 + VLM | 2.15GB | 90-100% | ✅ (YOLO 언로드) |
| 이동 + LLM | 1.65GB | 90-100% | ❌ TDP 초과 |

---

## 전력 예산 (Jetson 15W TDP)

### 시나리오별 Jetson 소비 전력

| 운용 시나리오 | CPU | GPU+추론 | 합계 | 판정 |
|---------------|-----|----------|------|------|
| 대기 (Wake word 감지) | ~1.5W | ~0.8W | ~2.3W | ✅ |
| 자율이동 + YOLO 연속 | ~3.0W | ~5.0W | ~8.0W | ✅ |
| 정지 + STT + LLM | ~3.5W | ~7.0W | ~10.5W | ✅ |
| 정지 + VLM (YOLO OFF) | ~3.0W | ~9.0W | ~12.0W | ✅ |
| 이동 + LLM 동시 | ~4.5W | ~12.0W | ~16.5W | ❌ TDP 초과 |
| 맵핑 + LLM 동시 | ~5.5W | ~7.0W | ~12.5W | ⚠️ 스로틀링 위험 |

### 전력 관리 설정

```bash
# 15W MAXN 모드 고정
sudo nvpmodel -m 0

# 클럭 최대화
sudo jetson_clocks

# 전력 모니터링 (13W 초과 시 경보)
tegrastats --interval 500 | grep VDD_CPU_GPU_CV
```

ℹ️ 서보 모터(HTD-35H3 ×6)는 11.1V 별도 LiPo 배터리로 구동. Jetson 15W TDP와 완전 독립.  
ℹ️ Mecanum DC 모터도 12V 별도 전원. Jetson 측 추가 부담은 USB 통신 ~0.9W 수준.

---

## 메모리 관리 정책

### 동적 메모리 할당

| 모델 | 할당 시점 | 해제 시점 | 회수 메모리 |
|------|-----------|-----------|-------------|
| YOLO11n TRT | 부팅 시 | VLM 호출 시 | 400MB |
| LLM KV 캐시 | 첫 LLM 호출 | VLM 호출 시 | 가변 |
| moondream2 | VLM 호출 시 | VLM 완료 후 | 1.75GB |

### 메모리 회수 시퀀스 (VLM 호출 시)

1. YOLO TRT 엔진 unload() → ~400MB 회수
2. LLM KV 캐시 초기화 → ~500MB 회수
3. moondream2 GGUF 로드 → 이미지 인코딩 → 추론
4. VLM 완료 후 YOLO TRT 엔진 재적재 (~200ms)

---

## 모니터링 명령어

```bash
# 메모리 사용량 확인
free -h
cat /proc/meminfo | grep MemAvailable

# GPU 메모리 확인
tegrastats | grep GR3D_FREQ
tegrastats | grep RAM

# 프로세스별 메모리 사용
ps aux --sort=-%mem | head -20

# ROS2 노드별 리소스
ros2 node info /node_name
```

---

*JetRover AI Architecture v3.0 | Jetson Orin Nano 8GB + ROS2 Humble + 6DOF Mecanum*
