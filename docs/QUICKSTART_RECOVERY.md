# 🚀 빠른 복구 가이드 (5분 안에 시작하기)

**작성일**: 2026-03-01
**상황**: STT/TTS/LLM 검증 실패 후 즉시 복구
**목표**: 최소한의 음성 파이프라인 동작 확보

---

## ⚡ 즉시 실행 (복사 & 붙여넣기)

### 1단계: TTS 의존성 설치 (2분)

```bash
cd /home/sang/dev_ws/AI_secretary_robot
bash scripts/setup_tts_dependencies.sh
```

**설치 내역**:
- ✅ espeak-ng (한국어 fallback TTS)
- ✅ edge-tts (클라우드 TTS, 선택적)
- ✅ piper (로컬 ONNX TTS, 고음질)

**예상 시간**: 2분

---

### 2단계: LLM 의존성 설치 (10분)

```bash
cd /home/sang/dev_ws/AI_secretary_robot
bash scripts/setup_llm_dependencies.sh
```

**설치 내역**:
- ✅ Ollama (즉시 사용 가능)
- ✅ Qwen2.5:1.5b 모델 (1.2GB)
- ⚠️ llama.cpp 빌드 (30분 소요, 백그라운드 실행)

**예상 시간**:
- Ollama 설치: 5분
- llama.cpp 빌드: 30분 (백그라운드)

---

### 3단계: 검증 테스트 (1분)

```bash
cd /home/sang/dev_ws/AI_secretary_robot
source install/setup.bash

# TTS 테스트
ros2 launch tts_cpp tts.launch.py &
sleep 3
ros2 topic pub /tts/text std_msgs/String "data: '안녕하세요'" --once

# LLM 테스트
ros2 launch llm_cpp llm.launch.py &
sleep 3
ros2 topic pub /intent_router/chat_text std_msgs/String "data: '자기소개해줘'" --once
```

**기대 출력**:
```
[tts_node]: TTS engine: espeak-ng
[tts_node]: Audio saved: /tmp/tts_audio/tts_xxxxx.wav
[llm_node]: LLM provider: ollama
[llm_node]: Response: {"intent": "chat", "response": "안녕하세요!..."}
```

---

## 📊 설치 후 확인 체크리스트

### TTS 엔진 상태
```bash
# espeak-ng 확인
espeak-ng --version
# 기대: eSpeak NG text-to-speech: 1.50

# edge-tts 확인
python3 -c "import edge_tts; print('✅ edge-tts installed')"

# piper 확인
piper --version
# 기대: piper version 1.2.0
```

### LLM 엔진 상태
```bash
# Ollama 확인
ollama list
# 기대: qwen2.5:1.5b  1.2GB  ...

# llama-server 확인 (빌드 완료 후)
llama-server --version
# 기대: llama server

# llama.cpp 빌드 진행 상태 (백그라운드)
tail -f /tmp/llama_cpp_build.log  # (빌드 중일 경우)
```

---

## 🎯 Phase별 복구 전략

### Phase 1: 최소 동작 (5분) ✅
**목표**: espeak-ng + Ollama로 기본 파이프라인 동작

```bash
# 이미 완료된 작업:
✅ espeak-ng 설치
✅ Ollama 설치
✅ Qwen2.5:1.5b 모델 다운로드

# 동작 가능한 파이프라인:
음성 입력 → STT (Moonshine) → LLM (Ollama) → TTS (espeak-ng) → 음성 출력
```

**제약**:
- TTS 음질: 낮음 (espeak-ng는 로봇틱한 음성)
- LLM 의존성: Ollama 데몬 필요

---

### Phase 2: 음질 개선 (추가 0분) ✅
**목표**: Piper TTS로 자연스러운 음성

```bash
# 이미 완료된 작업:
✅ piper 바이너리 설치
✅ Piper 모델 존재 (61MB ONNX)

# TTS 엔진 변경
ros2 param set /tts_node tts_engine "piper"
ros2 topic pub /tts/text std_msgs/String "data: '파이퍼 음성입니다'" --once

# 기대: 자연스러운 한국어 음성 (MOS 4.0/5.0 수준)
```

---

### Phase 3: 로컬 LLM (추가 30분) ⏳
**목표**: llama.cpp 직접 추론 (Planning 문서 준수)

```bash
# 백그라운드 빌드 대기 (30분)
# scripts/setup_llm_dependencies.sh 실행 중...

# 빌드 완료 후:
llama-server \
  --model /home/sang/dev_ws/AI_secretary_robot/models/llm/qwen2.5-1.5b-instruct-q4_k_m.gguf \
  --port 8081 \
  -ngl 99 &

# ROS2 노드에서 llama.cpp 사용
# (params.yaml 수정 후 재시작 필요)
```

---

## 🔥 트러블슈팅 (1분 안에 해결)

### 1. TTS 음성 재생 안 됨
**증상**: `aplay` 에러 메시지

**해결**:
```bash
# ALSA 장치 확인
aplay -l

# 기본 장치 변경
export ALSA_CARD=0  # 또는 실제 카드 번호

# ROS2 파라미터 변경
ros2 param set /tts_node alsa_device "plughw:CARD=Device,DEV=0"
```

---

### 2. Ollama 연결 실패
**증상**: `Failed to connect to Ollama`

**해결**:
```bash
# Ollama 서비스 상태 확인
sudo systemctl status ollama

# 수동 시작
ollama serve &

# 연결 테스트
curl http://localhost:11434/api/tags
```

---

### 3. piper 실행 실패
**증상**: `piper: command not found`

**해결**:
```bash
# PATH 확인
which piper
# 없으면 수동 설치:
sudo ln -sf /usr/local/bin/piper /usr/bin/piper

# 또는 전체 경로 사용
ros2 param set /tts_node piper_executable "/usr/local/bin/piper"
```

---

### 4. llama.cpp 빌드 실패
**증상**: `CMake error: CUDA not found`

**해결**:
```bash
# CUDA 경로 확인
nvcc --version

# 환경변수 설정
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH

# 빌드 재시도
cd /home/sang/dev_ws/AI_secretary_robot/external/llama.cpp/build
cmake .. -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87
make -j$(nproc)
```

---

## 📞 긴급 지원 명령어

### 전체 프로세스 재시작
```bash
# 모든 ROS2 노드 종료
pkill -f ros2

# Ollama 재시작
sudo systemctl restart ollama

# ROS2 노드 재시작
cd /home/sang/dev_ws/AI_secretary_robot
source install/setup.bash
ros2 launch tts_cpp voice_pipeline_local.launch.py
```

### 로그 확인
```bash
# TTS 로그
cat ~/.ros/log/latest/tts_node/stdout.log | tail -50

# LLM 로그
cat ~/.ros/log/latest/llm_node/stdout.log | tail -50

# Ollama 로그
journalctl -u ollama -n 50

# llama-server 로그 (실행 중일 경우)
cat /tmp/llama_server_8081.log
```

### 시스템 리소스 확인
```bash
# GPU 사용량
nvidia-smi

# Jetson 통합 모니터링
tegrastats

# 메모리 사용량
free -h

# 디스크 공간
df -h
```

---

## ✅ 최종 확인 (모든 단계 완료 후)

### 전체 파이프라인 테스트
```bash
cd /home/sang/dev_ws/AI_secretary_robot
source install/setup.bash

# 음성 파이프라인 실행
ros2 launch tts_cpp voice_pipeline_local.launch.py

# 별도 터미널에서:
# 1. Wake Word 시뮬레이션
ros2 topic pub /wake_vad/detected std_msgs/Bool "data: true" --once

# 2. 음성 입력 시뮬레이션 (실제 마이크 대신)
ros2 topic pub /stt/text std_msgs/String "data: '로봇 팔 들어'" --once

# 3. 기대 출력:
# - /llm/response: {"intent": "manipulator_move", ...}
# - /tts/audio_path: /tmp/tts_audio/tts_xxxxx.wav
# - 스피커에서 응답 음성 재생
```

---

## 🎊 성공 기준

### Phase 1 완료 (최소 동작)
- ✅ espeak-ng TTS 음성 재생 성공
- ✅ Ollama LLM 추론 성공
- ✅ ROS2 토픽 `/tts/text` → `/llm/response` 연결

### Phase 2 완료 (음질 개선)
- ✅ Piper TTS 음성 재생 성공
- ✅ 자연스러운 한국어 발음 확인
- ✅ TTS 지연시간 < 200ms

### Phase 3 완료 (로컬 LLM)
- ✅ llama-server 실행 성공
- ✅ GGUF 모델 로딩 성공
- ✅ GPU 오프로딩 확인 (`-ngl 99`)
- ✅ LLM 추론 속도 15~25ms/토큰

---

## 📚 관련 문서

- **상세 가이드**: [migration_recovery_guide.md](./migration_recovery_guide.md)
- **Codex 프롬프트**:
  - [STT sherpa-onnx 마이그레이션](./codex_prompt_stt_sherpa_migration.md)
  - [TTS sherpa-onnx 마이그레이션](./codex_prompt_tts_sherpa_migration.md)
  - [LLM llama.cpp 마이그레이션](./codex_prompt_llm_llamacpp_migration.md)
- **Planning 문서**: [plan.md](./plan.md)

---

**작성자 노트**:
이 가이드는 **5분 안에 최소 동작 파이프라인을 복구**하는 것을 목표로 합니다. Phase 1만 완료해도 음성 명령 처리가 가능하며, Phase 2/3는 품질 개선을 위한 선택적 단계입니다. 문제 발생 시 트러블슈팅 섹션을 먼저 확인하세요.
