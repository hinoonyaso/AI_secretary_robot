# Migration Recovery Guide - 마이그레이션 검증 실패 해결 가이드

**작성일**: 2026-03-01
**상황**: STT/TTS/LLM 마이그레이션 후 검증 실패
**담당**: 10년차 로봇 SW 시니어 개발팀장

---

## 📊 검증 결과 요약

### ✅ STT (Moonshine) - **통과**
- sherpa-onnx 통합 성공
- 더미 입력 테스트 이슈는 해결됨
- 실제 WAV 입력 테스트 **통과**

### ⚠️ TTS (Piper) - **미통과**
**문제점**:
1. ❌ `espeak-ng` 바이너리 없음
2. ❌ `edge-tts` Python 모듈 없음
3. ⚠️ `piper` 바이너리 없음 (CLI 호출 방식 사용 중)

### ⚠️ LLM (llama.cpp) - **미통과**
**문제점**:
1. ❌ `llama-server` 바이너리 없음
2. ❌ Qwen2.5-1.5B GGUF 모델 파일 없음
3. ⚠️ Ollama 미설치 (fallback도 미동작)

---

## 🔧 해결 방안 (우선순위 순)

---

## PART 1: TTS 복구 (즉시 실행 가능)

### 1.1 espeak-ng 설치 (필수 fallback 엔진)

```bash
# espeak-ng 설치
sudo apt update
sudo apt install -y espeak-ng espeak-ng-data

# 설치 확인
espeak-ng --version
# 기대 출력: eSpeak NG text-to-speech: 1.50 ...

# 한국어 음성 테스트
espeak-ng -v ko "안녕하세요" --stdout | aplay

# 한국어 voice 파일 확인
ls /usr/lib/aarch64-linux-gnu/espeak-ng-data/voices/ko
```

**예상 소요 시간**: 5분

---

### 1.2 edge-tts 설치 (선택적 클라우드 TTS)

```bash
# edge-tts Python 모듈 설치
pip3 install edge-tts

# 설치 확인
python3 -c "import edge_tts; print('edge-tts installed')"

# 테스트 (인터넷 필요)
edge-tts --voice ko-KR-SunHiNeural --text "안녕하세요" --write-media /tmp/test_edge.mp3
aplay /tmp/test_edge.mp3 2>/dev/null || mpg123 /tmp/test_edge.mp3
```

**예상 소요 시간**: 3분

---

### 1.3 Piper 바이너리 설치 (로컬 TTS)

#### Option A: 사전 빌드 바이너리 사용 (권장)

```bash
# Piper 바이너리 다운로드 (ARM64 / aarch64)
cd /tmp
wget https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_arm64.tar.gz

# 압축 해제
tar -xvzf piper_arm64.tar.gz

# 바이너리 설치
sudo cp piper/piper /usr/local/bin/
sudo chmod +x /usr/local/bin/piper

# 확인
piper --version
# 기대 출력: piper version 1.2.0

# 테스트 (기존 모델 사용)
echo "안녕하세요" | piper \
  --model /home/sang/dev_ws/AI_secretary_robot/models/tts/neurlang_piper_onnx_kss_korean/piper-kss-korean.onnx \
  --config /home/sang/dev_ws/AI_secretary_robot/models/tts/neurlang_piper_onnx_kss_korean/piper-kss-korean.onnx.json \
  --output_file /tmp/test_piper.wav

aplay /tmp/test_piper.wav
```

**예상 소요 시간**: 10분

#### Option B: 소스 빌드 (고급)

```bash
# 의존성 설치
sudo apt install -y cmake g++ libfmt-dev libspdlog-dev libpopt-dev

# Piper 소스 클론
cd /home/sang/dev_ws/AI_secretary_robot/external
git clone https://github.com/rhasspy/piper.git
cd piper

# onnxruntime 다운로드 (ARM64)
mkdir -p lib
cd lib
wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-aarch64-1.17.1.tgz
tar -xvzf onnxruntime-linux-aarch64-1.17.1.tgz
cd ..

# 빌드
mkdir build && cd build
cmake .. -DONNXRUNTIME_DIR=../lib/onnxruntime-linux-aarch64-1.17.1
make -j$(nproc)

# 설치
sudo cp piper /usr/local/bin/
```

**예상 소요 시간**: 30분

---

### 1.4 TTS 검증

```bash
# ROS2 노드 재실행 (espeak-ng fallback 테스트)
cd /home/sang/dev_ws/AI_secretary_robot
source install/setup.bash
ros2 launch tts_cpp tts.launch.py

# 별도 터미널에서 테스트
ros2 topic pub /tts/text std_msgs/String "data: '안녕하세요'" --once

# 기대 출력:
# - /tts/engine: "espeak-ng"
# - /tts/audio_path: "/home/ubuntu/rover_ws/src/tts_cpp/output/tts_xxxxx.wav"
# - 스피커에서 음성 재생됨

# Piper 엔진 테스트 (Piper 설치 후)
ros2 param set /tts_node tts_engine "piper"
ros2 topic pub /tts/text std_msgs/String "data: '로버입니다'" --once

# 기대 출력:
# - /tts/engine: "piper"
# - 더 자연스러운 음성 재생
```

---

## PART 2: LLM 복구 (중기 작업)

### 2.1 Ollama 설치 (즉시 fallback 복구)

```bash
# Ollama 설치 (공식 스크립트)
curl -fsSL https://ollama.com/install.sh | sh

# 설치 확인
ollama --version
# 기대 출력: ollama version is 0.x.x

# Ollama 서비스 시작
sudo systemctl start ollama
sudo systemctl enable ollama

# 모델 다운로드 (Qwen2.5-1.5B)
ollama pull qwen2.5:1.5b

# 모델 확인
ollama list
# 기대 출력:
# NAME              ID          SIZE     MODIFIED
# qwen2.5:1.5b      abc123...   1.2GB    1 minute ago

# 테스트
ollama run qwen2.5:1.5b "안녕하세요"
# 기대 출력: "안녕하세요! 무엇을 도와드릴까요?" (한국어 응답)
```

**예상 소요 시간**: 15분 (모델 다운로드 포함)

---

### 2.2 llama.cpp 빌드 (Planning 문서 준수)

```bash
# 1. 소스 클론
cd /home/sang/dev_ws/AI_secretary_robot/external
git clone https://github.com/ggerganov/llama.cpp.git
cd llama.cpp

# 2. CUDA 지원 빌드 (Jetson Orin Nano)
mkdir build && cd build
cmake .. \
  -DGGML_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=87 \
  -DBUILD_SHARED_LIBS=ON

cmake --build . --config Release -j$(nproc)

# 3. 서버 바이너리 확인
ls -lh bin/llama-server
# 기대 크기: ~50MB

# 4. 심볼릭 링크 생성 (편의)
sudo ln -sf $(pwd)/bin/llama-server /usr/local/bin/llama-server
```

**예상 소요 시간**: 45분 (빌드 시간)

---

### 2.3 Qwen2.5-1.5B GGUF 모델 다운로드

```bash
# 1. 모델 디렉토리 생성
mkdir -p /home/sang/dev_ws/AI_secretary_robot/models/llm
cd /home/sang/dev_ws/AI_secretary_robot/models/llm

# 2. GGUF 모델 다운로드 (Q4_K_M 양자화)
wget https://huggingface.co/Qwen/Qwen2.5-1.5B-Instruct-GGUF/resolve/main/qwen2.5-1.5b-instruct-q4_k_m.gguf

# 3. 모델 파일 확인
ls -lh qwen2.5-1.5b-instruct-q4_k_m.gguf
# 기대 크기: ~900MB

# 4. llama-server 실행 테스트
llama-server \
  --model qwen2.5-1.5b-instruct-q4_k_m.gguf \
  --port 8081 \
  -ngl 99 \
  -c 2048 \
  -t 4 &

# 5. 헬스체크
sleep 5
curl http://localhost:8081/health
# 기대 출력: {"status":"ok"}

# 6. 추론 테스트
curl http://localhost:8081/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [{"role": "user", "content": "안녕하세요"}],
    "temperature": 0.7,
    "max_tokens": 100
  }'

# 서버 종료
pkill llama-server
```

**예상 소요 시간**: 20분 (다운로드 시간)

---

### 2.4 LLM ROS2 노드 설정 업데이트

```bash
# params.yaml 수정
nano /home/sang/dev_ws/AI_secretary_robot/src/ai/llm_cpp/config/params.yaml
```

**추가할 파라미터**:
```yaml
llm_node:
  ros__parameters:
    # 기존 설정 유지...

    # llama.cpp 서버 설정 (신규)
    llama_server_binary: "/usr/local/bin/llama-server"
    llama_model_path: "/home/sang/dev_ws/AI_secretary_robot/models/llm/qwen2.5-1.5b-instruct-q4_k_m.gguf"
    llama_port: 8081
    llama_ngl: 99              # GPU 전체 레이어 오프로딩
    llama_ctx_size: 2048
    llama_threads: 4

    # Ollama fallback (기존)
    ollama_base_url: "http://localhost:11434"
    ollama_model: "qwen2.5:1.5b"
```

---

### 2.5 LLM 검증

```bash
# 1. Ollama 모드 테스트 (즉시 가능)
cd /home/sang/dev_ws/AI_secretary_robot
source install/setup.bash

# Ollama 서버 시작
ollama serve &

# ROS2 노드 실행
ros2 launch llm_cpp llm.launch.py

# 별도 터미널에서 테스트
ros2 topic pub /intent_router/chat_text std_msgs/String \
  "data: '로봇 팔 들어'" --once

# 기대 출력:
# - /llm/provider: "ollama"
# - /llm/response: "{\"intent\": \"manipulator_move\", ...}"

# 2. llama.cpp 모드 테스트 (llama-server 빌드 후)
# params.yaml에서 llama_server_binary 경로 설정 후 재시작
ros2 launch llm_cpp llm.launch.py

# 기대 출력 (로그):
# [INFO] [llm_node]: llama.cpp server started at http://localhost:8081
# [INFO] [llm_node]: LLM engine ready (provider: llama.cpp)

# 추론 테스트
ros2 topic pub /intent_router/chat_text std_msgs/String \
  "data: '자기소개 해줘'" --once

# 기대 출력:
# - /llm/provider: "llama.cpp"
# - /llm/response: "저는 로버입니다..."
```

---

## 🎯 복구 우선순위

### Phase 1: 즉시 실행 (30분)
1. ✅ **espeak-ng 설치** → TTS fallback 복구
2. ✅ **Ollama 설치** → LLM fallback 복구
3. ✅ **검증 테스트** → 음성 파이프라인 동작 확인

**목표**: 최소한의 음성 파이프라인 동작 (espeak + Ollama)

---

### Phase 2: 중기 개선 (2시간)
1. ⚠️ **Piper 바이너리 설치** → TTS 음질 개선
2. ⚠️ **llama.cpp 빌드** → LLM 로컬 추론 경로 확보
3. ⚠️ **Qwen GGUF 다운로드** → Planning 문서 준수

**목표**: Planning 문서 아키텍처 50% 달성

---

### Phase 3: 장기 완성 (1일)
1. 🔵 **sherpa-onnx TTS 마이그레이션** → Piper CLI 제거
2. 🔵 **LlamaServerManager 구현** → 서버 생명주기 관리
3. 🔵 **전체 통합 테스트** → 100% Planning 준수

**목표**: 3개 Codex 프롬프트 완전 구현

---

## 📋 최종 체크리스트

### TTS 복구
- [ ] espeak-ng 설치 완료 (`espeak-ng --version`)
- [ ] edge-tts 모듈 설치 완료 (`python3 -c "import edge_tts"`)
- [ ] piper 바이너리 설치 완료 (`piper --version`)
- [ ] TTS 노드 실행 성공 (`ros2 launch tts_cpp tts.launch.py`)
- [ ] espeak-ng TTS 테스트 통과
- [ ] Piper TTS 테스트 통과 (piper 설치 후)

### LLM 복구
- [ ] Ollama 설치 완료 (`ollama --version`)
- [ ] Qwen2.5:1.5b 모델 다운로드 (`ollama list`)
- [ ] llama.cpp 빌드 완료 (`llama-server --version`)
- [ ] Qwen GGUF 모델 다운로드 (900MB)
- [ ] LLM 노드 실행 성공 (Ollama 모드)
- [ ] llama.cpp 서버 시작 성공 (로컬 모드)
- [ ] LLM 추론 테스트 통과

### 전체 파이프라인
- [ ] Wake Word 검출 → STT → LLM → TTS 통합 테스트
- [ ] 실시간 음성 명령 처리 확인
- [ ] GPU 메모리 사용량 확인 (< 3GB)
- [ ] Planning 문서와 일치도 80% 이상

---

## 🚨 트러블슈팅

### espeak-ng 한국어 음성 품질 불량
**증상**: 한국어 발음이 부자연스러움
**해결**: Piper TTS 우선 사용, espeak-ng는 최종 fallback만

### Piper 모델 로딩 실패
**증상**: `Error loading model` 메시지
**해결**:
```bash
# 모델 파일 권한 확인
chmod 644 /home/sang/dev_ws/AI_secretary_robot/models/tts/neurlang_piper_onnx_kss_korean/*.onnx

# espeak-ng-data 경로 확인
ls /usr/lib/aarch64-linux-gnu/espeak-ng-data/
```

### llama-server OOM (메모리 부족)
**증상**: 서버 시작 직후 종료됨
**해결**:
```bash
# 1. ZRAM 확인
swapon -s

# 2. ngl 값 감소 (GPU 레이어 줄이기)
llama-server --model model.gguf -ngl 50  # 99 → 50

# 3. Context 윈도우 감소
llama-server --model model.gguf -c 1024  # 2048 → 1024
```

### Ollama 모델 다운로드 느림
**증상**: `ollama pull` 진행 없음
**해결**:
```bash
# 1. 프록시 확인
curl -I https://ollama.com

# 2. 수동 다운로드
cd ~/.ollama/models
wget https://huggingface.co/...  # Hugging Face에서 직접 다운로드

# 3. Ollama 재시작
sudo systemctl restart ollama
```

---

## 📞 긴급 지원

**상황별 대응**:
1. **TTS 완전 실패** → espeak-ng만 사용, 음질 타협
2. **LLM 로컬 실패** → Ollama fallback 의존, 인터넷 필요 시 클라우드 API
3. **전체 파이프라인 크래시** → 개별 노드 순차 실행, 로그 분석

**로그 수집**:
```bash
# ROS2 로그 확인
cat ~/.ros/log/latest/tts_node/stdout.log
cat ~/.ros/log/latest/llm_node/stdout.log

# llama-server 로그
cat /tmp/llama_server_8081.log

# 시스템 리소스
tegrastats | head -20
```

---

**작성자 노트**:
본 가이드는 마이그레이션 검증 실패 후 **즉시 복구 가능한 경로**를 우선 제시합니다. Phase 1만 완료해도 기본 음성 파이프라인은 동작하며, Phase 2/3는 Planning 문서 준수를 위한 점진적 개선입니다. **espeak-ng + Ollama 조합은 안정성 최우선 fallback**으로, 프로덕션 환경에서도 사용 가능합니다.
