# 🚨 긴급 복구 매뉴얼

**검증 실패 이슈**: TTS/LLM 의존성 부재
**복구 시간**: 5~30분
**작성일**: 2026-03-01

---

## 📋 현재 상태

### ✅ **정상 동작**
- **STT (Moonshine)**: sherpa-onnx 통합 완료, 실제 WAV 테스트 통과

### ⚠️ **의존성 부재**
- **TTS**: espeak-ng ❌, edge-tts ❌, piper ❌
- **LLM**: Ollama ❌, llama-server ❌, GGUF 모델 ❌

---

## ⚡ 즉시 해결 방법 (2가지 옵션)

### 옵션 1: 자동 스크립트 실행 (권장) ⭐

```bash
cd /home/sang/dev_ws/AI_secretary_robot

# TTS 의존성 설치 (2분)
bash scripts/setup_tts_dependencies.sh

# LLM 의존성 설치 (10분)
bash scripts/setup_llm_dependencies.sh

# 검증
source install/setup.bash
ros2 launch tts_cpp voice_pipeline_local.launch.py
```

**설치 내역**:
- ✅ espeak-ng (fallback TTS)
- ✅ edge-tts (클라우드 TTS)
- ✅ piper (로컬 고음질 TTS)
- ✅ Ollama (로컬 LLM)
- ✅ Qwen2.5:1.5b 모델
- ⚠️ llama.cpp (30분 빌드, 백그라운드)

---

### 옵션 2: 수동 단계별 설치

#### 1️⃣ TTS 복구 (필수)
```bash
# espeak-ng 설치 (필수 fallback)
sudo apt update
sudo apt install -y espeak-ng espeak-ng-data
espeak-ng -v ko "테스트" --stdout | aplay

# edge-tts 설치 (선택적)
pip3 install --user edge-tts

# piper 바이너리 설치 (권장)
cd /tmp
wget https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_arm64.tar.gz
tar -xzf piper_arm64.tar.gz
sudo cp piper/piper /usr/local/bin/
piper --version
```

#### 2️⃣ LLM 복구 (필수)
```bash
# Ollama 설치
curl -fsSL https://ollama.com/install.sh | sh
sudo systemctl start ollama

# Qwen2.5 모델 다운로드
ollama pull qwen2.5:1.5b
ollama list
```

---

## 🧪 검증 테스트

### TTS 검증
```bash
source /home/sang/dev_ws/AI_secretary_robot/install/setup.bash
ros2 launch tts_cpp tts.launch.py &
sleep 3
ros2 topic pub /tts/text std_msgs/String "data: '안녕하세요'" --once

# 기대 출력:
# - [tts_node]: TTS engine: espeak-ng
# - 스피커에서 음성 재생
```

### LLM 검증
```bash
ros2 launch llm_cpp llm.launch.py &
sleep 3
ros2 topic pub /intent_router/chat_text std_msgs/String "data: '자기소개해줘'" --once

# 기대 출력:
# - [llm_node]: LLM provider: ollama
# - /llm/response 토픽 발행
```

---

## 📊 복구 우선순위

### Phase 1: 최소 동작 (5분) 🔥
**목표**: espeak-ng + Ollama로 기본 파이프라인 동작

```bash
# 즉시 실행
bash scripts/setup_tts_dependencies.sh
bash scripts/setup_llm_dependencies.sh
```

**결과**:
- ✅ 음성 파이프라인 동작 (음질 낮음)
- ✅ LLM 추론 가능 (Ollama 의존)

---

### Phase 2: 음질 개선 (추가 0분) ⭐
**목표**: Piper TTS 사용

```bash
# Phase 1 완료 후 자동 사용 가능
ros2 param set /tts_node tts_engine "piper"
```

**결과**:
- ✅ 자연스러운 한국어 음성 (MOS 4.0/5.0)

---

### Phase 3: Planning 준수 (추가 30분) 🎯
**목표**: llama.cpp 직접 추론

```bash
# llama.cpp 빌드 (백그라운드 진행 중)
# scripts/setup_llm_dependencies.sh 실행 시 자동 빌드

# 빌드 완료 후:
llama-server --model /home/sang/dev_ws/AI_secretary_robot/models/llm/qwen2.5-1.5b-instruct-q4_k_m.gguf -ngl 99 &
```

**결과**:
- ✅ GPU 전체 오프로딩 (-ngl 99)
- ✅ Planning 문서 아키텍처 완전 준수

---

## 🔍 상세 문서

| 문서 | 설명 | 용도 |
|:---|:---|:---|
| **[QUICKSTART_RECOVERY.md](docs/QUICKSTART_RECOVERY.md)** | 5분 빠른 복구 가이드 | 즉시 실행 |
| **[migration_recovery_guide.md](docs/migration_recovery_guide.md)** | 전체 복구 가이드 | 상세 트러블슈팅 |
| **[codex_prompt_stt_sherpa_migration.md](docs/codex_prompt_stt_sherpa_migration.md)** | STT 마이그레이션 | 장기 개선 |
| **[codex_prompt_tts_sherpa_migration.md](docs/codex_prompt_tts_sherpa_migration.md)** | TTS 마이그레이션 | 장기 개선 |
| **[codex_prompt_llm_llamacpp_migration.md](docs/codex_prompt_llm_llamacpp_migration.md)** | LLM 마이그레이션 | 장기 개선 |

---

## 🎯 최종 목표

### 단기 (오늘)
- ✅ espeak-ng + Ollama로 기본 파이프라인 동작
- ✅ Piper TTS로 음질 개선
- ✅ 전체 음성 명령 처리 검증

### 중기 (이번 주)
- ⚠️ llama.cpp 직접 추론 전환
- ⚠️ sherpa-onnx TTS 마이그레이션
- ⚠️ LlamaServerManager 구현

### 장기 (이번 달)
- 🔵 Planning 문서 100% 준수
- 🔵 3개 Codex 프롬프트 완전 구현
- 🔵 성능 벤치마크 완료

---

## 🚀 지금 바로 시작하기

```bash
cd /home/sang/dev_ws/AI_secretary_robot

# 1단계: TTS 복구 (2분)
bash scripts/setup_tts_dependencies.sh

# 2단계: LLM 복구 (10분)
bash scripts/setup_llm_dependencies.sh

# 3단계: 검증
source install/setup.bash
ros2 launch tts_cpp voice_pipeline_local.launch.py

# 4단계: 테스트 (별도 터미널)
ros2 topic pub /tts/text std_msgs/String "data: '로버 준비 완료'" --once
```

**예상 결과**:
```
[wake_vad_node]: Wake word detector ready
[stt_node]: Moonshine ONNX ready
[intent_router_node]: Intent router ready
[llm_node]: LLM provider: ollama, ready
[tts_node]: TTS engine: piper, ready
[tts_node]: ♪ "로버 준비 완료" (음성 재생)
```

---

## 📞 긴급 지원

**문제 발생 시**:
1. [QUICKSTART_RECOVERY.md](docs/QUICKSTART_RECOVERY.md) 트러블슈팅 섹션 참조
2. 로그 확인: `cat ~/.ros/log/latest/*/stdout.log`
3. 전체 재시작: `pkill -f ros2 && ros2 launch ...`

**연락처**:
- 기술 문서: `docs/migration_recovery_guide.md`
- Planning 문서: `docs/plan.md`
- 프로젝트 상태: `PROJECT_STATE.md`

---

**작성자 노트**:
본 매뉴얼은 **검증 실패 후 최단 시간 복구**를 목표로 합니다. `scripts/setup_*.sh` 스크립트를 실행하면 모든 의존성이 자동 설치되며, Phase 1 완료만으로도 기본 음성 파이프라인이 동작합니다. Planning 문서 완전 준수는 Phase 3에서 달성됩니다.
