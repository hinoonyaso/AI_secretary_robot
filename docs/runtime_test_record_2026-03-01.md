# STT/LLM/TTS 런타임 테스트 기록

- 작성일: 2026-03-01
- 작업 경로: `/home/sang/dev_ws/AI_secretary_robot`
- 목적: STT, LLM, TTS 실제 동작 여부 확인

## 1) 공통 빌드/기동 상태

### 실행 명령
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select stt_cpp tts_cpp llm_cpp
```

### 결과
- 성공 (`stt_cpp`, `tts_cpp`, `llm_cpp` 빌드 완료)

---

## 2) STT 동작 테스트

### 테스트 A: AudioBuffer 더미 입력

#### 실행 명령
```bash
ros2 launch stt_cpp stt.launch.py
ros2 topic pub --once /wake_vad/audio_buffer ros_robot_controller_msgs/msg/AudioBuffer \
  "{samples: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], sample_rate: 16000, channels: 1, session_id: test-session}"
```

#### 결과
- 노드 기동: 성공
- 입력 처리: 수행됨
- 인식 결과: 실패

#### 핵심 로그
- `stt_cpp node started`
- `stt failed for audio buffer: http_400 ... Audio file is too short. Minimum audio length is 0.01 seconds.`

---

### 테스트 B: WAV 파일 경로 입력

#### 실행 명령
```bash
ros2 launch stt_cpp stt.launch.py
ros2 topic pub --once /wake_vad/audio_path std_msgs/msg/String \
  "{data: '/home/sang/dev_ws/AI_secretary_robot/src/ai/tts_cpp/output/piper_cpp_test.wav'}"
```

#### 결과
- 성공

#### 핵심 로그
- `stt_cpp node started`
- `stt(groq): HyperCPP 테스트 헤드미더`

---

## 3) LLM 동작 테스트

### 테스트: chat_text 입력 후 처리 확인

#### 실행 명령
```bash
ros2 launch llm_cpp llm.launch.py
ros2 topic pub --once /intent_router/chat_text std_msgs/msg/String "{data: '테스트 메시지'}"
```

#### 결과
- 노드 기동: 성공
- 요청 처리: 성공 (fallback 경로로 응답 생성)
- 로컬 llama.cpp 서버 자동기동: 현재 환경에서는 스킵

#### 핵심 로그
- `Skip llama.cpp autostart (binary/model not found). binary=missing model=missing`
- `llm_cpp node started`
- `llm provider=openai (fallback)`

---

## 4) TTS 동작 테스트

### 테스트 A: /llm/response 입력

#### 실행 명령
```bash
ros2 launch tts_cpp tts.launch.py
ros2 topic pub --once /llm/response std_msgs/msg/String "{data: '안녕하세요 테스트 음성 출력'}"
```

#### 결과
- 노드 기동: 성공
- 합성: 실패

#### 핵심 로그
- `tts_cpp node started`
- `tts failed: espeak failed: espeak_failed(exit=127): sh: 1: espeak-ng: not found`

---

### 테스트 B: edge-tts 스크립트 직접 실행 가능 여부

#### 실행 명령
```bash
python3 src/ai/tts_cpp/scripts/edge_tts_synth.py \
  --text '테스트' \
  --output /tmp/edge_tts_test.mp3 \
  --voice ko-KR-SunHiNeural \
  --rate +0% \
  --volume +0%
```

#### 결과
- 실패

#### 핵심 로그
- `edge_tts_failed: No module named 'edge_tts'`

---

## 5) 최종 판정

- STT: 동작 확인 완료 (Groq 경로)
- LLM: 동작 확인 완료 (OpenAI fallback 경로)
- TTS: 미통과 (현재 환경에 사용 가능한 TTS 엔진 없음: `espeak-ng` 미설치, `edge_tts` 미설치)

## 6) 후속 작업

1. `espeak-ng` 설치 또는 `edge_tts` Python 패키지 설치
2. sherpa-onnx 호환 Piper 모델(메타데이터 포함) 준비 시 Piper 경로 재검증
3. llama.cpp 로컬 실행 검증을 위해 다음 경로에 바이너리/모델 준비
   - `/home/ubuntu/external/llama.cpp/build/bin/llama-server`
   - `/home/ubuntu/models/qwen2.5-1.5b-instruct-q4_k_m.gguf`
