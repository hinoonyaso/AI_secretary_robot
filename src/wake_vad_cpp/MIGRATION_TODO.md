# wake_vad_cpp 구현 TODO

## 0. 아키텍처 확정 (2026-02-13)

- [x] 최종 방향 확정: **C++ 네이티브 구현**
- [x] 목표 스택 정의: Wakeword는 **Porcupine C SDK**
- [x] 목표 스택 정의: VAD는 **ONNX Runtime + Silero ONNX 모델**
- [x] 정책 정의: 최종본에서 Python 임베딩(`pvporcupine`, `torch`) 미사용

참고: Porcupine C SDK + ONNX Runtime 네이티브 경로로 전환 완료.

현재 `wake_vad_cpp`는 **네이티브 엔진 연동 완료 상태**이며, 아래 미완료 항목은 실기기 동작 검증 단계입니다.

## 1. 필수 구현 파일

### `src/wake_vad_cpp/src/audio_input.cpp`
- [x] 실제 오디오 입력 백엔드 연결 (PortAudio 또는 ALSA)
- [x] `16kHz`, `mono`, `int16`, `frame_length=512` 설정 적용
- [x] 마이크 디바이스 선택 로직 구현
- [x] 오디오 프레임 수신 시 등록된 callback 호출
- [x] `start()/stop()`에서 리소스 정상 생성/해제

### `src/wake_vad_cpp/src/porcupine_engine.cpp`
- [x] Porcupine C/C++ SDK 연동
- [x] `initialize()`에서 `access_key`, `.ppn`, `.pv`, `sensitivity` 반영
- [x] `process()`에서 프레임 단위 wakeword 감지 결과 반환
- [x] `shutdown()`에서 Porcupine 핸들 해제
- [x] 에러 코드/로그 처리 추가

### `src/wake_vad_cpp/src/vad_engine.cpp`
- [x] Silero VAD C++ 경로 확정
- [x] 방법 A: ONNX Runtime + silero onnx 모델 로딩
- [x] `initialize()`에서 threshold/model_path 반영
- [x] `is_speech()`에서 프레임 추론 후 threshold 비교
- [x] `reset()`에서 상태 초기화

### `src/wake_vad_cpp/src/wav_writer.cpp`
- [x] WAV 포맷으로 저장되도록 구현 (`RIFF/WAVE` 헤더 포함)
- [x] 샘플 포맷: `PCM 16-bit`, `mono`, `16kHz`
- [x] 현재 raw dump 방식 제거
- [x] 파일 쓰기 실패 시 false 반환 + 로그 확인

## 2. 노드 로직 점검

### `src/wake_vad_cpp/src/wake_vad_node.cpp`
- [ ] `WAITING -> LISTENING -> RECORDING` 전이 조건 실제 동작 검증
- [x] `listen_timeout`, `silence_duration`, `max_record_duration` 정확히 반영
- [ ] `/wake_vad/detected` publish 시점 검증
- [ ] `/wake_vad/audio_path` publish 시점 검증
- [x] `/wake_vad/state` 상태 문자열 검증

## 3. 파라미터/경로 점검

### `src/wake_vad_cpp/config/params.yaml`
- [x] `access_key`를 평문 대신 환경변수 기반으로 운용
- [x] `keyword_path`, `porcupine_model_path` 실제 배포 경로에 맞춤
- [x] 필요 시 `vad_model_path` 값 설정

## 4. CMake / 의존성 추가

### `src/wake_vad_cpp/CMakeLists.txt`
- [x] 오디오/VAD/Porcupine 라이브러리 링크 추가
- [x] include/lib 경로 등록
- [x] 필요 시 `find_package()` 추가 (예: ONNX Runtime wrapper)

### `src/wake_vad_cpp/package.xml`
- [x] 런타임에 필요한 시스템 의존성 문서화
- [x] 필요 시 ROS 의존성 추가

## 5. 기능 검증 절차

- [x] 빌드
  ```bash
  cd ~/rover_ws
  source /opt/ros/humble/setup.bash
  colcon build --packages-select wake_vad_cpp
  source ~/rover_ws/install/setup.bash
  ```

- [x] 실행
  ```bash
  ros2 launch wake_vad_cpp wake_vad.launch.py
  ```

- [x] 토픽 확인
  ```bash
  ros2 topic echo /wake_vad/state
  ros2 topic echo /wake_vad/detected
  ros2 topic echo /wake_vad/audio_path
  ```

- [ ] WAV 결과 확인
  - `/tmp/wake_vad_audio` 아래 파일 생성 확인
  - `file <wav파일>` 명령으로 WAV 헤더 정상 확인

## 6. 완료 기준 (Definition of Done)

- [ ] 웨이크워드 감지 시 `/wake_vad/detected=true` 발행
- [ ] 발화 구간만 녹음되어 WAV 저장
- [ ] 저장 경로가 `/wake_vad/audio_path`로 발행
- [ ] 무발화 timeout / 무음 종료 / 최대 길이 종료 모두 정상
- [ ] 노드 종료 시 리소스(스트림/엔진) 누수 없음
