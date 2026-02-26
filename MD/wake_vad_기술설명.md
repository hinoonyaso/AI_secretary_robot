# wake_vad에서 사용된 기술 상세 설명

## 1. 전체 아키텍처
`wake_vad_py`는 ROS2 Python 노드(`wake_vad_node`)로 구현되어 있으며, 파이프라인은 아래와 같습니다.

1. 마이크 입력(16kHz, mono, int16)
2. 웨이크워드 감지(Picovoice Porcupine)
3. 음성 시작/종료 판단(Silero VAD)
4. 음성 구간 WAV 저장
5. ROS2 토픽으로 이벤트/결과 발행

구현 파일 기준:
- `src/wake_vad_py/wake_vad_py/wake_vad_node.py`
- `src/wake_vad_py/launch/wake_vad.launch.py`
- `src/wake_vad_py/config/params.yaml`

## 2. ROS2 기술 요소
### 2.1 ROS2 노드/런치
- 프레임워크: `rclpy` 기반 `ament_python` 패키지
- 런치: `wake_vad.launch.py`에서 `wake_vad_node` 실행 + `params.yaml` 주입
- 패키지 선언: `package.xml`에서 `rclpy`, `std_msgs` 의존성 선언

### 2.2 ROS2 토픽 인터페이스
노드는 아래 토픽을 퍼블리시합니다.

- `/wake_vad/detected` (`std_msgs/Bool`): 웨이크워드 감지 신호
- `/wake_vad/audio_path` (`std_msgs/String`): 저장된 WAV 경로
- `/wake_vad/state` (`std_msgs/String`): 내부 상태(`waiting/listening/recording`)

### 2.3 파라미터 기반 운용
`declare_parameter()` + YAML 파일로 동작을 제어합니다.

주요 파라미터:
- `access_key`: Porcupine 인증키
- `keyword_path`: 커스텀 키워드 `.ppn` 파일(또는 디렉토리)
- `porcupine_model_path`: 언어별 `.pv` 모델
- `audio_device_index`: 오디오 장치 인덱스(-1이면 자동 탐색)
- `sensitivity`: 웨이크워드 민감도
- `vad_threshold`: VAD 음성 판정 임계값
- `silence_duration`: 무음 종료 임계 시간
- `max_record_duration`: 최대 녹음 시간
- `listen_timeout`: 웨이크 후 발화 시작 대기 시간

## 3. 오디오 입출력/신호 처리 기술
### 3.1 실시간 캡처
- 라이브러리: `sounddevice` (PortAudio 래퍼)
- 스트림: `InputStream` 사용
- 포맷: `samplerate=16000`, `channels=1`, `dtype='int16'`
- 블록 크기: Porcupine 프레임 길이(`frame_length`, 보통 512)에 맞춤

### 3.2 장치 자동 선택 로직
XFM-DP 마이크 우선 정책을 구현합니다.

1. 장치명에 `xfm-dp` 계열 문자열이 있고 16k 입력 가능하면 우선 선택
2. 없으면 USB 입력 장치 중 16k 가능 장치 선택
3. 없으면 기본 입력 장치 사용 시도
4. 마지막으로 16k 가능한 첫 입력 장치 선택

검증에는 `sd.check_input_settings()`를 사용하여 실제 16k mono int16 지원 여부를 확인합니다.

### 3.3 스레딩/큐 기반 비동기 처리
- 오디오 콜백 스레드: 프레임을 `queue.Queue`에 push
- 처리 스레드: 별도 daemon thread에서 pop 후 상태별 처리

이 구조는 오디오 캡처와 ML 추론/상태 전환을 분리해, 콜백 지연으로 인한 오디오 드롭을 줄이는 목적입니다.

## 4. Wake Word 기술 (Picovoice Porcupine)
### 4.1 엔진
- 엔진: `pvporcupine`
- 방식: 고정 길이 PCM 프레임 기반 온디바이스 키워드 감지
- 입력: `int16` PCM 샘플 리스트

### 4.2 모델/키워드 로딩 전략
- `access_key`가 없으면 엔진 초기화 실패
- `keyword_path`가 파일이면 해당 `.ppn` 사용
- `keyword_path`가 디렉토리면 내부 `.ppn` 자동 선택
- 커스텀 키워드 초기화 실패 시 built-in 키워드(`picovoice`)로 폴백
- 한국어 키워드 `.ppn` 사용 시 한국어 모델(`porcupine_params_ko.pv`) 지정 권장 경고 포함

즉, 운영 중 경로/모델 오류가 있어도 완전 중단 대신 폴백으로 동작 지속성을 확보한 구조입니다.

## 5. VAD 기술 (Silero VAD)
### 5.1 모델 로딩
- 프레임워크: PyTorch
- 로딩: `torch.hub.load('snakers4/silero-vad', 'silero_vad')`
- 옵션: `trust_repo=True`, `force_reload=False`

### 5.2 추론 입력 전처리
Silero 입력 규격에 맞춰 프레임 변환:

1. `int16` PCM -> `float32`
2. 스케일링: `/32768.0`으로 `[-1, 1]` 정규화
3. `torch.from_numpy()`로 텐서화
4. `model(tensor, 16000)` 호출
5. 반환 confidence를 `vad_threshold`와 비교해 음성 여부 결정

### 5.3 상태 리셋
구간 종료 시 `reset_states()`를 호출해 다음 발화에 이전 내부 상태가 누적되지 않도록 관리합니다.

## 6. 상태머신(State Machine) 설계
노드의 핵심 제어는 3상태 FSM입니다.

1. `WAITING`
2. `LISTENING`
3. `RECORDING`

전이 규칙:
- `WAITING -> LISTENING`: Porcupine 웨이크워드 감지 성공
- `LISTENING -> RECORDING`: VAD가 발화 시작 감지
- `LISTENING -> WAITING`: `listen_timeout` 내 발화 없음
- `RECORDING -> WAITING`: 무음이 `silence_duration` 이상 지속 또는 `max_record_duration` 도달

이 구조는 “항상 녹음”이 아닌 “웨이크 후 발화 구간만 녹음” 전략이라 저장량과 후단 처리량을 줄이는 데 유리합니다.

## 7. 녹음 저장 및 파일 처리
- 버퍼: `numpy` 프레임 리스트 누적 후 `np.concatenate()`
- 저장: `soundfile.write()`로 `PCM_16` WAV 생성
- 파일명: `wake_vad_YYYYMMDD_HHMMSS.wav`
- 결과 전달: 저장 경로를 `/wake_vad/audio_path`로 퍼블리시

## 8. 장애 대응/운영 안정성 포인트
코드에 반영된 안정화 기법:
- 장치 탐색 실패/포맷 불일치 로깅
- Porcupine 커스텀 로딩 실패 시 built-in 키워드 폴백
- VAD 로딩 실패 시 제한적 동작 유지(리스닝에서 바로 녹음 진입 가능 경로)
- 처리 루프/콜백 예외 로깅
- 종료 시 스트림 중지/해제 + Porcupine 리소스 해제

## 9. 기술 스택 요약
- 런타임/미들웨어: ROS2 (`rclpy`, `std_msgs`, launch system)
- 웨이크워드: Picovoice Porcupine (`pvporcupine`, `.ppn`, `.pv`)
- VAD: Silero VAD (PyTorch Hub, `snakers4/silero-vad`)
- 오디오 I/O: `sounddevice` (PortAudio)
- 오디오 파일 저장: `soundfile`
- 수치 처리: `numpy`
- 동시성: Python `threading` + `queue`

## 10. 한 줄 정리
`wake_vad`는 "ROS2 이벤트 파이프라인 + Porcupine 웨이크워드 + Silero VAD + 실시간 오디오 스트리밍/파일 저장"을 결합한, 온디바이스 음성 트리거형 녹음 노드입니다.
