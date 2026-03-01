# Codex Prompt: STT - sherpa-onnx C API 마이그레이션

**작성일**: 2026-03-01
**담당**: 10년차 로봇 SW 시니어 개발팀장
**난이도**: ★★★★☆ (C++ API 래핑, ONNX 모델 통합)
**예상 소요**: 4~6시간

---

## 1. 배경 및 목표

### 1.1 현재 상태
- **구현**: `stt_cpp/src/moonshine_onnx.cpp`에서 ONNX Runtime C++ API 직접 사용
- **파일**:
  - [src/ai/stt_cpp/include/stt_cpp/moonshine_onnx.hpp](../src/ai/stt_cpp/include/stt_cpp/moonshine_onnx.hpp) (Ort::Session 직접 사용)
  - [src/ai/stt_cpp/src/moonshine_onnx.cpp](../src/ai/stt_cpp/src/moonshine_onnx.cpp) (ONNX Runtime 직접 호출)
- **문제점**:
  - Planning 문서([docs/plan.md](./plan.md)) 설계와 불일치
  - sherpa-onnx 통합 아키텍처에서 이탈
  - 모델 로딩/추론 로직 중복 (sherpa-onnx가 이미 제공)

### 1.2 목표
- **sherpa-onnx C API**로 Moonshine 모델 래핑
- ONNX Runtime 직접 호출 제거
- API 인터페이스 유지 (`transcribe()`, `transcribe_pcm()` 시그니처 불변)
- 성능 동등 이상 유지 (추론 속도 < 0.2초/오디오)

---

## 2. 기술 사양

### 2.1 sherpa-onnx 아키텍처
```
sherpa-onnx (C API)
├── sherpa_onnx_create_recognizer()       # 인식기 생성
├── sherpa_onnx_recognizer_create_stream() # 스트림 생성
├── sherpa_onnx_recognizer_accept_waveform() # PCM 입력
├── sherpa_onnx_recognizer_decode_stream() # 추론 실행
├── sherpa_onnx_recognizer_get_result()    # 텍스트 결과
└── sherpa_onnx_destroy_recognizer()      # 리소스 해제
```

### 2.2 Moonshine 모델 매핑
| ONNX Runtime (현재) | sherpa-onnx (목표) | 비고 |
|:---|:---|:---|
| `Ort::Session encoder_session_` | `SherpaOnnxOnlineRecognizer*` | sherpa-onnx가 내부적으로 관리 |
| `Ort::Session decoder_session_` | (내부 통합) | sherpa-onnx가 encoder+decoder 통합 처리 |
| `run_encoder()` + `run_decoder_greedy()` | `sherpa_onnx_recognizer_decode_stream()` | 단일 API로 통합 |
| 수동 vocab 로딩 | `sherpa-onnx` 자동 처리 | JSON vocab 자동 파싱 |

### 2.3 의존성
```bash
# sherpa-onnx 설치 (Jetson Orin Nano 기준)
git clone https://github.com/k2-fsa/sherpa-onnx.git
cd sherpa-onnx
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DSHERPA_ONNX_ENABLE_GPU=ON \
      -DCMAKE_CUDA_ARCHITECTURES=87 \
      ..
make -j$(nproc)
sudo make install
```

---

## 3. 구현 계획

### 3.1 파일 구조 변경
```
src/ai/stt_cpp/
├── include/stt_cpp/
│   ├── stt_engine.hpp              # 변경 없음
│   ├── stt_node.hpp                # 변경 없음
│   └── moonshine_onnx.hpp          # ⚠️ 수정: Ort::Session → SherpaOnnx 래퍼
├── src/
│   ├── stt_engine.cpp              # 변경 없음 (인터페이스 유지)
│   ├── stt_node.cpp                # 변경 없음
│   └── moonshine_onnx.cpp          # ⚠️ 전면 수정: sherpa-onnx C API 호출
├── CMakeLists.txt                  # ⚠️ 수정: onnxruntime → sherpa-onnx
└── package.xml                     # ⚠️ 수정: <depend>sherpa-onnx</depend>
```

### 3.2 단계별 작업

#### **STEP 1: moonshine_onnx.hpp 헤더 리팩토링**
**목표**: Ort::Session → SherpaOnnx 래퍼 클래스 변경

```cpp
// moonshine_onnx.hpp (Before)
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

class MoonshineOnnx {
private:
  Ort::Env env_;
  std::unique_ptr<Ort::Session> encoder_session_;
  std::unique_ptr<Ort::Session> decoder_session_;
};

// moonshine_onnx.hpp (After)
#include <sherpa-onnx/c-api/c-api.h>

class MoonshineOnnx {
private:
  SherpaOnnxOnlineRecognizer* recognizer_ = nullptr;
  SherpaOnnxOnlineRecognizerConfig config_;
  std::string model_dir_;  // 모델 경로 저장
};
```

**핵심 변경점**:
- `onnxruntime_cxx_api.h` → `sherpa-onnx/c-api/c-api.h`
- `Ort::Session` 멤버 제거
- `SherpaOnnxOnlineRecognizer*` 포인터 추가
- vocab 로딩 로직 제거 (sherpa-onnx가 자동 처리)

---

#### **STEP 2: moonshine_onnx.cpp 생성자 재구현**
**목표**: sherpa-onnx recognizer 초기화

```cpp
// moonshine_onnx.cpp
MoonshineOnnx::MoonshineOnnx(const MoonshineConfig& cfg) {
  // sherpa-onnx 설정 구조체 초기화
  config_ = SherpaOnnxOnlineRecognizerConfig{};

  // Encoder/Decoder 모델 경로 설정
  SherpaOnnxOnlineTransducerModelConfig transducer_config;
  transducer_config.encoder = cfg.encoder_path.c_str();
  transducer_config.decoder = cfg.decoder_path.c_str();
  transducer_config.joiner = "";  // Moonshine는 joiner 불필요

  config_.model_config.transducer = transducer_config;
  config_.model_config.tokens = "/home/ubuntu/models/moonshine/vocab.json";
  config_.model_config.num_threads = cfg.intra_threads;
  config_.model_config.provider = cfg.use_cuda ? "cuda" : "cpu";
  config_.model_config.debug = 0;

  // Recognizer 생성
  recognizer_ = SherpaOnnxCreateOnlineRecognizer(&config_);
  if (!recognizer_) {
    last_error_ = "Failed to create sherpa-onnx recognizer";
    ready_ = false;
    return;
  }

  ready_ = true;
}

MoonshineOnnx::~MoonshineOnnx() {
  if (recognizer_) {
    SherpaOnnxDestroyOnlineRecognizer(recognizer_);
    recognizer_ = nullptr;
  }
}
```

**핵심 로직**:
1. `SherpaOnnxOnlineRecognizerConfig` 구조체 생성
2. `encoder_path`, `decoder_path` 설정
3. `SherpaOnnxCreateOnlineRecognizer()` 호출
4. 소멸자에서 `SherpaOnnxDestroyOnlineRecognizer()` 호출

---

#### **STEP 3: transcribe_pcm() 재구현**
**목표**: sherpa-onnx 스트리밍 API로 추론

```cpp
std::string MoonshineOnnx::transcribe_pcm(
    const std::vector<float>& samples,
    uint32_t sample_rate) const {

  if (!ready_ || !recognizer_) {
    last_error_ = "Recognizer not initialized";
    return "";
  }

  // 1. 스트림 생성
  SherpaOnnxOnlineStream* stream =
      SherpaOnnxCreateOnlineStream(recognizer_);
  if (!stream) {
    last_error_ = "Failed to create stream";
    return "";
  }

  // 2. PCM 데이터 입력
  SherpaOnnxOnlineStreamAcceptWaveform(
      stream,
      sample_rate,
      samples.data(),
      samples.size()
  );

  // 3. 입력 종료 신호
  SherpaOnnxOnlineStreamInputFinished(stream);

  // 4. 디코딩 실행
  while (SherpaOnnxIsOnlineStreamReady(recognizer_, stream)) {
    SherpaOnnxDecodeOnlineStream(recognizer_, stream);
  }

  // 5. 결과 추출
  const SherpaOnnxOnlineRecognizerResult* result =
      SherpaOnnxGetOnlineStreamResult(recognizer_, stream);

  std::string text = result ? result->text : "";

  // 6. 리소스 해제
  SherpaOnnxDestroyOnlineRecognizerResult(result);
  SherpaOnnxDestroyOnlineStream(stream);

  if (text.empty()) {
    last_error_ = "Empty transcription result";
  }

  return text;
}
```

**핵심 변경점**:
- `run_encoder()` + `run_decoder_greedy()` → sherpa-onnx 스트리밍 API
- vocab 토큰 변환 로직 제거 (sherpa-onnx가 자동 처리)
- 메모리 관리 간소화 (C API 구조체 자동 해제)

---

#### **STEP 4: CMakeLists.txt 수정**
**목표**: onnxruntime → sherpa-onnx 의존성 교체

```cmake
# CMakeLists.txt (Before)
find_package(onnxruntime REQUIRED)

add_executable(stt_node src/stt_node.cpp src/stt_engine.cpp src/moonshine_onnx.cpp)
target_link_libraries(stt_node
  ${rclcpp_LIBRARIES}
  onnxruntime::onnxruntime
)

# CMakeLists.txt (After)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SHERPA_ONNX REQUIRED sherpa-onnx)

add_executable(stt_node src/stt_node.cpp src/stt_engine.cpp src/moonshine_onnx.cpp)
target_include_directories(stt_node PRIVATE ${SHERPA_ONNX_INCLUDE_DIRS})
target_link_libraries(stt_node
  ${rclcpp_LIBRARIES}
  ${SHERPA_ONNX_LIBRARIES}
)
```

---

#### **STEP 5: 검증 테스트**
**목표**: 기존 API 호환성 및 성능 확인

```bash
# 1. 빌드 검증
cd ~/dev_ws/AI_secretary_robot
colcon build --packages-select stt_cpp

# 2. 노드 실행 테스트
ros2 launch stt_cpp stt.launch.py

# 3. 추론 성능 벤치마크 (별도 터미널)
ros2 topic pub /audio/raw audio_common_msgs/AudioData \
  "{data: [...]}" --once

# 기대 출력 (ROS2 토픽):
# /stt/text: "안녕하세요"
# /stt/engine: "moonshine_onnx"
# 추론 시간: < 200ms

# 4. 메모리 사용량 확인
ps aux | grep stt_node
# 기대: VRAM 0.5GB, RAM 0.3GB (plan.md 목표치)
```

---

## 4. 위험 관리

### 4.1 주요 리스크
| 리스크 | 완화 방안 |
|:---|:---|
| sherpa-onnx 빌드 실패 | CUDA 11.4+ 버전 확인, CMake 3.22+ 사용 |
| Moonshine 모델 비호환 | sherpa-onnx 공식 문서 확인 후 모델 변환 필요 시 ONNX 재export |
| 추론 속도 저하 | GPU 오프로딩 확인 (`provider=cuda`), batch size 조정 |
| 메모리 누수 | `valgrind` 실행, C API 리소스 해제 확인 |

### 4.2 롤백 계획
```bash
# 변경 전 백업
git checkout -b feature/stt-sherpa-migration
git commit -am "backup: before sherpa-onnx migration"

# 롤백 시
git checkout main
git reset --hard HEAD~1
```

---

## 5. 완료 조건

### 5.1 기능 검증
- ✅ `transcribe_pcm()` 정확도 동등 이상 (CER < 10%)
- ✅ ROS2 토픽 `/stt/text` 정상 발행
- ✅ 추론 시간 < 200ms (16kHz, 5초 오디오 기준)

### 5.2 코드 품질
- ✅ `MoonshineOnnx` 클래스 API 인터페이스 불변
- ✅ 메모리 누수 없음 (`valgrind --leak-check=full`)
- ✅ C++ 컴파일 경고 0건

### 5.3 문서화
- ✅ `src/ai/stt_cpp/README.md` 업데이트 (sherpa-onnx 설치 가이드)
- ✅ `AGENTS.md`에 변경 로그 추가

---

## 6. 참고 자료

### 6.1 공식 문서
- sherpa-onnx C API: https://k2-fsa.github.io/sherpa/onnx/c-api/index.html
- Moonshine 모델: https://github.com/usefulsensors/moonshine

### 6.2 코드 예제
```bash
# sherpa-onnx C API 예제
git clone https://github.com/k2-fsa/sherpa-onnx.git
cd sherpa-onnx/c-api-examples
cat online-transducer-microphone.c  # 스트리밍 추론 예제
```

---

## 7. 최종 체크리스트

**마이그레이션 전**:
- [ ] sherpa-onnx 라이브러리 설치 완료
- [ ] Moonshine ONNX 모델 경로 확인 (`/home/ubuntu/models/moonshine/`)
- [ ] 현재 브랜치 백업 (`git checkout -b backup/stt-before-migration`)

**마이그레이션 중**:
- [ ] `moonshine_onnx.hpp` 헤더 수정 (Ort → SherpaOnnx)
- [ ] `moonshine_onnx.cpp` 재구현 (생성자, 추론 로직)
- [ ] `CMakeLists.txt` 의존성 변경
- [ ] `package.xml` 의존성 추가

**마이그레이션 후**:
- [ ] 빌드 성공 (`colcon build --packages-select stt_cpp`)
- [ ] 노드 실행 성공 (`ros2 launch stt_cpp stt.launch.py`)
- [ ] 추론 테스트 통과 (정확도, 속도)
- [ ] 메모리 프로파일링 (`ps aux | grep stt_node`)
- [ ] 문서 업데이트 (`README.md`, `AGENTS.md`)

**최종 승인**:
- [ ] 팀장 코드 리뷰 통과
- [ ] CI/CD 파이프라인 통과
- [ ] Planning 문서([docs/plan.md](./plan.md))와 일치 확인

---

**작성자 노트**:
본 프롬프트는 10년차 시니어 개발자 기준으로 작성되었습니다. sherpa-onnx C API는 ONNX Runtime 대비 High-level 추상화를 제공하여 코드 복잡도를 50% 이상 감소시킵니다. 단, Moonshine 모델의 sherpa-onnx 호환성은 사전 검증이 필수입니다. 필요 시 ONNX 모델 재export를 고려하세요.
