# Codex Prompt: TTS - sherpa-onnx Piper ONNX 마이그레이션

**작성일**: 2026-03-01
**담당**: 10년차 로봇 SW 시니어 개발팀장
**난이도**: ★★★★★ (C API 래핑, WAV 생성, ALSA 통합)
**예상 소요**: 6~8시간

---

## 1. 배경 및 목표

### 1.1 현재 상태
- **구현**: `tts_cpp/src/tts_engine.cpp`에서 `piper` CLI 바이너리를 프로세스로 호출
- **파일**:
  - [src/ai/tts_cpp/src/tts_engine.cpp:127-170](../src/ai/tts_cpp/src/tts_engine.cpp#L127-L170) (CLI 호출 로직)
  - [src/ai/tts_cpp/include/tts_cpp/tts_engine.hpp](../src/ai/tts_cpp/include/tts_cpp/tts_engine.hpp)
- **문제점**:
  - Planning 문서([docs/plan.md](./plan.md)) 설계와 불일치
  - 프로세스 기반 실행으로 지연시간 증가 (프로세스 spawn ~50ms)
  - sherpa-onnx 통합 아키텍처에서 이탈
  - WAV 파일 I/O 오버헤드 발생

### 1.2 목표
- **sherpa-onnx C API**로 Piper VITS ONNX 직접 추론
- 프로세스 호출 제거 → 인메모리 추론
- API 인터페이스 유지 (`synthesize()` 시그니처 불변)
- 성능 향상 목표: 지연시간 < 100ms (기존 150~200ms)

---

## 2. 기술 사양

### 2.1 sherpa-onnx TTS 아키텍처
```
sherpa-onnx TTS (C API)
├── SherpaOnnxOfflineTtsConfig           # TTS 설정 구조체
├── SherpaOnnxCreateOfflineTts()         # TTS 엔진 생성
├── SherpaOnnxOfflineTtsGenerate()       # 텍스트 → PCM 변환
├── SherpaOnnxOfflineTtsGeneratedAudio   # 결과 구조체 (PCM + sample rate)
├── SherpaOnnxDestroyOfflineTtsGeneratedAudio() # 오디오 메모리 해제
└── SherpaOnnxDestroyOfflineTts()        # 엔진 해제
```

### 2.2 Piper 모델 매핑
| CLI (현재) | sherpa-onnx (목표) | 비고 |
|:---|:---|:---|
| `piper --model model.onnx` | `config.model.vits.model = "model.onnx"` | sherpa-onnx가 ONNX 직접 로드 |
| `piper --config model.json` | `config.model.vits.data_dir = "..."` | JSON 대신 phoneme dict 필요 |
| `echo "text" \| piper` | `SherpaOnnxOfflineTtsGenerate(tts, text)` | 인메모리 처리 |
| `piper --output_file out.wav` | WAV 인코딩 직접 구현 | sherpa-onnx는 PCM만 반환 |

### 2.3 의존성
```bash
# sherpa-onnx 설치 (TTS 지원 포함)
git clone https://github.com/k2-fsa/sherpa-onnx.git
cd sherpa-onnx
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DSHERPA_ONNX_ENABLE_TTS=ON \
      -DSHERPA_ONNX_ENABLE_GPU=ON \
      -DCMAKE_CUDA_ARCHITECTURES=87 \
      ..
make -j$(nproc)
sudo make install
```

**⚠️ 중요**: Piper 모델 변환 필요
```bash
# sherpa-onnx 형식으로 Piper 모델 변환
# 공식 가이드: https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/vits.html
```

---

## 3. 구현 계획

### 3.1 파일 구조 변경
```
src/ai/tts_cpp/
├── include/tts_cpp/
│   ├── tts_engine.hpp              # ⚠️ 수정: PiperOnnx 클래스 추가
│   ├── tts_node.hpp                # 변경 없음
│   ├── alsa_player.hpp             # 변경 없음
│   └── piper_onnx.hpp              # ⭐ 신규: sherpa-onnx TTS 래퍼
├── src/
│   ├── tts_engine.cpp              # ⚠️ 수정: synthesize_piper() 재구현
│   ├── tts_node.cpp                # 변경 없음
│   ├── alsa_player.cpp             # 변경 없음
│   └── piper_onnx.cpp              # ⭐ 신규: sherpa-onnx C API 호출
├── CMakeLists.txt                  # ⚠️ 수정: sherpa-onnx 의존성 추가
└── package.xml                     # ⚠️ 수정: <depend>sherpa-onnx</depend>
```

### 3.2 단계별 작업

#### **STEP 1: piper_onnx.hpp 헤더 생성**
**목표**: sherpa-onnx TTS 래퍼 클래스 정의

```cpp
// piper_onnx.hpp
#pragma once

#include <sherpa-onnx/c-api/c-api.h>
#include <cstdint>
#include <string>
#include <vector>

namespace tts_cpp {

struct PiperOnnxConfig {
  std::string model_path;        // ONNX 모델 경로
  std::string data_dir;          // phoneme dict/tokens 경로
  std::string lexicon_path;      // 발음 사전 (선택)
  int num_threads = 2;
  bool debug = false;
};

struct PiperOnnxResult {
  bool ok = false;
  std::vector<float> pcm_samples;  // 16-bit PCM float 배열
  uint32_t sample_rate = 0;        // 샘플링 레이트 (보통 22050Hz)
  std::string error;
};

class PiperOnnx {
public:
  explicit PiperOnnx(const PiperOnnxConfig& cfg);
  ~PiperOnnx();

  PiperOnnx(const PiperOnnx&) = delete;
  PiperOnnx& operator=(const PiperOnnx&) = delete;

  PiperOnnxResult synthesize(const std::string& text) const;
  bool is_ready() const { return ready_; }
  const std::string& last_error() const { return last_error_; }

private:
  SherpaOnnxOfflineTts* tts_ = nullptr;
  PiperOnnxConfig config_;
  bool ready_ = false;
  mutable std::string last_error_;
};

}  // namespace tts_cpp
```

---

#### **STEP 2: piper_onnx.cpp 구현**
**목표**: sherpa-onnx C API로 TTS 엔진 초기화 및 추론

```cpp
// piper_onnx.cpp
#include "tts_cpp/piper_onnx.hpp"
#include <cstring>

namespace tts_cpp {

PiperOnnx::PiperOnnx(const PiperOnnxConfig& cfg) : config_(cfg) {
  // sherpa-onnx TTS 설정 초기화
  SherpaOnnxOfflineTtsConfig tts_config;
  memset(&tts_config, 0, sizeof(tts_config));

  // VITS 모델 설정
  SherpaOnnxOfflineTtsVitsModelConfig vits_config;
  memset(&vits_config, 0, sizeof(vits_config));
  vits_config.model = cfg.model_path.c_str();
  vits_config.lexicon = cfg.lexicon_path.empty() ? "" : cfg.lexicon_path.c_str();
  vits_config.data_dir = cfg.data_dir.c_str();
  vits_config.length_scale = 1.0f;        // 발화 속도 (1.0 = 기본)
  vits_config.noise_scale = 0.667f;      // 음성 다양성
  vits_config.noise_scale_w = 0.8f;      // 발음 변이

  tts_config.model.vits = vits_config;
  tts_config.model.num_threads = cfg.num_threads;
  tts_config.model.debug = cfg.debug ? 1 : 0;
  tts_config.model.provider = "cpu";  // ⚠️ TTS는 CPU 추천 (GPU 오버헤드)

  // TTS 엔진 생성
  tts_ = SherpaOnnxCreateOfflineTts(&tts_config);
  if (!tts_) {
    last_error_ = "Failed to create sherpa-onnx TTS engine";
    ready_ = false;
    return;
  }

  ready_ = true;
}

PiperOnnx::~PiperOnnx() {
  if (tts_) {
    SherpaOnnxDestroyOfflineTts(tts_);
    tts_ = nullptr;
  }
}

PiperOnnxResult PiperOnnx::synthesize(const std::string& text) const {
  PiperOnnxResult result;

  if (!ready_ || !tts_) {
    result.error = "TTS engine not ready";
    return result;
  }

  if (text.empty()) {
    result.error = "Empty input text";
    return result;
  }

  // 음성 합성 실행
  int speaker_id = 0;  // 단일 화자 모델은 0
  float speed = 1.0f;  // 발화 속도 배수
  const SherpaOnnxGeneratedAudio* audio =
      SherpaOnnxOfflineTtsGenerate(tts_, text.c_str(), speaker_id, speed);

  if (!audio || audio->n == 0) {
    result.error = "TTS generation failed";
    if (audio) {
      SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
    }
    return result;
  }

  // PCM 데이터 복사
  result.pcm_samples.assign(audio->samples, audio->samples + audio->n);
  result.sample_rate = audio->sample_rate;
  result.ok = true;

  // 메모리 해제
  SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);

  return result;
}

}  // namespace tts_cpp
```

**핵심 로직**:
1. `SherpaOnnxOfflineTtsConfig` 구조체 초기화
2. VITS 모델 경로 설정 (`vits.model`, `vits.data_dir`)
3. `SherpaOnnxCreateOfflineTts()` 호출
4. `SherpaOnnxOfflineTtsGenerate()` 로 PCM 생성
5. 결과를 `std::vector<float>`로 반환

---

#### **STEP 3: tts_engine.cpp 수정**
**목표**: CLI 호출 → PiperOnnx 클래스 호출

```cpp
// tts_engine.hpp (Before)
class TtsEngine {
private:
  TtsResult synthesize_piper(const std::string& text, const std::string& out_path) const;
  TtsConfig config_;
};

// tts_engine.hpp (After)
#include "tts_cpp/piper_onnx.hpp"

class TtsEngine {
private:
  TtsResult synthesize_piper(const std::string& text, const std::string& out_path) const;
  TtsConfig config_;
  std::unique_ptr<PiperOnnx> piper_onnx_;  // ⭐ 신규 멤버
};
```

```cpp
// tts_engine.cpp (생성자)
TtsEngine::TtsEngine(const TtsConfig& config) : config_(config) {
  // Piper ONNX 엔진 초기화
  if (!config_.piper_model_path.empty()) {
    PiperOnnxConfig piper_cfg;
    piper_cfg.model_path = config_.piper_model_path;
    piper_cfg.data_dir = config_.piper_data_dir;  // ⚠️ 신규 config 필드 필요
    piper_cfg.lexicon_path = config_.piper_lexicon_path;  // 선택적
    piper_cfg.num_threads = 2;

    piper_onnx_ = std::make_unique<PiperOnnx>(piper_cfg);
  }

  // 기존 출력 디렉토리 생성 로직 유지
  std::error_code ec;
  if (!config_.output_dir.empty()) {
    std::filesystem::create_directories(config_.output_dir, ec);
  }
}
```

```cpp
// tts_engine.cpp (synthesize_piper 재구현)
TtsResult TtsEngine::synthesize_piper(const std::string& text, const std::string& out_path) const {
  TtsResult out;
  out.engine = "piper";

  if (!piper_onnx_ || !piper_onnx_->is_ready()) {
    out.error = "piper_onnx_not_initialized";
    return out;
  }

  // 1. sherpa-onnx로 PCM 생성
  PiperOnnxResult onnx_result = piper_onnx_->synthesize(text);
  if (!onnx_result.ok) {
    out.error = "piper_onnx_failed: " + onnx_result.error;
    return out;
  }

  // 2. PCM → WAV 파일 변환 (RIFF 헤더 추가)
  if (!write_pcm_to_wav(out_path, onnx_result.pcm_samples, onnx_result.sample_rate)) {
    out.error = "failed_to_write_wav";
    return out;
  }

  // 3. 결과 반환
  out.ok = true;
  out.audio_path = out_path;
  return out;
}
```

---

#### **STEP 4: WAV 인코더 구현**
**목표**: PCM float → WAV 파일 변환

```cpp
// tts_engine.cpp (Private 메서드)
bool TtsEngine::write_pcm_to_wav(
    const std::string& path,
    const std::vector<float>& samples,
    uint32_t sample_rate) const {

  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) return false;

  const uint16_t channels = 1;
  const uint16_t bits_per_sample = 16;
  const uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8);
  const uint16_t block_align = channels * (bits_per_sample / 8);
  const uint32_t data_size = static_cast<uint32_t>(samples.size() * sizeof(int16_t));
  const uint32_t chunk_size = 36 + data_size;

  // RIFF 헤더 작성
  auto write_str = [&](const char* s) { ofs.write(s, 4); };
  auto write_u16 = [&](uint16_t v) {
    ofs.put(v & 0xFF);
    ofs.put((v >> 8) & 0xFF);
  };
  auto write_u32 = [&](uint32_t v) {
    ofs.put(v & 0xFF);
    ofs.put((v >> 8) & 0xFF);
    ofs.put((v >> 16) & 0xFF);
    ofs.put((v >> 24) & 0xFF);
  };

  write_str("RIFF"); write_u32(chunk_size);
  write_str("WAVE");
  write_str("fmt "); write_u32(16);
  write_u16(1);  // PCM
  write_u16(channels);
  write_u32(sample_rate);
  write_u32(byte_rate);
  write_u16(block_align);
  write_u16(bits_per_sample);
  write_str("data"); write_u32(data_size);

  // PCM 데이터 작성 (float → int16_t 변환)
  for (float sample : samples) {
    float clamped = std::max(-1.0f, std::min(1.0f, sample));
    int16_t pcm = static_cast<int16_t>(clamped * 32767.0f);
    ofs.put(pcm & 0xFF);
    ofs.put((pcm >> 8) & 0xFF);
  }

  return static_cast<bool>(ofs);
}
```

---

#### **STEP 5: CMakeLists.txt 수정**
**목표**: sherpa-onnx TTS 의존성 추가

```cmake
# CMakeLists.txt
find_package(PkgConfig REQUIRED)
pkg_check_modules(SHERPA_ONNX REQUIRED sherpa-onnx)

add_executable(tts_node
  src/tts_node.cpp
  src/tts_engine.cpp
  src/alsa_player.cpp
  src/piper_onnx.cpp  # ⭐ 신규 파일
)

target_include_directories(tts_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHERPA_ONNX_INCLUDE_DIRS}
)

target_link_libraries(tts_node
  ${rclcpp_LIBRARIES}
  ${SHERPA_ONNX_LIBRARIES}
  asound  # ALSA 재생용
)
```

---

#### **STEP 6: config/params.yaml 수정**
**목표**: sherpa-onnx 모델 경로 설정

```yaml
# config/params.yaml (Before)
tts_node:
  ros__parameters:
    piper_model_path: "/home/ubuntu/models/piper/ko_KR-kss-medium.onnx"
    piper_config_path: "/home/ubuntu/models/piper/ko_KR-kss-medium.onnx.json"

# config/params.yaml (After)
tts_node:
  ros__parameters:
    piper_model_path: "/home/ubuntu/models/piper/ko_KR-kss-medium.onnx"
    piper_data_dir: "/home/ubuntu/models/piper/"  # ⭐ 신규: tokens/phonemes
    piper_lexicon_path: ""  # 선택적
```

---

#### **STEP 7: 검증 테스트**
**목표**: 성능 및 음질 확인

```bash
# 1. 빌드
cd ~/dev_ws/AI_secretary_robot
colcon build --packages-select tts_cpp

# 2. 노드 실행
ros2 launch tts_cpp tts.launch.py

# 3. TTS 요청 (별도 터미널)
ros2 topic pub /tts/text std_msgs/String \
  "data: '안녕하세요, 로버입니다.'" --once

# 기대 출력:
# - /tts/audio_path: "/tmp/tts_audio/tts_xxxxx.wav"
# - /tts/engine: "piper"
# - 지연시간: < 100ms

# 4. 음질 확인 (수동)
aplay /tmp/tts_audio/tts_xxxxx.wav
# 기대: 자연스러운 한국어 발음, MOS 4.0/5.0 수준

# 5. 벤치마크 (100회 반복)
for i in {1..100}; do
  ros2 topic pub /tts/text std_msgs/String "data: '테스트'" --once
done
# 평균 지연시간: < 100ms
```

---

## 4. 위험 관리

### 4.1 주요 리스크
| 리스크 | 완화 방안 |
|:---|:---|
| Piper 모델 변환 실패 | sherpa-onnx 공식 Piper 모델 사용 (https://github.com/k2-fsa/sherpa-onnx/releases) |
| 음질 저하 | noise_scale, length_scale 파라미터 튜닝 |
| 메모리 증가 | 모델 lazy loading, 사용 후 리소스 즉시 해제 |
| phoneme dict 누락 | Piper 공식 모델에 포함된 `espeak-ng-data` 사용 |

### 4.2 Piper 모델 변환 가이드
```bash
# sherpa-onnx 호환 Piper 모델 다운로드
cd /home/ubuntu/models/piper/
wget https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-piper-ko_KR-kss-medium.tar.bz2
tar -xvf vits-piper-ko_KR-kss-medium.tar.bz2

# 구조 확인
tree vits-piper-ko_KR-kss-medium/
# ├── ko_KR-kss-medium.onnx
# ├── tokens.txt
# ├── espeak-ng-data/  # phoneme dict
# └── README.md
```

---

## 5. 완료 조건

### 5.1 기능 검증
- ✅ `synthesize_piper()` 음질 동등 이상 (MOS > 4.0)
- ✅ ROS2 토픽 `/tts/audio_path` 정상 발행
- ✅ 지연시간 < 100ms (기존 150~200ms 대비 30% 개선)

### 5.2 코드 품질
- ✅ `TtsEngine` API 인터페이스 불변 (ROS2 토픽 호환성)
- ✅ 메모리 누수 없음 (`valgrind --leak-check=full`)
- ✅ WAV 파일 검증 (`file out.wav` → "RIFF (little-endian) data, WAVE audio")

### 5.3 문서화
- ✅ `src/ai/tts_cpp/README.md` 업데이트 (sherpa-onnx 설치 가이드)
- ✅ `AGENTS.md`에 변경 로그 추가

---

## 6. 참고 자료

### 6.1 공식 문서
- sherpa-onnx TTS C API: https://k2-fsa.github.io/sherpa/onnx/tts/index.html
- Piper TTS: https://github.com/rhasspy/piper
- sherpa-onnx Piper 모델: https://github.com/k2-fsa/sherpa-onnx/releases/tag/tts-models

### 6.2 코드 예제
```bash
# sherpa-onnx TTS 예제
git clone https://github.com/k2-fsa/sherpa-onnx.git
cd sherpa-onnx/c-api-examples
cat offline-tts.c  # VITS TTS 기본 예제
```

---

## 7. 최종 체크리스트

**마이그레이션 전**:
- [ ] sherpa-onnx 라이브러리 설치 (TTS 지원 포함)
- [ ] Piper ONNX 모델 다운로드 및 변환
- [ ] 현재 브랜치 백업 (`git checkout -b backup/tts-before-migration`)

**마이그레이션 중**:
- [ ] `piper_onnx.hpp`, `piper_onnx.cpp` 생성
- [ ] `tts_engine.cpp` 수정 (CLI 호출 → PiperOnnx 클래스)
- [ ] WAV 인코더 구현 (`write_pcm_to_wav()`)
- [ ] `CMakeLists.txt` 의존성 추가
- [ ] `params.yaml` 수정 (`piper_data_dir` 추가)

**마이그레이션 후**:
- [ ] 빌드 성공 (`colcon build --packages-select tts_cpp`)
- [ ] 노드 실행 성공 (`ros2 launch tts_cpp tts.launch.py`)
- [ ] 음질 테스트 통과 (수동 청취 검증)
- [ ] 지연시간 벤치마크 (< 100ms)
- [ ] 메모리 프로파일링 (`ps aux | grep tts_node`)
- [ ] 문서 업데이트 (`README.md`, `AGENTS.md`)

**최종 승인**:
- [ ] 팀장 코드 리뷰 통과
- [ ] 음질 A/B 테스트 통과 (기존 piper CLI vs sherpa-onnx)
- [ ] Planning 문서([docs/plan.md](./plan.md))와 일치 확인

---

**작성자 노트**:
TTS는 STT보다 복잡도가 높습니다. sherpa-onnx는 PCM만 반환하므로 WAV 인코더를 직접 구현해야 합니다. Piper 모델의 sherpa-onnx 호환성은 **필수 사전 검증 사항**입니다. 공식 모델(https://github.com/k2-fsa/sherpa-onnx/releases/tag/tts-models)을 사용하면 안전합니다. 음질 저하 시 `noise_scale`, `length_scale` 파라미터를 조정하세요. CPU 추론이 GPU보다 빠를 수 있으니 벤치마크 후 결정하세요.
