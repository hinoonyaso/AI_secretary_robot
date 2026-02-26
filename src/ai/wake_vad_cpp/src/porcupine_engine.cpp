#include "wake_vad_cpp/porcupine_engine.hpp"

#include <filesystem>
#include <iostream>

#include "picovoice.h"
#include "pv_porcupine.h"

using namespace std;


namespace wake_vad_cpp
{

struct PorcupineEngine::Impl
{
  pv_porcupine_t * handle{nullptr};
};

PorcupineEngine::PorcupineEngine()
: impl_(make_unique<Impl>()), initialized_(false), frame_length_(512)
{
}

PorcupineEngine::~PorcupineEngine()
{
  shutdown();
}

bool PorcupineEngine::initialize(
  const string & access_key,
  const string & keyword_path,
  const string & model_path,
  float sensitivity)
{
  /// 키/모델/키워드 파일 유효성 검증 후 Porcupine 핸들을 생성
  shutdown();

  if (access_key.empty()) {
    cerr << "[wake_vad_cpp] porcupine access_key is empty" << endl;
    return false;
  }
  if (keyword_path.empty() || !filesystem::is_regular_file(keyword_path)) {
    cerr << "[wake_vad_cpp] invalid keyword_path: " << keyword_path << endl;
    return false;
  }
  if (model_path.empty() || !filesystem::is_regular_file(model_path)) {
    cerr << "[wake_vad_cpp] invalid porcupine model_path: " << model_path << endl;
    return false;
  }

  const char * keyword_paths[1] = {keyword_path.c_str()};
  const float sensitivities[1] = {sensitivity};

  pv_status_t status = pv_porcupine_init(
    access_key.c_str(),
    model_path.c_str(),
    "best",
    1,
    keyword_paths,
    sensitivities,
    &impl_->handle);

  if (status != PV_STATUS_SUCCESS) {
    cerr << "[wake_vad_cpp] pv_porcupine_init failed: "
              << pv_status_to_string(status) << endl;
    impl_->handle = nullptr;
    return false;
  }

  frame_length_ = pv_porcupine_frame_length();
  initialized_ = true;
  return true;
}

bool PorcupineEngine::process(const vector<int16_t> & pcm_frame)
{
  /// Porcupine 고정 프레임 길이(엔진 제공) 단위로만 호출하며, 감지 시 true 반환
  if (!initialized_ || impl_->handle == nullptr) {
    return false;
  }
  if (static_cast<int>(pcm_frame.size()) != frame_length_) {
    return false;
  }

  int32_t keyword_index = -1;
  pv_status_t status = pv_porcupine_process(impl_->handle, pcm_frame.data(), &keyword_index);
  if (status != PV_STATUS_SUCCESS) {
    cerr << "[wake_vad_cpp] pv_porcupine_process failed: "
              << pv_status_to_string(status) << endl;
    return false;
  }

  return keyword_index >= 0;
}

void PorcupineEngine::shutdown()
{
  /// 재초기화/종료 시 네이티브 핸들을 안전하게 해제하고 상태를 초기값으로 복원
  if (impl_ && impl_->handle) {
    pv_porcupine_delete(impl_->handle);
    impl_->handle = nullptr;
  }
  initialized_ = false;
  frame_length_ = 512;
}

bool PorcupineEngine::initialized() const
{
  return initialized_;
}

int PorcupineEngine::frame_length() const
{
  return frame_length_;
}

}  // namespace wake_vad_cpp
