#include "wake_vad_cpp/vad_engine.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <vector>

#include "onnxruntime_cxx_api.h"

using namespace std;


namespace wake_vad_cpp
{

/// Silero VAD ONNX 모델의 내부 상태 (RNN hidden state + context 버퍼)
struct VadEngine::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "wake_vad_cpp_vad"};
  Ort::SessionOptions session_options;
  unique_ptr<Ort::Session> session;
  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

  array<int64_t, 2> input_dims{1, 576};
  array<int64_t, 3> state_dims{2, 1, 128};
  array<int64_t, 1> sr_dims{1};
  array<int64_t, 1> sr_values{16000};

  vector<float> state = vector<float>(2 * 1 * 128, 0.0F);
  vector<float> context = vector<float>(64, 0.0F);
  vector<float> input_buffer = vector<float>(576, 0.0F);

  array<const char *, 3> input_names{"input", "state", "sr"};
  array<const char *, 2> output_names{"output", "stateN"};
};

VadEngine::VadEngine()
: impl_(make_unique<Impl>()), threshold_(0.5F), initialized_(false)
{
}

VadEngine::~VadEngine() = default;

bool VadEngine::initialize(float threshold, const string & model_path)
{
  threshold_ = threshold;
  initialized_ = false;

  if (model_path.empty() || !filesystem::is_regular_file(model_path)) {
    cerr << "[wake_vad_cpp] invalid vad_model_path: " << model_path << endl;
    return false;
  }

  impl_->session_options.SetIntraOpNumThreads(1);
  impl_->session_options.SetInterOpNumThreads(1);
  impl_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  try {
    impl_->session = make_unique<Ort::Session>(
      impl_->env, model_path.c_str(), impl_->session_options);
    reset();
    initialized_ = true;
    return true;
  } catch (const Ort::Exception & e) {
    cerr << "[wake_vad_cpp] onnxruntime init failed: " << e.what() << endl;
    impl_->session.reset();
    initialized_ = false;
    return false;
  }
}

/// 512 샘플 PCM 프레임에 대해 음성 확률을 추론하여 threshold 이상이면 true
/// 입력 구성: [이전 context 64샘플 | 현재 PCM 512샘플] = 576샘플 → ONNX 추론
bool VadEngine::is_speech(const vector<int16_t> & pcm_frame)
{
  if (!initialized_ || !impl_->session) {
    return false;
  }
  if (pcm_frame.size() != 512) {
    return false;
  }

  // input_buffer = [context(64) + 정규화된 PCM(512)] 조립
  for (size_t i = 0; i < impl_->context.size(); ++i) {
    impl_->input_buffer[i] = impl_->context[i];
  }
  for (size_t i = 0; i < pcm_frame.size(); ++i) {
    impl_->input_buffer[i + impl_->context.size()] = static_cast<float>(pcm_frame[i]) / 32768.0F;
  }

  array<Ort::Value, 3> inputs = {
    Ort::Value::CreateTensor<float>(
      impl_->memory_info, impl_->input_buffer.data(), impl_->input_buffer.size(),
      impl_->input_dims.data(), impl_->input_dims.size()),
    Ort::Value::CreateTensor<float>(
      impl_->memory_info, impl_->state.data(), impl_->state.size(),
      impl_->state_dims.data(), impl_->state_dims.size()),
    Ort::Value::CreateTensor<int64_t>(
      impl_->memory_info, impl_->sr_values.data(), impl_->sr_values.size(),
      impl_->sr_dims.data(), impl_->sr_dims.size())
  };

  try {
    auto outputs = impl_->session->Run(
      Ort::RunOptions{nullptr},
      impl_->input_names.data(),
      inputs.data(),
      inputs.size(),
      impl_->output_names.data(),
      impl_->output_names.size());

    const float prob = outputs[0].GetTensorMutableData<float>()[0];
    // RNN hidden state 갱신 + 다음 프레임용 context(마지막 64샘플) 보존
    float * state_out = outputs[1].GetTensorMutableData<float>();
    memcpy(impl_->state.data(), state_out, impl_->state.size() * sizeof(float));
    copy(impl_->input_buffer.end() - 64, impl_->input_buffer.end(), impl_->context.begin());
    return prob >= threshold_;
  } catch (const Ort::Exception & e) {
    cerr << "[wake_vad_cpp] onnxruntime inference failed: " << e.what() << endl;
    return false;
  }
}

void VadEngine::reset()
{
  fill(impl_->state.begin(), impl_->state.end(), 0.0F);
  fill(impl_->context.begin(), impl_->context.end(), 0.0F);
}

bool VadEngine::initialized() const
{
  return initialized_;
}

}  // namespace wake_vad_cpp

