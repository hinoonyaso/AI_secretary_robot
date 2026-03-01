#pragma once

#include <sherpa-onnx/c-api/c-api.h>

#include <cstdint>
#include <string>
#include <vector>

namespace stt_cpp
{

struct MoonshineConfig
{
  std::string encoder_path;
  std::string decoder_path;
  bool use_cuda = true;
  int gpu_device_id = 0;
  int intra_threads = 2;
};

class MoonshineOnnx
{
public:
  explicit MoonshineOnnx(const MoonshineConfig & cfg);
  ~MoonshineOnnx();

  MoonshineOnnx(const MoonshineOnnx &) = delete;
  MoonshineOnnx & operator=(const MoonshineOnnx &) = delete;

  std::string transcribe(const std::string & wav_path) const;
  std::string transcribe_pcm(const std::vector<float> & samples, uint32_t sample_rate) const;

  bool is_ready() const { return ready_; }
  const std::string & last_error() const { return last_error_; }

private:
  std::vector<float> load_audio_pcm(const std::string & wav_path) const;
  std::string resolve_tokens_path(const MoonshineConfig & cfg) const;
  const SherpaOnnxOnlineRecognizer * recognizer_ = nullptr;
  std::string model_dir_;
  std::string encoder_path_;
  std::string decoder_path_;
  std::string tokens_path_;
  std::string provider_;
  bool ready_ = false;
  mutable std::string last_error_;
  static constexpr int kSampleRate = 16000;
};

}  // namespace stt_cpp
