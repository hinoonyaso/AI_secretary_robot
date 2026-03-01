#pragma once

#include <sherpa-onnx/c-api/c-api.h>

#include <cstdint>
#include <string>
#include <vector>

namespace tts_cpp
{

struct PiperOnnxConfig
{
  std::string model_path;
  std::string data_dir;
  std::string tokens_path;
  std::string lexicon_path;
  int num_threads = 2;
  bool debug = false;
};

struct PiperOnnxResult
{
  bool ok = false;
  std::vector<float> pcm_samples;
  uint32_t sample_rate = 0;
  std::string error;
};

class PiperOnnx
{
public:
  explicit PiperOnnx(const PiperOnnxConfig & cfg);
  ~PiperOnnx();

  PiperOnnx(const PiperOnnx &) = delete;
  PiperOnnx & operator=(const PiperOnnx &) = delete;

  PiperOnnxResult synthesize(const std::string & text) const;
  bool is_ready() const { return ready_; }
  const std::string & last_error() const { return last_error_; }

private:
  const SherpaOnnxOfflineTts * tts_ = nullptr;
  PiperOnnxConfig config_;
  bool ready_ = false;
  mutable std::string last_error_;
};

}  // namespace tts_cpp
