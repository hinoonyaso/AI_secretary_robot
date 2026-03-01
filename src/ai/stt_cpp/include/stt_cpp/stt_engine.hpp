#pragma once

#include "stt_cpp/moonshine_onnx.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace stt_cpp
{

struct SttConfig
{
  bool enabled = true;
  // "auto"=moonshine_onnx→groq, "moonshine_onnx"=local only, "groq"=cloud only
  std::string engine = "auto";

  // Groq Whisper API
  std::string groq_api_key;
  std::string groq_model = "whisper-large-v3";
  std::string groq_language = "ko";
  long groq_timeout_sec = 30;

  // Moonshine ONNX (local)
  std::string onnx_encoder_path;
  std::string onnx_decoder_path;
  bool onnx_use_cuda = true;
  int onnx_gpu_device_id = 0;
  int onnx_intra_threads = 2;
};

struct SttResult
{
  bool ok = false;
  bool used_fallback = false;
  std::string engine;
  std::string text;
  std::string error;
};

class SttEngine
{
public:
  explicit SttEngine(const SttConfig & config);
  ~SttEngine() = default;
  SttResult transcribe(const std::string & wav_path) const;
  SttResult transcribe_pcm(const std::vector<float> & samples, uint32_t sample_rate) const;

private:
  SttResult transcribe_groq(const std::string & wav_path) const;
  SttResult transcribe_moonshine_onnx(const std::string & wav_path) const;
  SttResult transcribe_moonshine_onnx_pcm(
    const std::vector<float> & samples, uint32_t sample_rate) const;
  bool write_temp_wav(
    const std::string & path,
    const std::vector<float> & samples,
    uint32_t sample_rate) const;

  SttConfig config_;
  std::unique_ptr<MoonshineOnnx> moonshine_onnx_;
};

}  // namespace stt_cpp
