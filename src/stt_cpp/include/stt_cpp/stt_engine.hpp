#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace stt_cpp
{

struct SttConfig
{
  bool enabled = true;
  std::string groq_api_key;
  std::string groq_model = "whisper-large-v3";
  std::string groq_language = "ko";
  long groq_timeout_sec = 30;
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
  bool write_temp_wav(
    const std::string & path,
    const std::vector<float> & samples,
    uint32_t sample_rate) const;

  SttConfig config_;
};

}  // namespace stt_cpp
