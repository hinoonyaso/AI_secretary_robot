#pragma once

#include "tts_cpp/piper_onnx.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tts_cpp
{

struct TtsConfig
{
  bool enabled = true;
  std::string output_dir = "/tmp/tts_audio";
  // "auto"=edge→piper→espeak, "piper"=piper only, "espeak"=espeak only
  std::string engine = "auto";

  std::string edge_script_path;
  std::string edge_voice = "ko-KR-SunHiNeural";
  std::string edge_rate = "+0%";
  std::string edge_volume = "+0%";

  std::string piper_executable = "piper";
  std::string piper_model_path;
  std::string piper_config_path;
  std::string piper_data_dir;
  std::string piper_lexicon_path;
  std::string piper_tokens_path;
  std::string piper_speaker = "";

  std::string espeak_executable = "espeak-ng";
  std::string espeak_voice = "ko";
};

struct TtsResult
{
  bool ok = false;
  bool used_fallback = false;
  std::string engine;
  std::string audio_path;
  std::vector<int16_t> pcm_samples;
  uint32_t pcm_sample_rate = 0;
  uint16_t pcm_channels = 1;
  std::string error;

  bool has_pcm() const { return !pcm_samples.empty(); }
};

class TtsEngine
{
public:
  explicit TtsEngine(const TtsConfig & config);
  TtsResult synthesize(const std::string & text) const;

private:
  TtsResult synthesize_edge(const std::string & text, const std::string & out_path) const;
  TtsResult synthesize_piper(const std::string & text, const std::string & out_path) const;
  TtsResult synthesize_espeak(const std::string & text, const std::string & out_path) const;
  std::string make_output_path(const std::string & extension) const;
  bool write_pcm_to_wav(
    const std::string & path,
    const std::vector<int16_t> & samples,
    uint32_t sample_rate) const;
  static int16_t float_to_pcm16(float sample);

  TtsConfig config_;
  std::unique_ptr<PiperOnnx> piper_onnx_;
};

}  // namespace tts_cpp
