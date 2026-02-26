#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tts_cpp
{

struct TtsConfig
{
  bool enabled = true;
  std::string output_dir = "/tmp/tts_audio";
  // "auto"=edge→melo→espeak, "melo"=melo→espeak, "espeak"=espeak only
  std::string engine = "auto";

  std::string edge_script_path;
  std::string edge_voice = "ko-KR-SunHiNeural";
  std::string edge_rate = "+0%";
  std::string edge_volume = "+0%";

  std::string melo_script_path;
  std::string melo_language = "KR";
  std::string melo_speaker = "";
  std::string melo_speed = "1.0";
  std::string melo_device = "auto";
  std::string melo_server_url = "http://127.0.0.1:5500/synthesize";
  long melo_server_timeout_sec = 15;

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
  TtsResult synthesize_melo(const std::string & text, const std::string & out_path) const;
  TtsResult synthesize_espeak(const std::string & text, const std::string & out_path) const;
  std::string make_output_path(const std::string & extension) const;

  TtsConfig config_;
};

}  // namespace tts_cpp
