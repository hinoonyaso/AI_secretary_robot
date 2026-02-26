#pragma once

#include <cstdint>
#include <string>
#include <vector>

#ifdef HAVE_ALSA
#include <alsa/asoundlib.h>
#endif

namespace tts_cpp
{

class AlsaPlayer
{
public:
  explicit AlsaPlayer(const std::string & device = "default");
  ~AlsaPlayer();

  bool play_pcm(
    const std::vector<int16_t> & samples,
    uint32_t sample_rate,
    uint16_t channels = 1);

  bool is_available() const { return available_; }
  const std::string & last_error() const { return last_error_; }

private:
#ifdef HAVE_ALSA
  bool open_device(uint32_t sample_rate, uint16_t channels);
  void close_device();
  snd_pcm_t * pcm_handle_ = nullptr;
#endif
  std::string device_;
  bool available_ = false;
  mutable std::string last_error_;
};

}  // namespace tts_cpp
