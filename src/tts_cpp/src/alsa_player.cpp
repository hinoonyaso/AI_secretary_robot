#include "tts_cpp/alsa_player.hpp"

namespace tts_cpp
{

AlsaPlayer::AlsaPlayer(const std::string & device)
: device_(device)
{
#ifdef HAVE_ALSA
  available_ = true;
#else
  available_ = false;
  last_error_ = "alsa_not_compiled";
#endif
}

AlsaPlayer::~AlsaPlayer()
{
#ifdef HAVE_ALSA
  close_device();
#endif
}

bool AlsaPlayer::play_pcm(
  const std::vector<int16_t> & samples,
  uint32_t sample_rate,
  uint16_t channels)
{
#ifndef HAVE_ALSA
  (void)samples;
  (void)sample_rate;
  (void)channels;
  last_error_ = "alsa_not_compiled";
  return false;
#else
  last_error_.clear();
  if (!available_) {
    last_error_ = "alsa_unavailable";
    return false;
  }
  if (samples.empty()) {
    last_error_ = "empty_samples";
    return false;
  }
  if (!open_device(sample_rate, channels)) {
    return false;
  }

  const snd_pcm_uframes_t total_frames = samples.size() / channels;
  snd_pcm_uframes_t written_frames = 0;
  while (written_frames < total_frames) {
    const int16_t * ptr = samples.data() + static_cast<size_t>(written_frames * channels);
    const snd_pcm_uframes_t remaining = total_frames - written_frames;
    const snd_pcm_sframes_t rc = snd_pcm_writei(pcm_handle_, ptr, remaining);
    if (rc >= 0) {
      written_frames += static_cast<snd_pcm_uframes_t>(rc);
      continue;
    }

    const int rec = snd_pcm_recover(pcm_handle_, static_cast<int>(rc), 1);
    if (rec < 0) {
      last_error_ = snd_strerror(rec);
      close_device();
      return false;
    }
  }

  const int drain_rc = snd_pcm_drain(pcm_handle_);
  if (drain_rc < 0) {
    last_error_ = snd_strerror(drain_rc);
    close_device();
    return false;
  }

  close_device();
  return true;
#endif
}

#ifdef HAVE_ALSA
bool AlsaPlayer::open_device(uint32_t sample_rate, uint16_t channels)
{
  close_device();

  const int open_rc = snd_pcm_open(&pcm_handle_, device_.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
  if (open_rc < 0) {
    last_error_ = snd_strerror(open_rc);
    pcm_handle_ = nullptr;
    return false;
  }

  const int set_rc = snd_pcm_set_params(
    pcm_handle_,
    SND_PCM_FORMAT_S16_LE,
    SND_PCM_ACCESS_RW_INTERLEAVED,
    channels,
    sample_rate,
    1,
    500000);
  if (set_rc < 0) {
    last_error_ = snd_strerror(set_rc);
    close_device();
    return false;
  }

  return true;
}

void AlsaPlayer::close_device()
{
  if (pcm_handle_ != nullptr) {
    snd_pcm_close(pcm_handle_);
    pcm_handle_ = nullptr;
  }
}
#endif

}  // namespace tts_cpp
