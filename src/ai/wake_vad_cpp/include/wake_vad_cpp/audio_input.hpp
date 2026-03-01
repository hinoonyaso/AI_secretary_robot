#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <portaudio.h>

namespace wake_vad_cpp {

class AudioInput {
public:
  using Frame = std::vector<int16_t>;
  using FrameCallback = std::function<void(const Frame& frame)>;

  AudioInput();
  ~AudioInput();

  bool configure(int device_index, int sample_rate, int channels, int frame_length);
  void set_preferred_device_keywords(const std::vector<std::string> & keywords);

  bool start();
  void stop();
  bool is_running() const;
  int selected_device_index() const;
  std::string selected_device_name() const;
  std::string last_error() const;

  void set_callback(FrameCallback cb);

private:
  static int pa_callback(const void* input,
                         void* output,
                         unsigned long frameCount,
                         const PaStreamCallbackTimeInfo* timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void* userData);

  bool validate_config_or_fix_() const;
  bool supports_16k_mono_int16_(int device_index) const;
  int  resolve_device_index_() const;

private:
  int device_index_;
  int sample_rate_;
  int channels_;
  int frame_length_;
  std::vector<std::string> preferred_device_keywords_;
  int selected_device_index_;
  std::string selected_device_name_;
  std::string last_error_;

  std::atomic<bool> running_;
  std::atomic<bool> initialized_;

  PaStream* stream_;

  std::mutex cb_mutex_;
  FrameCallback callback_;

  // PortAudio callback 스레드에서 사용하는 고정 길이 프레임 버퍼
  Frame frame_buf_;
};

}  // namespace wake_vad_cpp
