#include "wake_vad_cpp/audio_input.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <utility>

using namespace std;


namespace wake_vad_cpp {

namespace {
string to_lower(string s)
{
  transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return static_cast<char>(tolower(c)); });
  return s;
}
}  // namespace

AudioInput::AudioInput()
: device_index_(-1),
  sample_rate_(16000),
  channels_(1),
  frame_length_(512),
  running_(false),
  initialized_(false),
  stream_(nullptr),
  frame_buf_(512, 0)
{
}

AudioInput::~AudioInput()
{
  stop();
}

bool AudioInput::configure(int device_index, int sample_rate, int channels, int frame_length)
{
  if (is_running()) return false;  // 실행 중 설정 변경 금지(안전)

  device_index_  = device_index;
  sample_rate_   = sample_rate;
  channels_      = channels;
  frame_length_  = frame_length;

  return validate_config_or_fix_();
}

bool AudioInput::validate_config_or_fix_() const
{
  // wake_vad 파이프라인 요구 규격 고정
  if (sample_rate_ != 16000) return false;
  if (channels_ != 1) return false;
  if (frame_length_ != 512) return false;
  return true;
}

bool AudioInput::supports_16k_mono_int16_(int device_index) const
{
  const PaDeviceInfo* info = Pa_GetDeviceInfo(device_index);
  if (!info || info->maxInputChannels < 1) return false;

  PaStreamParameters in_params{};
  in_params.device = device_index;
  in_params.channelCount = 1;
  in_params.sampleFormat = paInt16;
  in_params.suggestedLatency = info->defaultLowInputLatency;
  in_params.hostApiSpecificStreamInfo = nullptr;

  return Pa_IsFormatSupported(&in_params, nullptr, 16000.0) == paFormatIsSupported;
}

/// 오디오 입력 디바이스 자동 선택 우선순위:
/// 1) 파라미터로 지정된 인덱스  2) XFM-DP 마이크 어레이
/// 3) USB 마이크  4) 시스템 기본 장치  5) 16kHz mono 지원하는 첫 장치
int AudioInput::resolve_device_index_() const
{
  const int device_count = Pa_GetDeviceCount();
  if (device_count <= 0) return paNoDevice;

  if (device_index_ >= 0 &&
      device_index_ < device_count &&
      supports_16k_mono_int16_(device_index_)) {
    return device_index_;
  }

  int usb_candidate = paNoDevice;
  int first_supported = paNoDevice;

  for (int i = 0; i < device_count; ++i) {
    if (!supports_16k_mono_int16_(i)) continue;
    if (first_supported == paNoDevice) first_supported = i;

    const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
    const string name = to_lower(info && info->name ? info->name : "");
    if (name.find("xfm-dp") != string::npos ||
        name.find("xfmdp") != string::npos ||
        name.find("xfm dp") != string::npos) {
      return i;
    }
    if (usb_candidate == paNoDevice && name.find("usb") != string::npos) {
      usb_candidate = i;
    }
  }

  if (usb_candidate != paNoDevice) return usb_candidate;

  const int default_device = Pa_GetDefaultInputDevice();
  if (default_device != paNoDevice && supports_16k_mono_int16_(default_device)) {
    return default_device;
  }

  return first_supported;
}

void AudioInput::set_callback(FrameCallback cb)
{
  lock_guard<mutex> lock(cb_mutex_);
  callback_ = move(cb);
}

/// PortAudio 초기화 → 디바이스 선택 → 16kHz mono int16 스트림 오픈 → 콜백 루프 시작
bool AudioInput::start()
{
  if (running_) return true;

  if (!validate_config_or_fix_()) return false;

  PaError err;

  if (!initialized_) {
    err = Pa_Initialize();
    if (err != paNoError) return false;
    initialized_ = true;
  }

  const int dev = resolve_device_index_();
  if (dev == paNoDevice) return false;

  const PaDeviceInfo* devInfo = Pa_GetDeviceInfo(dev);
  if (!devInfo) return false;
  if (devInfo->maxInputChannels < 1) return false;

  PaStreamParameters inParams;
  inParams.device = dev;
  inParams.channelCount = 1;               // mono
  inParams.sampleFormat = paInt16;         // int16
  inParams.suggestedLatency = devInfo->defaultLowInputLatency;
  inParams.hostApiSpecificStreamInfo = nullptr;

  if (stream_) {
    Pa_CloseStream(stream_);
    stream_ = nullptr;
  }

  frame_buf_.assign(static_cast<size_t>(frame_length_), 0);

  err = Pa_OpenStream(
    &stream_,
    &inParams,
    nullptr,                 // output 없음
    sample_rate_,            // 16000
    static_cast<unsigned long>(frame_length_), // 512
    paNoFlag,
    &AudioInput::pa_callback,
    this
  );

  if (err != paNoError) {
    stream_ = nullptr;
    return false;
  }

  err = Pa_StartStream(stream_);
  if (err != paNoError) {
    Pa_CloseStream(stream_);
    stream_ = nullptr;
    return false;
  }

  running_ = true;
  return true;
}

void AudioInput::stop()
{
  running_ = false;

  if (stream_) {
    if (Pa_IsStreamActive(stream_) == 1) {
      Pa_StopStream(stream_);
    }
    Pa_CloseStream(stream_);
    stream_ = nullptr;
  }

  if (initialized_) {
    Pa_Terminate();
    initialized_ = false;
  }
}

bool AudioInput::is_running() const
{
  return running_.load();
}

/// PortAudio 콜백 (오디오 스레드): PCM 프레임을 frame_buf_에 복사 후 등록된 콜백 호출
int AudioInput::pa_callback(const void* input,
                            void* /*output*/,
                            unsigned long frameCount,
                            const PaStreamCallbackTimeInfo* /*timeInfo*/,
                            PaStreamCallbackFlags /*statusFlags*/,
                            void* userData)
{
  auto* self = static_cast<AudioInput*>(userData);
  if (!self || !self->running_.load()) return paComplete;

  const auto* in = static_cast<const int16_t*>(input);
  if (!in) {
    fill(self->frame_buf_.begin(), self->frame_buf_.end(), 0);
  } else {
    const unsigned long n =
      min<unsigned long>(frameCount, static_cast<unsigned long>(self->frame_buf_.size()));
    copy(in, in + n, self->frame_buf_.begin());
    if (n < self->frame_buf_.size()) {
      fill(self->frame_buf_.begin() + n, self->frame_buf_.end(), 0);
    }
  }

  AudioInput::FrameCallback cb;
  {
    lock_guard<mutex> lock(self->cb_mutex_);
    cb = self->callback_;
  }
  if (cb) {
    cb(self->frame_buf_);
  }

  return paContinue;
}

}  // namespace wake_vad_cpp
