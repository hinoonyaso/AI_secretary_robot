#pragma once

#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <cstdint>
#include <memory>
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
  ~MoonshineOnnx() = default;

  MoonshineOnnx(const MoonshineOnnx &) = delete;
  MoonshineOnnx & operator=(const MoonshineOnnx &) = delete;

  std::string transcribe(const std::string & wav_path) const;
  std::string transcribe_pcm(const std::vector<float> & samples, uint32_t sample_rate) const;

  bool is_ready() const { return ready_; }
  const std::string & last_error() const { return last_error_; }

private:
  std::vector<float> load_audio_pcm(const std::string & wav_path) const;

  std::vector<float> run_encoder(
    const std::vector<float> & pcm,
    std::vector<int64_t> & hidden_shape) const;

  std::vector<int64_t> run_decoder_greedy(
    const std::vector<float> & enc_hidden,
    const std::vector<int64_t> & enc_shape) const;

  std::string tokens_to_text(const std::vector<int64_t> & token_ids) const;

  Ort::Env env_;
  Ort::SessionOptions enc_opts_;
  Ort::SessionOptions dec_opts_;
  std::unique_ptr<Ort::Session> encoder_session_;
  std::unique_ptr<Ort::Session> decoder_session_;
  bool ready_ = false;
  mutable std::string last_error_;

  mutable bool vocab_loaded_ = false;
  mutable std::vector<std::string> vocab_id_to_piece_;

  static constexpr int64_t kBosTokenId = 1;
  static constexpr int64_t kEosTokenId = 2;
  static constexpr int64_t kPadTokenId = 0;
  static constexpr int64_t kMaxDecodeSteps = 448;
  static constexpr int kSampleRate = 16000;
};

}  // namespace stt_cpp
