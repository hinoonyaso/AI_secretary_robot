#include "stt_cpp/moonshine_onnx.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#ifdef STT_CPP_USE_SNDFILE
#include <sndfile.h>
#endif

namespace stt_cpp
{
namespace
{

constexpr const char * kDefaultVocabJsonPath = "/home/ubuntu/models/moonshine/vocab.json";

uint16_t read_le_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8U);
}

uint32_t read_le_u32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8U) |
         (static_cast<uint32_t>(p[2]) << 16U) |
         (static_cast<uint32_t>(p[3]) << 24U);
}

std::vector<float> resample_linear(
  const std::vector<float> & input,
  uint32_t input_rate,
  uint32_t output_rate)
{
  if (input.empty() || input_rate == 0U || output_rate == 0U) {
    return {};
  }
  if (input_rate == output_rate) {
    return input;
  }

  const double ratio = static_cast<double>(output_rate) / static_cast<double>(input_rate);
  const size_t out_size = std::max<size_t>(1U, static_cast<size_t>(input.size() * ratio));
  std::vector<float> out(out_size, 0.0F);
  for (size_t i = 0; i < out_size; ++i) {
    const double src_pos = static_cast<double>(i) / ratio;
    const size_t left = static_cast<size_t>(src_pos);
    const size_t right = std::min(left + 1U, input.size() - 1U);
    const double frac = src_pos - static_cast<double>(left);
    const double v = static_cast<double>(input[left]) * (1.0 - frac) +
      static_cast<double>(input[right]) * frac;
    out[i] = std::clamp(static_cast<float>(v), -1.0F, 1.0F);
  }
  return out;
}

}  // namespace

MoonshineOnnx::MoonshineOnnx(const MoonshineConfig & cfg)
{
  last_error_.clear();
  ready_ = false;

  if (cfg.encoder_path.empty() || cfg.decoder_path.empty()) {
    last_error_ = "onnx path is empty";
    return;
  }
  if (!std::filesystem::is_regular_file(cfg.encoder_path)) {
    last_error_ = "encoder onnx not found: " + cfg.encoder_path;
    return;
  }
  if (!std::filesystem::is_regular_file(cfg.decoder_path)) {
    last_error_ = "decoder onnx not found: " + cfg.decoder_path;
    return;
  }

  model_dir_ = std::filesystem::path(cfg.encoder_path).parent_path().string();
  encoder_path_ = cfg.encoder_path;
  decoder_path_ = cfg.decoder_path;
  tokens_path_ = resolve_tokens_path(cfg);
  provider_ = cfg.use_cuda ? "cuda" : "cpu";

  if (tokens_path_.empty()) {
    last_error_ = "vocab json not found";
    return;
  }

  SherpaOnnxOnlineRecognizerConfig config;
  std::memset(&config, 0, sizeof(config));

  config.model_config.transducer.encoder = encoder_path_.c_str();
  config.model_config.transducer.decoder = decoder_path_.c_str();
  config.model_config.transducer.joiner = "";
  config.model_config.tokens = tokens_path_.c_str();
  config.model_config.num_threads = std::max(1, cfg.intra_threads);
  config.model_config.provider = provider_.c_str();
  config.model_config.debug = 0;

  recognizer_ = SherpaOnnxCreateOnlineRecognizer(&config);
  if (recognizer_ == nullptr) {
    last_error_ = "failed to create sherpa-onnx recognizer";
    return;
  }

  ready_ = true;
}

MoonshineOnnx::~MoonshineOnnx()
{
  if (recognizer_ != nullptr) {
    SherpaOnnxDestroyOnlineRecognizer(recognizer_);
    recognizer_ = nullptr;
  }
}

std::string MoonshineOnnx::transcribe(const std::string & wav_path) const
{
  last_error_.clear();

  if (!ready_ || recognizer_ == nullptr) {
    last_error_ = "moonshine onnx is not ready";
    return {};
  }

  const auto pcm = load_audio_pcm(wav_path);
  if (pcm.empty()) {
    if (last_error_.empty()) {
      last_error_ = "empty audio pcm";
    }
    return {};
  }

  return transcribe_pcm(pcm, kSampleRate);
}

std::string MoonshineOnnx::transcribe_pcm(
  const std::vector<float> & samples,
  uint32_t sample_rate) const
{
  last_error_.clear();

  if (!ready_ || recognizer_ == nullptr) {
    last_error_ = "moonshine onnx is not ready";
    return {};
  }
  if (samples.empty()) {
    last_error_ = "empty audio pcm";
    return {};
  }
  if (sample_rate == 0U) {
    last_error_ = "invalid sample rate";
    return {};
  }

  std::vector<float> pcm = resample_linear(
    samples, sample_rate, static_cast<uint32_t>(kSampleRate));
  if (pcm.empty()) {
    last_error_ = "resample failed";
    return {};
  }

  const SherpaOnnxOnlineStream * stream = SherpaOnnxCreateOnlineStream(recognizer_);
  if (stream == nullptr) {
    last_error_ = "failed to create online stream";
    return {};
  }

  SherpaOnnxOnlineStreamAcceptWaveform(
    stream,
    static_cast<int32_t>(kSampleRate),
    pcm.data(),
    static_cast<int32_t>(pcm.size()));
  SherpaOnnxOnlineStreamInputFinished(stream);

  while (SherpaOnnxIsOnlineStreamReady(recognizer_, stream)) {
    SherpaOnnxDecodeOnlineStream(recognizer_, stream);
  }

  const SherpaOnnxOnlineRecognizerResult * result =
    SherpaOnnxGetOnlineStreamResult(recognizer_, stream);

  std::string text;
  if (result != nullptr && result->text != nullptr) {
    text = result->text;
  }

  SherpaOnnxDestroyOnlineRecognizerResult(result);
  SherpaOnnxDestroyOnlineStream(stream);

  if (text.empty()) {
    last_error_ = "empty transcription result";
  }

  return text;
}

std::vector<float> MoonshineOnnx::load_audio_pcm(const std::string & wav_path) const
{
  last_error_.clear();
  if (!std::filesystem::is_regular_file(wav_path)) {
    last_error_ = "wav file not found: " + wav_path;
    return {};
  }

#ifdef STT_CPP_USE_SNDFILE
  SF_INFO sf_info{};
  SNDFILE * snd = sf_open(wav_path.c_str(), SFM_READ, &sf_info);
  if (snd != nullptr) {
    if (sf_info.samplerate != kSampleRate) {
      sf_close(snd);
      last_error_ = "wav sample rate must be 16000";
      return {};
    }
    if (sf_info.channels <= 0) {
      sf_close(snd);
      last_error_ = "wav channel count is invalid";
      return {};
    }

    std::vector<float> interleaved(static_cast<size_t>(sf_info.frames) * sf_info.channels, 0.0F);
    const sf_count_t read_frames = sf_readf_float(snd, interleaved.data(), sf_info.frames);
    sf_close(snd);

    if (read_frames <= 0) {
      last_error_ = "failed to read wav samples (libsndfile)";
      return {};
    }

    std::vector<float> pcm(static_cast<size_t>(read_frames), 0.0F);
    for (sf_count_t i = 0; i < read_frames; ++i) {
      float v = 0.0F;
      for (int ch = 0; ch < sf_info.channels; ++ch) {
        v += interleaved[static_cast<size_t>(i) * sf_info.channels + static_cast<size_t>(ch)];
      }
      v /= static_cast<float>(sf_info.channels);
      pcm[static_cast<size_t>(i)] = std::clamp(v, -1.0F, 1.0F);
    }
    return pcm;
  }
#endif

  std::ifstream ifs(wav_path, std::ios::binary);
  if (!ifs) {
    last_error_ = "failed to open wav file";
    return {};
  }
  const std::vector<uint8_t> bytes(
    (std::istreambuf_iterator<char>(ifs)),
    std::istreambuf_iterator<char>());
  if (bytes.size() < 44) {
    last_error_ = "invalid wav file (too small)";
    return {};
  }
  if (std::memcmp(bytes.data(), "RIFF", 4) != 0 || std::memcmp(bytes.data() + 8, "WAVE", 4) != 0) {
    last_error_ = "invalid wav header (not RIFF/WAVE)";
    return {};
  }

  uint16_t audio_format = 0;
  uint16_t channels = 0;
  uint32_t sample_rate = 0;
  uint16_t bits_per_sample = 0;
  const uint8_t * data_ptr = nullptr;
  uint32_t data_size = 0;

  size_t pos = 12;
  while (pos + 8 <= bytes.size()) {
    const uint8_t * chunk = bytes.data() + pos;
    const uint32_t chunk_size = read_le_u32(chunk + 4);
    const size_t data_start = pos + 8;
    const size_t next_pos = data_start + static_cast<size_t>(chunk_size) + (chunk_size & 1U);
    if (data_start > bytes.size() || next_pos > bytes.size()) {
      break;
    }

    if (std::memcmp(chunk, "fmt ", 4) == 0) {
      if (chunk_size >= 16U) {
        const uint8_t * fmt = bytes.data() + data_start;
        audio_format = read_le_u16(fmt + 0);
        channels = read_le_u16(fmt + 2);
        sample_rate = read_le_u32(fmt + 4);
        bits_per_sample = read_le_u16(fmt + 14);
      }
    } else if (std::memcmp(chunk, "data", 4) == 0) {
      data_ptr = bytes.data() + data_start;
      data_size = chunk_size;
    }

    pos = next_pos;
  }

  if (data_ptr == nullptr || data_size == 0) {
    last_error_ = "wav data chunk not found";
    return {};
  }
  if (sample_rate != static_cast<uint32_t>(kSampleRate)) {
    last_error_ = "wav sample rate must be 16000";
    return {};
  }
  if (channels == 0) {
    last_error_ = "wav channels invalid";
    return {};
  }

  std::vector<float> pcm;
  if (audio_format == 1U && bits_per_sample == 16U) {
    const size_t total_samples = data_size / sizeof(int16_t);
    const size_t frames = total_samples / channels;
    pcm.resize(frames, 0.0F);
    const auto * s16 = reinterpret_cast<const int16_t *>(data_ptr);
    for (size_t i = 0; i < frames; ++i) {
      float v = 0.0F;
      for (uint16_t ch = 0; ch < channels; ++ch) {
        const int16_t raw = s16[i * channels + ch];
        v += static_cast<float>(raw) / 32768.0F;
      }
      pcm[i] = std::clamp(v / static_cast<float>(channels), -1.0F, 1.0F);
    }
  } else if (audio_format == 1U && bits_per_sample == 8U) {
    const size_t total_samples = data_size;
    const size_t frames = total_samples / channels;
    pcm.resize(frames, 0.0F);
    for (size_t i = 0; i < frames; ++i) {
      float v = 0.0F;
      for (uint16_t ch = 0; ch < channels; ++ch) {
        const uint8_t raw = data_ptr[i * channels + ch];
        v += (static_cast<float>(raw) - 128.0F) / 128.0F;
      }
      pcm[i] = std::clamp(v / static_cast<float>(channels), -1.0F, 1.0F);
    }
  } else if (audio_format == 1U && bits_per_sample == 32U) {
    const size_t total_samples = data_size / sizeof(int32_t);
    const size_t frames = total_samples / channels;
    pcm.resize(frames, 0.0F);
    const auto * s32 = reinterpret_cast<const int32_t *>(data_ptr);
    for (size_t i = 0; i < frames; ++i) {
      float v = 0.0F;
      for (uint16_t ch = 0; ch < channels; ++ch) {
        const int32_t raw = s32[i * channels + ch];
        v += static_cast<float>(raw / 2147483648.0);
      }
      pcm[i] = std::clamp(v / static_cast<float>(channels), -1.0F, 1.0F);
    }
  } else if (audio_format == 3U && bits_per_sample == 32U) {
    const size_t total_samples = data_size / sizeof(float);
    const size_t frames = total_samples / channels;
    pcm.resize(frames, 0.0F);
    const auto * f32 = reinterpret_cast<const float *>(data_ptr);
    for (size_t i = 0; i < frames; ++i) {
      float v = 0.0F;
      for (uint16_t ch = 0; ch < channels; ++ch) {
        v += f32[i * channels + ch];
      }
      pcm[i] = std::clamp(v / static_cast<float>(channels), -1.0F, 1.0F);
    }
  } else {
    std::ostringstream oss;
    oss << "unsupported wav format (audio_format=" << audio_format
        << ", bits=" << bits_per_sample << ")";
    last_error_ = oss.str();
    return {};
  }

  return pcm;
}

std::string MoonshineOnnx::resolve_tokens_path(const MoonshineConfig & cfg) const
{
  if (std::filesystem::is_regular_file(kDefaultVocabJsonPath)) {
    return kDefaultVocabJsonPath;
  }

  const auto decoder_dir = std::filesystem::path(cfg.decoder_path).parent_path();
  const auto encoder_dir = std::filesystem::path(cfg.encoder_path).parent_path();

  const auto decoder_vocab = decoder_dir / "vocab.json";
  if (std::filesystem::is_regular_file(decoder_vocab)) {
    return decoder_vocab.string();
  }

  const auto encoder_vocab = encoder_dir / "vocab.json";
  if (std::filesystem::is_regular_file(encoder_vocab)) {
    return encoder_vocab.string();
  }

  return {};
}

}  // namespace stt_cpp
