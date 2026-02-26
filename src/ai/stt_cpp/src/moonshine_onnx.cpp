#include "stt_cpp/moonshine_onnx.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#ifdef STT_CPP_USE_SNDFILE
#include <sndfile.h>
#endif

namespace stt_cpp
{
namespace
{

constexpr const char * kVocabJsonPath = "/home/ubuntu/models/moonshine/vocab.json";
constexpr const char * kSpmSpaceMarker = "\xE2\x96\x81";  // U+2581

std::string trim_copy(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

void append_utf8_from_codepoint(uint32_t cp, std::string & out)
{
  if (cp <= 0x7FU) {
    out.push_back(static_cast<char>(cp));
  } else if (cp <= 0x7FFU) {
    out.push_back(static_cast<char>(0xC0U | (cp >> 6U)));
    out.push_back(static_cast<char>(0x80U | (cp & 0x3FU)));
  } else if (cp <= 0xFFFFU) {
    out.push_back(static_cast<char>(0xE0U | (cp >> 12U)));
    out.push_back(static_cast<char>(0x80U | ((cp >> 6U) & 0x3FU)));
    out.push_back(static_cast<char>(0x80U | (cp & 0x3FU)));
  } else {
    out.push_back(static_cast<char>(0xF0U | (cp >> 18U)));
    out.push_back(static_cast<char>(0x80U | ((cp >> 12U) & 0x3FU)));
    out.push_back(static_cast<char>(0x80U | ((cp >> 6U) & 0x3FU)));
    out.push_back(static_cast<char>(0x80U | (cp & 0x3FU)));
  }
}

int hex_to_int(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return 10 + (c - 'a');
  }
  if (c >= 'A' && c <= 'F') {
    return 10 + (c - 'A');
  }
  return -1;
}

std::string json_unescape(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] != '\\') {
      out.push_back(s[i]);
      continue;
    }
    if (i + 1 >= s.size()) {
      break;
    }
    const char esc = s[++i];
    switch (esc) {
      case '"':
      case '\\':
      case '/':
        out.push_back(esc);
        break;
      case 'b':
        out.push_back('\b');
        break;
      case 'f':
        out.push_back('\f');
        break;
      case 'n':
        out.push_back('\n');
        break;
      case 'r':
        out.push_back('\r');
        break;
      case 't':
        out.push_back('\t');
        break;
      case 'u': {
        if (i + 4 >= s.size()) {
          break;
        }
        int d0 = hex_to_int(s[i + 1]);
        int d1 = hex_to_int(s[i + 2]);
        int d2 = hex_to_int(s[i + 3]);
        int d3 = hex_to_int(s[i + 4]);
        if (d0 < 0 || d1 < 0 || d2 < 0 || d3 < 0) {
          i += 4;
          break;
        }
        uint32_t cp = static_cast<uint32_t>((d0 << 12) | (d1 << 8) | (d2 << 4) | d3);
        append_utf8_from_codepoint(cp, out);
        i += 4;
        break;
      }
      default:
        out.push_back(esc);
        break;
    }
  }
  return out;
}

void replace_all(std::string & s, const std::string & from, const std::string & to)
{
  if (from.empty()) {
    return;
  }
  size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.size(), to);
    pos += to.size();
  }
}

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
: env_(ORT_LOGGING_LEVEL_WARNING, "moonshine")
{
  auto logger = rclcpp::get_logger("moonshine_onnx");
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

  enc_opts_.SetIntraOpNumThreads(std::max(1, cfg.intra_threads));
  dec_opts_.SetIntraOpNumThreads(std::max(1, cfg.intra_threads));
  enc_opts_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  dec_opts_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  if (cfg.use_cuda) {
    try {
      OrtCUDAProviderOptions cuda_options{};
      cuda_options.device_id = cfg.gpu_device_id;
      enc_opts_.AppendExecutionProvider_CUDA(cuda_options);
      dec_opts_.AppendExecutionProvider_CUDA(cuda_options);
    } catch (const Ort::Exception & e) {
      RCLCPP_WARN(
        logger,
        "CUDA EP unavailable, using CPU: %s",
        e.what());
    }
  }

  try {
    encoder_session_ = std::make_unique<Ort::Session>(env_, cfg.encoder_path.c_str(), enc_opts_);
    decoder_session_ = std::make_unique<Ort::Session>(env_, cfg.decoder_path.c_str(), dec_opts_);
    ready_ = true;
  } catch (const Ort::Exception & e) {
    last_error_ = std::string("onnx session init failed: ") + e.what();
    encoder_session_.reset();
    decoder_session_.reset();
    ready_ = false;
  }
}

std::string MoonshineOnnx::transcribe(const std::string & wav_path) const
{
  last_error_.clear();

  if (!ready_ || !encoder_session_ || !decoder_session_) {
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

  if (!ready_ || !encoder_session_ || !decoder_session_) {
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

  std::vector<int64_t> enc_shape;
  const auto enc_hidden = run_encoder(pcm, enc_shape);
  if (enc_hidden.empty()) {
    if (last_error_.empty()) {
      last_error_ = "encoder output is empty";
    }
    return {};
  }

  const auto token_ids = run_decoder_greedy(enc_hidden, enc_shape);
  if (token_ids.empty()) {
    if (last_error_.empty()) {
      last_error_ = "decoder produced no tokens";
    }
    return {};
  }

  return tokens_to_text(token_ids);
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

std::vector<float> MoonshineOnnx::run_encoder(
  const std::vector<float> & pcm,
  std::vector<int64_t> & hidden_shape) const
{
  last_error_.clear();
  hidden_shape.clear();

  if (!encoder_session_) {
    last_error_ = "encoder session is null";
    return {};
  }
  if (pcm.empty()) {
    last_error_ = "encoder input pcm is empty";
    return {};
  }

  std::array<int64_t, 2> input_shape{
    1,
    static_cast<int64_t>(pcm.size())
  };

  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    mem_info,
    const_cast<float *>(pcm.data()),
    pcm.size(),
    input_shape.data(),
    input_shape.size());

  const std::array<const char *, 1> input_names{"input_values"};
  const std::array<const char *, 1> output_names{"encoder_hidden_states"};

  try {
    auto outputs = encoder_session_->Run(
      Ort::RunOptions{nullptr},
      input_names.data(),
      &input_tensor,
      1,
      output_names.data(),
      1);

    if (outputs.empty() || !outputs[0].IsTensor()) {
      last_error_ = "encoder output is not a tensor";
      return {};
    }

    auto type_info = outputs[0].GetTensorTypeAndShapeInfo();
    hidden_shape = type_info.GetShape();
    size_t total = 1;
    for (int64_t d : hidden_shape) {
      if (d <= 0) {
        last_error_ = "encoder output has invalid dynamic shape";
        return {};
      }
      total *= static_cast<size_t>(d);
    }

    const float * ptr = outputs[0].GetTensorData<float>();
    return std::vector<float>(ptr, ptr + total);
  } catch (const Ort::Exception & e) {
    last_error_ = std::string("encoder inference failed: ") + e.what();
    return {};
  }
}

std::vector<int64_t> MoonshineOnnx::run_decoder_greedy(
  const std::vector<float> & enc_hidden,
  const std::vector<int64_t> & enc_shape) const
{
  last_error_.clear();

  if (!decoder_session_) {
    last_error_ = "decoder session is null";
    return {};
  }
  if (enc_hidden.empty() || enc_shape.size() != 3) {
    last_error_ = "encoder hidden state shape must be [1, seq_len, hidden_dim]";
    return {};
  }

  const std::array<const char *, 2> input_names{"input_ids", "encoder_hidden_states"};
  const std::array<const char *, 1> output_names{"logits"};

  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
  std::vector<int64_t> token_ids;
  token_ids.reserve(static_cast<size_t>(kMaxDecodeSteps) + 1U);
  token_ids.push_back(kBosTokenId);

  for (int64_t step = 0; step < kMaxDecodeSteps; ++step) {
    const std::array<int64_t, 2> ids_shape{
      1,
      static_cast<int64_t>(token_ids.size())
    };

    Ort::Value ids_tensor = Ort::Value::CreateTensor<int64_t>(
      mem_info,
      token_ids.data(),
      token_ids.size(),
      ids_shape.data(),
      ids_shape.size());

    Ort::Value enc_tensor = Ort::Value::CreateTensor<float>(
      mem_info,
      const_cast<float *>(enc_hidden.data()),
      enc_hidden.size(),
      enc_shape.data(),
      enc_shape.size());

    std::array<Ort::Value, 2> inputs = {std::move(ids_tensor), std::move(enc_tensor)};

    std::vector<Ort::Value> outputs;
    try {
      outputs = decoder_session_->Run(
        Ort::RunOptions{nullptr},
        input_names.data(),
        inputs.data(),
        inputs.size(),
        output_names.data(),
        output_names.size());
    } catch (const Ort::Exception & e) {
      last_error_ = std::string("decoder inference failed: ") + e.what();
      return {};
    }

    if (outputs.empty() || !outputs[0].IsTensor()) {
      last_error_ = "decoder output is not a tensor";
      return {};
    }

    auto logits_info = outputs[0].GetTensorTypeAndShapeInfo();
    const auto logits_shape = logits_info.GetShape();
    if (logits_shape.size() != 3 || logits_shape[0] != 1 || logits_shape[1] <= 0 || logits_shape[2] <= 0) {
      last_error_ = "decoder logits shape is invalid";
      return {};
    }

    const size_t seq_len = static_cast<size_t>(logits_shape[1]);
    const size_t vocab_size = static_cast<size_t>(logits_shape[2]);
    const float * logits = outputs[0].GetTensorData<float>();
    const size_t offset = (seq_len - 1U) * vocab_size;

    const auto * begin = logits + offset;
    const auto * end = begin + vocab_size;
    const auto * max_it = std::max_element(begin, end);
    const int64_t next_id = static_cast<int64_t>(std::distance(begin, max_it));

    token_ids.push_back(next_id);
    if (next_id == kEosTokenId) {
      break;
    }
  }

  return token_ids;
}

std::string MoonshineOnnx::tokens_to_text(const std::vector<int64_t> & token_ids) const
{
  last_error_.clear();

  if (!vocab_loaded_) {
    vocab_loaded_ = true;
    vocab_id_to_piece_.clear();

    std::ifstream ifs(kVocabJsonPath);
    if (!ifs) {
      last_error_ = std::string("failed to open vocab json: ") + kVocabJsonPath;
      return {};
    }
    const std::string content(
      (std::istreambuf_iterator<char>(ifs)),
      std::istreambuf_iterator<char>());

    const std::regex id_piece_pattern(
      R"__REGEX__(\{[^{}]*"id"\s*:\s*([0-9]+)[^{}]*"piece"\s*:\s*"((?:\\.|[^"\\])*)")__REGEX__");
    const std::regex piece_id_pattern(
      R"__REGEX__(\{[^{}]*"piece"\s*:\s*"((?:\\.|[^"\\])*)"[^{}]*"id"\s*:\s*([0-9]+))__REGEX__");
    const std::regex key_value_pattern(
      R"__REGEX__("((?:\\.|[^"\\])*)"\s*:\s*([0-9]+))__REGEX__");

    bool found = false;
    for (std::sregex_iterator it(content.begin(), content.end(), id_piece_pattern), end; it != end; ++it) {
      const int64_t id = std::stoll((*it)[1].str());
      const std::string piece = json_unescape((*it)[2].str());
      if (id < 0) {
        continue;
      }
      if (static_cast<size_t>(id) >= vocab_id_to_piece_.size()) {
        vocab_id_to_piece_.resize(static_cast<size_t>(id) + 1U);
      }
      vocab_id_to_piece_[static_cast<size_t>(id)] = piece;
      found = true;
    }
    for (std::sregex_iterator it(content.begin(), content.end(), piece_id_pattern), end; it != end; ++it) {
      const int64_t id = std::stoll((*it)[2].str());
      const std::string piece = json_unescape((*it)[1].str());
      if (id < 0) {
        continue;
      }
      if (static_cast<size_t>(id) >= vocab_id_to_piece_.size()) {
        vocab_id_to_piece_.resize(static_cast<size_t>(id) + 1U);
      }
      vocab_id_to_piece_[static_cast<size_t>(id)] = piece;
      found = true;
    }
    if (!found) {
      for (std::sregex_iterator it(content.begin(), content.end(), key_value_pattern), end; it != end; ++it) {
        const std::string piece = json_unescape((*it)[1].str());
        const int64_t id = std::stoll((*it)[2].str());
        if (id < 0) {
          continue;
        }
        if (static_cast<size_t>(id) >= vocab_id_to_piece_.size()) {
          vocab_id_to_piece_.resize(static_cast<size_t>(id) + 1U);
        }
        vocab_id_to_piece_[static_cast<size_t>(id)] = piece;
        found = true;
      }
    }

    if (!found) {
      last_error_ = "failed to parse vocab json";
      return {};
    }
  }

  std::string text;
  for (int64_t id : token_ids) {
    if (id == kBosTokenId || id == kEosTokenId || id == kPadTokenId || id < 0) {
      continue;
    }
    if (static_cast<size_t>(id) >= vocab_id_to_piece_.size()) {
      continue;
    }
    std::string piece = vocab_id_to_piece_[static_cast<size_t>(id)];
    if (piece.empty()) {
      continue;
    }
    if (piece.front() == '<' && piece.back() == '>') {
      continue;
    }
    replace_all(piece, kSpmSpaceMarker, " ");
    text += piece;
  }

  return trim_copy(text);
}

}  // namespace stt_cpp
