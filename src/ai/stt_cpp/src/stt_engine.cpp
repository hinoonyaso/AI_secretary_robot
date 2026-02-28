#include "stt_cpp/stt_engine.hpp"

#include <rover_common/curl_utils.hpp>
#include <rover_common/json_utils.hpp>
#include <rover_common/string_utils.hpp>

#include <curl/curl.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>

using namespace std;


namespace stt_cpp
{

static const rover_common::CurlGlobalGuard curl_guard;

SttEngine::SttEngine(const SttConfig & config)
: config_(config)
{
  if (config_.engine == "moonshine_onnx" || config_.engine == "auto") {
    MoonshineConfig moonshine_cfg;
    moonshine_cfg.encoder_path = config_.onnx_encoder_path;
    moonshine_cfg.decoder_path = config_.onnx_decoder_path;
    moonshine_cfg.use_cuda = config_.onnx_use_cuda;
    moonshine_cfg.gpu_device_id = config_.onnx_gpu_device_id;
    moonshine_cfg.intra_threads = config_.onnx_intra_threads;
    moonshine_onnx_ = std::make_unique<MoonshineOnnx>(moonshine_cfg);
  }
}

SttResult SttEngine::transcribe(const string & wav_path) const
{
  SttResult result;
  if (!config_.enabled) {
    result.error = "stt disabled";
    return result;
  }

  if (config_.engine == "moonshine_onnx") {
    return transcribe_moonshine_onnx(wav_path);
  }
  if (config_.engine == "groq") {
    if (config_.groq_api_key.empty()) {
      result.error = "groq_api_key is empty";
      return result;
    }
    return transcribe_groq(wav_path);
  }
  if (config_.engine == "auto") {
    SttResult local = transcribe_moonshine_onnx(wav_path);
    if (local.ok) {
      return local;
    }
    if (config_.groq_api_key.empty()) {
      return local;
    }
    SttResult groq = transcribe_groq(wav_path);
    groq.used_fallback = true;
    if (!groq.ok && !local.error.empty()) {
      groq.error = local.error + " | fallback=" + groq.error;
    }
    return groq;
  }

  result.error = "unknown stt engine: " + config_.engine;
  return result;
}

SttResult SttEngine::transcribe_pcm(const vector<float> & samples, uint32_t sample_rate) const
{
  SttResult out;
  if (!config_.enabled) {
    out.error = "stt disabled";
    return out;
  }
  if (samples.empty()) {
    out.error = "empty_pcm_samples";
    return out;
  }
  if (sample_rate == 0U) {
    out.error = "invalid_sample_rate";
    return out;
  }
  if (config_.engine == "moonshine_onnx") {
    return transcribe_moonshine_onnx_pcm(samples, sample_rate);
  }
  if (config_.engine == "groq") {
    if (config_.groq_api_key.empty()) {
      out.error = "groq_api_key is empty";
      return out;
    }
  } else if (config_.engine != "auto") {
    out.error = "unknown stt engine: " + config_.engine;
    return out;
  }

  const string tmp_path = "/tmp/stt_tmp_" +
    to_string(chrono::steady_clock::now().time_since_epoch().count()) + ".wav";
  if (!write_temp_wav(tmp_path, samples, sample_rate)) {
    out.error = "failed_to_write_temp_wav";
    return out;
  }
  SttResult groq = transcribe_groq(tmp_path);
  std::error_code ec;
  std::filesystem::remove(tmp_path, ec);
  return groq;
}

SttResult SttEngine::transcribe_moonshine_onnx(const std::string & wav_path) const
{
  SttResult out;
  out.engine = "moonshine_onnx";
  if (!moonshine_onnx_) {
    out.error = "moonshine onnx engine is not initialized";
    return out;
  }
  const std::string text = moonshine_onnx_->transcribe(wav_path);
  if (text.empty()) {
    out.error = moonshine_onnx_->last_error();
    return out;
  }
  out.ok = true;
  out.text = rover_common::trim(text);
  if (out.text.empty()) {
    out.ok = false;
    out.error = "moonshine_onnx_empty_text";
  }
  return out;
}

SttResult SttEngine::transcribe_moonshine_onnx_pcm(
  const std::vector<float> & samples, uint32_t sample_rate) const
{
  SttResult out;
  out.engine = "moonshine_onnx";
  if (!moonshine_onnx_) {
    out.error = "moonshine onnx engine is not initialized";
    return out;
  }
  const std::string text = moonshine_onnx_->transcribe_pcm(samples, sample_rate);
  if (text.empty()) {
    out.error = moonshine_onnx_->last_error();
    return out;
  }
  out.ok = true;
  out.text = rover_common::trim(text);
  if (out.text.empty()) {
    out.ok = false;
    out.error = "moonshine_onnx_empty_text";
  }
  return out;
}

SttResult SttEngine::transcribe_groq(const string & wav_path) const
{
  SttResult out;
  out.engine = "groq";

  CURL * curl = curl_easy_init();
  if (!curl) {
    out.error = "curl_init_failed";
    return out;
  }

  string response_body;
  string auth = "Authorization: Bearer " + config_.groq_api_key;
  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, auth.c_str());

  curl_mime * mime = curl_mime_init(curl);
  curl_mimepart * part = nullptr;

  part = curl_mime_addpart(mime);
  curl_mime_name(part, "file");
  curl_mime_filedata(part, wav_path.c_str());

  part = curl_mime_addpart(mime);
  curl_mime_name(part, "model");
  curl_mime_data(part, config_.groq_model.c_str(), CURL_ZERO_TERMINATED);

  if (!config_.groq_language.empty()) {
    part = curl_mime_addpart(mime);
    curl_mime_name(part, "language");
    curl_mime_data(part, config_.groq_language.c_str(), CURL_ZERO_TERMINATED);
  }

  part = curl_mime_addpart(mime);
  curl_mime_name(part, "response_format");
  curl_mime_data(part, "json", CURL_ZERO_TERMINATED);

  curl_easy_setopt(curl, CURLOPT_URL, "https://api.groq.com/openai/v1/audio/transcriptions");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, rover_common::curl_write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, config_.groq_timeout_sec);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);

  const CURLcode rc = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  if (rc != CURLE_OK) {
    out.error = "curl_error:" + string(curl_easy_strerror(rc));
  } else if (http_code != 200) {
    ostringstream oss;
    oss << "http_" << http_code;
    if (!response_body.empty()) {
      oss << ":" << rover_common::trim(response_body);
    }
    out.error = oss.str();
  } else {
    string text;
    if (rover_common::extract_json_string_field(response_body, "text", text)) {
      out.ok = true;
      out.text = rover_common::trim(text);
      if (out.text.empty()) {
        out.ok = false;
        out.error = "groq_empty_text";
      }
    } else {
      out.error = "groq_invalid_json_response:" + rover_common::trim(response_body);
    }
  }

  curl_mime_free(mime);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return out;
}

bool SttEngine::write_temp_wav(
  const string & path,
  const vector<float> & samples,
  uint32_t sample_rate) const
{
  ofstream ofs(path, ios::binary);
  if (!ofs) {
    return false;
  }

  const uint16_t channels = 1;
  const uint16_t bits_per_sample = 16;
  const uint16_t block_align = channels * (bits_per_sample / 8U);
  const uint32_t byte_rate = sample_rate * block_align;
  const uint32_t data_size = static_cast<uint32_t>(samples.size() * sizeof(int16_t));
  const uint32_t chunk_size = 36U + data_size;

  auto write_le16 = [&ofs](uint16_t v) {
      const char b[2] = {
        static_cast<char>(v & 0xFFU),
        static_cast<char>((v >> 8U) & 0xFFU)
      };
      ofs.write(b, sizeof(b));
    };
  auto write_le32 = [&ofs](uint32_t v) {
      const char b[4] = {
        static_cast<char>(v & 0xFFU),
        static_cast<char>((v >> 8U) & 0xFFU),
        static_cast<char>((v >> 16U) & 0xFFU),
        static_cast<char>((v >> 24U) & 0xFFU)
      };
      ofs.write(b, sizeof(b));
    };

  ofs.write("RIFF", 4);
  write_le32(chunk_size);
  ofs.write("WAVE", 4);
  ofs.write("fmt ", 4);
  write_le32(16U);
  write_le16(1U);
  write_le16(channels);
  write_le32(sample_rate);
  write_le32(byte_rate);
  write_le16(block_align);
  write_le16(bits_per_sample);
  ofs.write("data", 4);
  write_le32(data_size);

  for (float s : samples) {
    const float clamped = (s < -1.0F) ? -1.0F : (s > 1.0F ? 1.0F : s);
    const int16_t pcm = static_cast<int16_t>(clamped * 32767.0F);
    const char b[2] = {
      static_cast<char>(pcm & 0xFF),
      static_cast<char>((static_cast<uint16_t>(pcm) >> 8U) & 0xFFU)
    };
    ofs.write(b, sizeof(b));
  }

  return static_cast<bool>(ofs);
}

}  // namespace stt_cpp
