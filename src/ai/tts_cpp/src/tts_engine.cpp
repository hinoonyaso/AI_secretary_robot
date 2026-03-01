#include "tts_cpp/tts_engine.hpp"

#include <rover_common/shell_utils.hpp>
#include <rover_common/string_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

using namespace std;


namespace tts_cpp
{
TtsEngine::TtsEngine(const TtsConfig & config)
: config_(config)
{
  // espeak 전용 모드에서는 Piper 초기화를 생략해 모델 호환성 문제로 인한 기동 실패를 방지한다.
  if (config_.engine != "espeak" && !config_.piper_model_path.empty()) {
    PiperOnnxConfig piper_cfg;
    piper_cfg.model_path = config_.piper_model_path;
    const std::filesystem::path model_path(config_.piper_model_path);
    const std::filesystem::path model_dir = model_path.parent_path();

    piper_cfg.data_dir = config_.piper_data_dir;
    if (piper_cfg.data_dir.empty()) {
      piper_cfg.data_dir = model_dir.string();
    }

    piper_cfg.tokens_path = config_.piper_tokens_path;
    if (piper_cfg.tokens_path.empty()) {
      const std::filesystem::path tokens = model_dir / "tokens.txt";
      if (std::filesystem::is_regular_file(tokens)) {
        piper_cfg.tokens_path = tokens.string();
      }
    }

    piper_cfg.lexicon_path = config_.piper_lexicon_path;
    if (piper_cfg.lexicon_path.empty()) {
      const std::filesystem::path lexicon = model_dir / "lexicon.txt";
      if (std::filesystem::is_regular_file(lexicon)) {
        piper_cfg.lexicon_path = lexicon.string();
      }
    }

    piper_cfg.num_threads = 2;
    piper_cfg.debug = false;
    piper_onnx_ = std::make_unique<PiperOnnx>(piper_cfg);
  }

  error_code ec;
  if (!config_.output_dir.empty()) {
    filesystem::create_directories(config_.output_dir, ec);
  }
}

TtsResult TtsEngine::synthesize(const string & text) const
{
  /// engine 파라미터에 따라 시작점을 결정하는 TTS 체인
  /// "auto"  : edge-tts(클라우드) → Piper(로컬) → espeak-ng(로컬)
  /// "piper" : Piper(로컬) → espeak-ng(로컬)
  /// "espeak": espeak-ng(로컬)
  TtsResult out;
  if (!config_.enabled) {
    out.error = "tts disabled";
    return out;
  }
  if (text.empty()) {
    out.error = "empty_text";
    return out;
  }

  const bool skip_edge = (config_.engine == "piper" || config_.engine == "espeak");
  const bool skip_piper = (config_.engine == "espeak");

  TtsResult edge;
  if (!skip_edge) {
    const string edge_path = make_output_path("mp3");
    edge = synthesize_edge(text, edge_path);
    if (edge.ok) {
      return edge;
    }
  }

  TtsResult piper;
  if (!skip_piper) {
    const string piper_path = make_output_path("wav");
    piper = synthesize_piper(text, piper_path);
    piper.used_fallback = skip_edge ? false : true;
    if (piper.ok) {
      return piper;
    }
  }

  const string espeak_path = make_output_path("wav");
  TtsResult espeak = synthesize_espeak(text, espeak_path);
  espeak.used_fallback = (skip_edge && skip_piper) ? false : true;
  if (espeak.ok) {
    return espeak;
  }

  TtsResult merged;
  merged.ok = false;
  string engines;
  if (!skip_edge) {
    engines += "edge-tts";
    merged.error += "edge failed: " + edge.error;
  }
  if (!skip_piper) {
    if (!engines.empty()) { engines += "+"; merged.error += " | "; }
    engines += "piper";
    merged.error += "piper failed: " + piper.error;
  }
  if (!engines.empty()) { engines += "+"; merged.error += " | "; }
  engines += "espeak-ng";
  merged.error += "espeak failed: " + espeak.error;
  merged.engine = engines;
  return merged;
}

TtsResult TtsEngine::synthesize_edge(const string & text, const string & out_path) const
{
  /// edge-tts 파이썬 스크립트를 실행하고 결과 파일 경로를 반환
  TtsResult out;
  out.engine = "edge-tts";

  if (config_.edge_script_path.empty()) {
    out.error = "edge_script_path_empty";
    return out;
  }

  ostringstream cmd;
  cmd << "python3 " << rover_common::shell_escape_single_quote(config_.edge_script_path)
      << " --text " << rover_common::shell_escape_single_quote(text)
      << " --voice " << rover_common::shell_escape_single_quote(config_.edge_voice)
      << " --rate " << rover_common::shell_escape_single_quote(config_.edge_rate)
      << " --volume " << rover_common::shell_escape_single_quote(config_.edge_volume)
      << " --output " << rover_common::shell_escape_single_quote(out_path)
      << " 2>&1";

  const rover_common::ShellResult shell = rover_common::run_shell_command(cmd.str());
  if (shell.exit_code < 0) {
    out.error = "edge_popen_failed";
    return out;
  }

  if (shell.ok) {
    out.ok = true;
    out.audio_path = out_path;
    return out;
  }

  ostringstream err;
  err << "edge_failed(exit=" << shell.exit_code << "): " << rover_common::trim(shell.output);
  out.error = err.str();
  return out;
}

TtsResult TtsEngine::synthesize_piper(const string & text, const string & out_path) const
{
  /// Piper TTS (로컬 VITS ONNX) — sherpa-onnx C API 직접 호출
  TtsResult out;
  out.engine = "piper";

  if (!piper_onnx_ || !piper_onnx_->is_ready()) {
    out.error = "piper_onnx_not_initialized";
    if (piper_onnx_ && !piper_onnx_->last_error().empty()) {
      out.error += ": " + piper_onnx_->last_error();
    }
    return out;
  }

  const PiperOnnxResult gen = piper_onnx_->synthesize(text);
  if (!gen.ok || gen.pcm_samples.empty() || gen.sample_rate == 0U) {
    out.error = "piper_onnx_failed: " + gen.error;
    return out;
  }

  out.pcm_samples.reserve(gen.pcm_samples.size());
  for (float v : gen.pcm_samples) {
    out.pcm_samples.push_back(float_to_pcm16(v));
  }
  out.pcm_sample_rate = gen.sample_rate;
  out.pcm_channels = 1;

  if (!write_pcm_to_wav(out_path, out.pcm_samples, out.pcm_sample_rate)) {
    out.error = "failed_to_write_wav";
    out.pcm_samples.clear();
    out.pcm_sample_rate = 0;
    return out;
  }

  out.ok = true;
  out.audio_path = out_path;
  return out;
}

TtsResult TtsEngine::synthesize_espeak(const string & text, const string & out_path) const
{
  /// 마지막 안전망으로 espeak-ng를 사용해 WAV 생성
  TtsResult out;
  out.engine = "espeak-ng";

  ostringstream cmd;
  cmd << rover_common::shell_escape_single_quote(config_.espeak_executable)
      << " -v " << rover_common::shell_escape_single_quote(config_.espeak_voice)
      << " -w " << rover_common::shell_escape_single_quote(out_path)
      << " " << rover_common::shell_escape_single_quote(text)
      << " 2>&1";

  const rover_common::ShellResult shell = rover_common::run_shell_command(cmd.str());
  if (shell.exit_code < 0) {
    out.error = "espeak_popen_failed";
    return out;
  }

  if (shell.ok) {
    out.ok = true;
    out.audio_path = out_path;
    return out;
  }

  ostringstream err;
  err << "espeak_failed(exit=" << shell.exit_code << "): " << rover_common::trim(shell.output);
  out.error = err.str();
  return out;
}

string TtsEngine::make_output_path(const string & extension) const
{
  /// 파일명 충돌을 줄이기 위해 epoch milliseconds 기반으로 출력 경로 생성
  const auto now = chrono::system_clock::now();
  const auto ms = chrono::duration_cast<chrono::milliseconds>(
    now.time_since_epoch()).count();
  ostringstream oss;
  oss << config_.output_dir << "/tts_" << ms << "." << extension;
  return oss.str();
}

bool TtsEngine::write_pcm_to_wav(
  const std::string & path,
  const std::vector<int16_t> & samples,
  uint32_t sample_rate) const
{
  if (samples.empty() || sample_rate == 0U) {
    return false;
  }

  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    return false;
  }

  const uint16_t channels = 1;
  const uint16_t bits_per_sample = 16;
  const uint16_t block_align = channels * (bits_per_sample / 8U);
  const uint32_t byte_rate = sample_rate * block_align;
  const uint32_t data_size = static_cast<uint32_t>(samples.size() * sizeof(int16_t));
  const uint32_t chunk_size = 36U + data_size;

  auto write_u16 = [&ofs](uint16_t v) {
      const char b[2] = {
        static_cast<char>(v & 0xFFU),
        static_cast<char>((v >> 8U) & 0xFFU)
      };
      ofs.write(b, sizeof(b));
    };
  auto write_u32 = [&ofs](uint32_t v) {
      const char b[4] = {
        static_cast<char>(v & 0xFFU),
        static_cast<char>((v >> 8U) & 0xFFU),
        static_cast<char>((v >> 16U) & 0xFFU),
        static_cast<char>((v >> 24U) & 0xFFU)
      };
      ofs.write(b, sizeof(b));
    };

  ofs.write("RIFF", 4);
  write_u32(chunk_size);
  ofs.write("WAVE", 4);
  ofs.write("fmt ", 4);
  write_u32(16U);
  write_u16(1U);
  write_u16(channels);
  write_u32(sample_rate);
  write_u32(byte_rate);
  write_u16(block_align);
  write_u16(bits_per_sample);
  ofs.write("data", 4);
  write_u32(data_size);

  for (const int16_t pcm : samples) {
    const char b[2] = {
      static_cast<char>(pcm & 0xFF),
      static_cast<char>((static_cast<uint16_t>(pcm) >> 8U) & 0xFFU)
    };
    ofs.write(b, sizeof(b));
  }

  return static_cast<bool>(ofs);
}

int16_t TtsEngine::float_to_pcm16(float sample)
{
  const float clamped = std::clamp(sample, -1.0F, 1.0F);
  return static_cast<int16_t>(clamped * 32767.0F);
}

}  // namespace tts_cpp
