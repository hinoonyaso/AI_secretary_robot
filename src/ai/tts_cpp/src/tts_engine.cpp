#include "tts_cpp/tts_engine.hpp"

#include <rover_common/shell_utils.hpp>
#include <rover_common/string_utils.hpp>

#include <chrono>
#include <filesystem>
#include <sstream>
#include <system_error>

using namespace std;


namespace tts_cpp
{
TtsEngine::TtsEngine(const TtsConfig & config)
: config_(config)
{
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
  /// Piper TTS (로컬 VITS ONNX) — sherpa-onnx 또는 piper 바이너리 호출
  TtsResult out;
  out.engine = "piper";

  if (config_.piper_model_path.empty()) {
    out.error = "piper_model_path_empty";
    return out;
  }

  ostringstream cmd;
  cmd << rover_common::shell_escape_single_quote(config_.piper_executable)
      << " --model " << rover_common::shell_escape_single_quote(config_.piper_model_path);
  if (!config_.piper_config_path.empty()) {
    cmd << " --config " << rover_common::shell_escape_single_quote(config_.piper_config_path);
  }
  if (!config_.piper_speaker.empty()) {
    cmd << " --speaker " << rover_common::shell_escape_single_quote(config_.piper_speaker);
  }
  cmd << " --output_file " << rover_common::shell_escape_single_quote(out_path)
      << " 2>&1";

  // piper reads text from stdin
  const string full_cmd = string("echo ") + rover_common::shell_escape_single_quote(text)
    + " | " + cmd.str();

  const rover_common::ShellResult shell = rover_common::run_shell_command(full_cmd);
  if (shell.exit_code < 0) {
    out.error = "piper_popen_failed";
    return out;
  }

  if (shell.ok) {
    out.ok = true;
    out.audio_path = out_path;
    return out;
  }

  ostringstream err;
  err << "piper_failed(exit=" << shell.exit_code << "): " << rover_common::trim(shell.output);
  out.error = err.str();
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

}  // namespace tts_cpp
