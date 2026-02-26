#include "tts_cpp/tts_engine.hpp"

#include <rover_common/curl_utils.hpp>
#include <rover_common/json_utils.hpp>
#include <rover_common/shell_utils.hpp>
#include <rover_common/string_utils.hpp>

#include <curl/curl.h>

#include <chrono>
#include <filesystem>
#include <cstring>
#include <sstream>
#include <system_error>

using namespace std;


namespace tts_cpp
{
namespace
{

static const rover_common::CurlGlobalGuard curl_guard;

size_t curl_header_callback(char * buffer, size_t size, size_t nitems, void * userdata)
{
  const size_t total = size * nitems;
  if (userdata == nullptr || total == 0) {
    return total;
  }
  auto * sample_rate = static_cast<uint32_t *>(userdata);
  const string line(buffer, buffer + total);
  const string key = "X-Sample-Rate:";
  if (line.rfind(key, 0) == 0) {
    const string value = rover_common::trim(line.substr(key.size()));
    try {
      const unsigned long parsed = std::stoul(value);
      *sample_rate = static_cast<uint32_t>(parsed);
    } catch (...) {
    }
  }
  return total;
}

}  // namespace

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
  /// "auto"  : edge-tts(클라우드) → MeloTTS(로컬) → espeak-ng(로컬)
  /// "melo"  : MeloTTS(로컬) → espeak-ng(로컬)
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

  const bool skip_edge = (config_.engine == "melo" || config_.engine == "espeak");
  const bool skip_melo = (config_.engine == "espeak");

  TtsResult edge;
  if (!skip_edge) {
    const string edge_path = make_output_path("mp3");
    edge = synthesize_edge(text, edge_path);
    if (edge.ok) {
      return edge;
    }
  }

  TtsResult melo;
  if (!skip_melo) {
    const string melo_path = make_output_path("wav");
    melo = synthesize_melo(text, melo_path);
    melo.used_fallback = skip_edge ? false : true;
    if (melo.ok) {
      return melo;
    }
  }

  const string espeak_path = make_output_path("wav");
  TtsResult espeak = synthesize_espeak(text, espeak_path);
  espeak.used_fallback = (skip_edge && skip_melo) ? false : true;
  if (espeak.ok) {
    return espeak;
  }

  TtsResult merged;
  merged.ok = false;
  merged.engine = skip_edge ? (skip_melo ? "espeak-ng" : "melotts+espeak-ng")
                             : "edge-tts+melotts+espeak-ng";
  if (!skip_edge) {
    merged.error += "edge failed: " + edge.error + " | ";
  }
  if (!skip_melo) {
    merged.error += "melo failed: " + melo.error + " | ";
  }
  merged.error += "espeak failed: " + espeak.error;
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

TtsResult TtsEngine::synthesize_melo(const string & text, const string & out_path) const
{
  /// MeloTTS 상주 서버에서 PCM을 직접 받아오고 실패 시 스크립트 fallback을 시도
  TtsResult out;
  out.engine = "MeloTTS-server";

  if (!config_.melo_server_url.empty()) {
    CURL * curl = curl_easy_init();
    if (curl != nullptr) {
      string response_body;
      uint32_t sample_rate = 0U;
      double speed_value = 1.0;
      try {
        speed_value = stod(config_.melo_speed);
      } catch (...) {
        speed_value = 1.0;
      }
      ostringstream body;
      body << "{\"text\":\"" << rover_common::json_escape(text)
           << "\",\"speed\":" << speed_value << "}";
      const string request_body = body.str();
      struct curl_slist * headers = nullptr;
      headers = curl_slist_append(headers, "Content-Type: application/json");

      curl_easy_setopt(curl, CURLOPT_URL, config_.melo_server_url.c_str());
      curl_easy_setopt(curl, CURLOPT_POST, 1L);
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
      curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, request_body.size());
      curl_easy_setopt(curl, CURLOPT_TIMEOUT, config_.melo_server_timeout_sec);
      curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, rover_common::curl_write_callback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
      curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, curl_header_callback);
      curl_easy_setopt(curl, CURLOPT_HEADERDATA, &sample_rate);

      const CURLcode rc = curl_easy_perform(curl);
      long http_code = 0;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

      if (rc == CURLE_OK && http_code == 200 && !response_body.empty()) {
        if (response_body.size() % sizeof(int16_t) == 0U) {
          out.ok = true;
          out.engine = "MeloTTS-server";
          out.pcm_sample_rate = (sample_rate == 0U) ? 44100U : sample_rate;
          out.pcm_channels = 1;
          out.pcm_samples.resize(response_body.size() / sizeof(int16_t));
          std::memcpy(
            out.pcm_samples.data(),
            response_body.data(),
            out.pcm_samples.size() * sizeof(int16_t));
          curl_slist_free_all(headers);
          curl_easy_cleanup(curl);
          return out;
        }
        out.error = "melo_server_invalid_pcm_size";
      } else if (rc != CURLE_OK) {
        out.error = "melo_server_curl_error:" + string(curl_easy_strerror(rc));
      } else {
        ostringstream oss;
        oss << "melo_server_http_" << http_code;
        out.error = oss.str();
      }

      curl_slist_free_all(headers);
      curl_easy_cleanup(curl);
    } else {
      out.error = "melo_server_curl_init_failed";
    }
  }

  if (config_.melo_script_path.empty()) {
    if (out.error.empty()) {
      out.error = "melo_script_path_empty";
    }
    return out;
  }

  ostringstream cmd;
  cmd << "python3 " << rover_common::shell_escape_single_quote(config_.melo_script_path)
      << " --text " << rover_common::shell_escape_single_quote(text)
      << " --output_file " << rover_common::shell_escape_single_quote(out_path)
      << " --language " << rover_common::shell_escape_single_quote(config_.melo_language)
      << " --speaker " << rover_common::shell_escape_single_quote(config_.melo_speaker)
      << " --speed " << rover_common::shell_escape_single_quote(config_.melo_speed)
      << " --device " << rover_common::shell_escape_single_quote(config_.melo_device)
      << " 2>&1";

  const rover_common::ShellResult shell = rover_common::run_shell_command(cmd.str());
  if (shell.exit_code < 0) {
    out.error = out.error.empty() ? "melo_popen_failed" : out.error + " | melo_popen_failed";
    return out;
  }
  if (shell.ok) {
    out.ok = true;
    out.engine = "MeloTTS";
    out.audio_path = out_path;
    return out;
  }

  ostringstream err;
  err << "melo_failed(exit=" << shell.exit_code << "): " << rover_common::trim(shell.output);
  if (out.error.empty()) {
    out.error = err.str();
  } else {
    out.error += " | " + err.str();
  }
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
