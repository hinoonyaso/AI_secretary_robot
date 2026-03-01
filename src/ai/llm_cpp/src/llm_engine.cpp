#include "llm_cpp/llm_engine.hpp"

#include <rover_common/curl_utils.hpp>
#include <rover_common/json_utils.hpp>
#include <rover_common/string_utils.hpp>

#include <curl/curl.h>

#include <sstream>
#include <utility>
#include <vector>

using namespace std;


namespace llm_cpp
{
namespace
{

static const rover_common::CurlGlobalGuard curl_guard;

}  // namespace

LlmEngine::LlmEngine(const LlmConfig & config, std::shared_ptr<LlamaServerManager> server_mgr)
: config_(config), server_mgr_(std::move(server_mgr))
{
}

/// provider 파라미터에 따라 시작점을 결정하는 LLM 체인
/// "auto"     : llama.cpp → OpenAI → Groq → Gemini
/// "llamacpp" : llama.cpp 직접 호출
/// "ollama"   : 레거시 호환을 위해 llama.cpp 직접 호출로 매핑
LlmResult LlmEngine::generate(const string & user_text) const
{
  LlmResult out;
  if (!config_.enabled) {
    out.error = "llm disabled";
    return out;
  }

  if (config_.provider == "llamacpp" || config_.provider == "ollama") {
    return generate_llamacpp(user_text);
  }

  LlmResult local = generate_llamacpp(user_text);
  if (local.ok) {
    return local;
  }

  LlmResult openai = generate_openai(user_text);
  openai.used_fallback = true;
  if (openai.ok) {
    return openai;
  }

  LlmResult groq = generate_groq(user_text);
  groq.used_fallback = true;
  if (groq.ok) {
    return groq;
  }

  LlmResult gemini = generate_gemini(user_text);
  gemini.used_fallback = true;
  if (gemini.ok) {
    return gemini;
  }

  LlmResult merged;
  merged.ok = false;
  merged.provider = "llama.cpp+openai+groq+gemini";
  merged.error =
    "llama.cpp failed: " + local.error + " | openai failed: " + openai.error +
    " | groq failed: " + groq.error + " | gemini failed: " + gemini.error;
  return merged;
}

LlmResult LlmEngine::generate_llamacpp(const string & user_text) const
{
  /// llama.cpp(OpenAI 호환) 로컬 HTTP 서버로 1차 생성 시도
  LlmResult out;
  out.provider = "llama.cpp";

  string endpoint = config_.llamacpp_endpoint;
  if (server_mgr_) {
    endpoint = server_mgr_->endpoint();
    if (!server_mgr_->is_healthy()) {
      out.error = "llamacpp_unhealthy";
      return out;
    }
  }

  const vector<string> headers{"Content-Type: application/json"};

  ostringstream body;
  body << "{"
       << "\"model\":\"" << rover_common::json_escape(config_.llamacpp_model) << "\","
       << "\"temperature\":0.3,"
       << "\"messages\":["
       << "{\"role\":\"system\",\"content\":\"" << rover_common::json_escape(config_.system_prompt) << "\"},"
       << "{\"role\":\"user\",\"content\":\"" << rover_common::json_escape(user_text) << "\"}"
       << "]"
       << "}";

  long http_code = 0;
  string response_body;
  string request_error;
  if (!rover_common::perform_post_json(
      endpoint + "/v1/chat/completions",
      headers,
      body.str(),
      config_.llamacpp_timeout_sec,
      http_code,
      response_body,
      request_error))
  {
    if (request_error.find("curl_error:") == 0) {
      out.error = "local_error:" + request_error.substr(string("curl_error:").size());
    } else {
      out.error = request_error;
    }
    return out;
  }

  if (http_code != 200) {
    ostringstream err;
    err << "http_" << http_code;
    if (!response_body.empty()) {
      err << ":" << rover_common::trim(response_body);
    }
    out.error = err.str();
    return out;
  }

  string text;
  if (!rover_common::extract_json_string_field(response_body, "content", text)) {
    out.error = "invalid_response:" + rover_common::trim(response_body);
    return out;
  }
  out.text = rover_common::trim(text);
  if (out.text.empty()) {
    out.error = "empty_response";
    return out;
  }

  out.ok = true;
  return out;
}

LlmResult LlmEngine::generate_openai(const string & user_text) const
{
  /// OpenAI 호환 Chat Completions API를 사용해 GPT-4 계열 1차 생성 시도
  LlmResult out;
  out.provider = "openai";

  if (config_.openai_api_key.empty()) {
    out.error = "openai_key_empty";
    return out;
  }

  const vector<string> headers{
    "Content-Type: application/json",
    "Authorization: Bearer " + config_.openai_api_key};

  ostringstream body;
  body << "{"
       << "\"model\":\"" << rover_common::json_escape(config_.openai_model) << "\","
       << "\"temperature\":0.3,"
       << "\"messages\":["
       << "{\"role\":\"system\",\"content\":\"" << rover_common::json_escape(config_.system_prompt) << "\"},"
       << "{\"role\":\"user\",\"content\":\"" << rover_common::json_escape(user_text) << "\"}"
       << "]"
       << "}";

  long http_code = 0;
  string response_body;
  string request_error;
  if (!rover_common::perform_post_json(
      config_.openai_url, headers, body.str(),
      config_.openai_timeout_sec, http_code, response_body, request_error))
  {
    if (request_error.find("curl_error:") == 0) {
      const string code = request_error.substr(string("curl_error:").size());
      out.error = "network_error:" + code;
    } else {
      out.error = request_error;
    }
    return out;
  }

  if (http_code != 200) {
    ostringstream err;
    err << "http_" << http_code;
    if (!response_body.empty()) {
      err << ":" << rover_common::trim(response_body);
    }
    out.error = err.str();
    return out;
  }

  string text;
  if (!rover_common::extract_json_string_field(response_body, "content", text)) {
    out.error = "invalid_response:" + rover_common::trim(response_body);
    return out;
  }
  out.text = rover_common::trim(text);
  if (out.text.empty()) {
    out.error = "empty_response";
    return out;
  }

  out.ok = true;
  return out;
}

LlmResult LlmEngine::generate_groq(const string & user_text) const
{
  /// Groq(OpenAI 호환) 요청/응답 스키마를 사용해 1차 생성 시도
  LlmResult out;
  out.provider = "groq";

  if (config_.groq_api_key.empty()) {
    out.error = "groq_key_empty";
    return out;
  }

  const string url = "https://api.groq.com/openai/v1/chat/completions";
  const vector<string> headers{
    "Content-Type: application/json",
    "Authorization: Bearer " + config_.groq_api_key};

  ostringstream body;
  body << "{"
       << "\"model\":\"" << rover_common::json_escape(config_.groq_model) << "\","
       << "\"temperature\":0.3,"
       << "\"messages\":["
       << "{\"role\":\"system\",\"content\":\"" << rover_common::json_escape(config_.system_prompt) << "\"},"
       << "{\"role\":\"user\",\"content\":\"" << rover_common::json_escape(user_text) << "\"}"
       << "]"
       << "}";

  long http_code = 0;
  string response_body;
  string request_error;
  if (!rover_common::perform_post_json(
      url, headers, body.str(), config_.groq_timeout_sec, http_code, response_body, request_error))
  {
    if (request_error.find("curl_error:") == 0) {
      const string code = request_error.substr(string("curl_error:").size());
      out.error = "network_error:" + code;
    } else {
      out.error = request_error;
    }
    return out;
  }

  if (http_code != 200) {
    ostringstream err;
    err << "http_" << http_code;
    if (!response_body.empty()) {
      err << ":" << rover_common::trim(response_body);
    }
    out.error = err.str();
    return out;
  }

  string text;
  if (!rover_common::extract_json_string_field(response_body, "content", text)) {
    out.error = "invalid_response:" + rover_common::trim(response_body);
    return out;
  }
  out.text = rover_common::trim(text);
  if (out.text.empty()) {
    out.error = "empty_response";
    return out;
  }

  out.ok = true;
  return out;
}

LlmResult LlmEngine::generate_gemini(const string & user_text) const
{
  /// Gemini REST 포맷으로 변환해 2차 생성 시도
  LlmResult out;
  out.provider = "gemini";

  if (config_.gemini_api_key.empty()) {
    out.error = "gemini_key_empty";
    return out;
  }

  const string url =
    "https://generativelanguage.googleapis.com/v1beta/models/" + config_.gemini_model +
    ":generateContent?key=" + config_.gemini_api_key;
  const vector<string> headers{"Content-Type: application/json"};

  ostringstream body;
  body << "{"
       << "\"system_instruction\":{\"parts\":[{\"text\":\"" << rover_common::json_escape(config_.system_prompt) << "\"}]},"
       << "\"contents\":[{\"parts\":[{\"text\":\"" << rover_common::json_escape(user_text) << "\"}]}]"
       << "}";

  long http_code = 0;
  string response_body;
  string request_error;
  if (!rover_common::perform_post_json(
      url, headers, body.str(), config_.gemini_timeout_sec, http_code, response_body, request_error))
  {
    if (request_error.find("curl_error:") == 0) {
      out.error = "network_error:" + request_error.substr(string("curl_error:").size());
    } else {
      out.error = request_error;
    }
    return out;
  }

  if (http_code != 200) {
    ostringstream err;
    err << "http_" << http_code;
    if (!response_body.empty()) {
      err << ":" << rover_common::trim(response_body);
    }
    out.error = err.str();
    return out;
  }

  string text;
  if (!rover_common::extract_json_string_field(response_body, "text", text)) {
    out.error = "invalid_response:" + rover_common::trim(response_body);
    return out;
  }
  out.text = rover_common::trim(text);
  if (out.text.empty()) {
    out.error = "empty_response";
    return out;
  }

  out.ok = true;
  return out;
}

}  // namespace llm_cpp
