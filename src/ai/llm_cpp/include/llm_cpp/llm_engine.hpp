#pragma once

#include <memory>
#include <string>

#include "llm_cpp/llama_server_manager.hpp"

namespace llm_cpp
{

struct LlmConfig
{
  bool enabled = true;
  std::string system_prompt = "You are a concise Korean voice assistant.";
  // "auto"=llama.cpp→openai→groq→gemini chain
  // "llamacpp"/"ollama"=llama.cpp only ("ollama" is legacy-compatible alias)
  std::string provider = "auto";

  std::string openai_api_key;
  std::string openai_model = "gpt-4o";
  std::string openai_url = "https://models.inference.ai.azure.com/chat/completions";
  long openai_timeout_sec = 20;

  std::string groq_api_key;
  std::string groq_model = "llama-3.1-8b-instant";
  long groq_timeout_sec = 20;

  std::string gemini_api_key;
  std::string gemini_model = "gemini-2.5-flash";
  long gemini_timeout_sec = 20;

  std::string llamacpp_endpoint = "http://127.0.0.1:8081";
  std::string llamacpp_model = "qwen2.5-1.5b-instruct";
  long llamacpp_timeout_sec = 45;
};

struct LlmResult
{
  bool ok = false;
  bool used_fallback = false;
  std::string provider;
  std::string text;
  std::string error;
};

class LlmEngine
{
public:
  explicit LlmEngine(
    const LlmConfig & config,
    std::shared_ptr<LlamaServerManager> server_mgr = nullptr);
  LlmResult generate(const std::string & user_text) const;

private:
  LlmResult generate_llamacpp(const std::string & user_text) const;
  LlmResult generate_openai(const std::string & user_text) const;
  LlmResult generate_groq(const std::string & user_text) const;
  LlmResult generate_gemini(const std::string & user_text) const;

  LlmConfig config_;
  std::shared_ptr<LlamaServerManager> server_mgr_;
};

}  // namespace llm_cpp
