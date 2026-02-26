#pragma once

#include <string>

namespace llm_cpp
{

struct LlmConfig
{
  bool enabled = true;
  std::string system_prompt = "You are a concise Korean voice assistant.";
  // "auto"=openai→groq→gemini→ollama chain, "ollama"=ollama only
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

  std::string ollama_url = "http://127.0.0.1:11434/api/generate";
  std::string ollama_model = "qwen2.5:1.5b";
  long ollama_timeout_sec = 45;
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
  explicit LlmEngine(const LlmConfig & config);
  LlmResult generate(const std::string & user_text) const;

private:
  LlmResult generate_openai(const std::string & user_text) const;
  LlmResult generate_groq(const std::string & user_text) const;
  LlmResult generate_gemini(const std::string & user_text) const;
  LlmResult generate_ollama(const std::string & user_text) const;

  LlmConfig config_;
};

}  // namespace llm_cpp
