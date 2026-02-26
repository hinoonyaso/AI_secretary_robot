#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "llm_cpp/llm_engine.hpp"

namespace llm_cpp
{

class LlmNode : public rclcpp::Node
{
public:
  LlmNode();

private:
  void declare_and_get_parameters();
  void on_chat_text(const std_msgs::msg::String::SharedPtr msg);

  bool llm_enabled_ = true;
  std::string input_topic_ = "/intent_router/chat_text";
  std::string response_topic_ = "/llm/response";
  std::string provider_topic_ = "/llm/provider";
  std::string debug_topic_ = "/llm/debug";

  std::string system_prompt_;
  std::string openai_api_key_;
  std::string openai_model_;
  std::string openai_url_;
  int openai_timeout_sec_ = 20;
  std::string groq_api_key_;
  std::string groq_model_;
  int groq_timeout_sec_ = 20;
  std::string gemini_api_key_;
  std::string gemini_model_;
  int gemini_timeout_sec_ = 20;
  std::string ollama_url_;
  std::string ollama_model_;
  int ollama_timeout_sec_ = 45;

  std::unique_ptr<LlmEngine> engine_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_chat_text_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_response_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_provider_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;
};

}  // namespace llm_cpp
