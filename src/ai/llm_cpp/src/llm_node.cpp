#include "llm_cpp/llm_node.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>

using namespace std;


namespace llm_cpp
{

LlmNode::LlmNode()
: Node("llm_node")
{
  declare_and_get_parameters();

  pub_response_ = create_publisher<std_msgs::msg::String>(response_topic_, 10);
  pub_provider_ = create_publisher<std_msgs::msg::String>(provider_topic_, 10);
  pub_debug_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);

  sub_chat_text_ = create_subscription<std_msgs::msg::String>(
    input_topic_, 10, bind(&LlmNode::on_chat_text, this, placeholders::_1));

  health_timer_ = create_wall_timer(
    chrono::minutes(1),
    [this]() {
      if (!server_mgr_) {
        return;
      }
      if (!server_mgr_->is_healthy()) {
        RCLCPP_WARN(get_logger(), "llama.cpp server unhealthy, restarting");
        if (!server_mgr_->restart()) {
          RCLCPP_ERROR(
            get_logger(), "llama.cpp restart failed: %s", server_mgr_->last_error().c_str());
        }
      }
    });

  RCLCPP_INFO(get_logger(), "llm_cpp node started");
}

void LlmNode::declare_and_get_parameters()
{
  declare_parameter<bool>("llm_enabled", true);
  declare_parameter<string>("input_topic", "/intent_router/chat_text");
  declare_parameter<string>("response_topic", "/llm/response");
  declare_parameter<string>("provider_topic", "/llm/provider");
  declare_parameter<string>("debug_topic", "/llm/debug");

  declare_parameter<string>("system_prompt", "You are a concise Korean voice assistant.");

  declare_parameter<string>("openai_api_key", "");
  declare_parameter<string>("openai_model", "gpt-4o");
  declare_parameter<string>("openai_url", "https://models.inference.ai.azure.com/chat/completions");
  declare_parameter<int>("openai_timeout_sec", 20);

  declare_parameter<string>("groq_api_key", "");
  declare_parameter<string>("groq_model", "llama-3.1-8b-instant");
  declare_parameter<int>("groq_timeout_sec", 20);

  declare_parameter<string>("gemini_api_key", "");
  declare_parameter<string>("gemini_model", "gemini-2.5-flash");
  declare_parameter<int>("gemini_timeout_sec", 20);

  declare_parameter<string>("llamacpp_endpoint", "http://127.0.0.1:8081");
  declare_parameter<string>("llamacpp_model", "qwen2.5-1.5b-instruct");
  declare_parameter<int>("llamacpp_timeout_sec", 45);

  declare_parameter<string>(
    "llama_server_binary",
    "/usr/local/bin/llama-server");
  declare_parameter<string>(
    "llama_model_path",
    "/home/sang/dev_ws/AI_secretary_robot/models/llm/qwen2.5-1.5b-instruct-q4_k_m.gguf");
  declare_parameter<int>("llama_port", 8081);
  declare_parameter<int>("llama_ngl", 99);
  declare_parameter<int>("llama_ctx_size", 2048);
  declare_parameter<int>("llama_threads", 4);
  declare_parameter<bool>("llama_use_mmap", true);
  declare_parameter<bool>("llama_use_mlock", false);
  declare_parameter<string>("llm_provider", "auto");

  llm_enabled_ = get_parameter("llm_enabled").as_bool();
  input_topic_ = get_parameter("input_topic").as_string();
  response_topic_ = get_parameter("response_topic").as_string();
  provider_topic_ = get_parameter("provider_topic").as_string();
  debug_topic_ = get_parameter("debug_topic").as_string();
  system_prompt_ = get_parameter("system_prompt").as_string();

  openai_api_key_ = get_parameter("openai_api_key").as_string();
  openai_model_ = get_parameter("openai_model").as_string();
  openai_url_ = get_parameter("openai_url").as_string();
  openai_timeout_sec_ = get_parameter("openai_timeout_sec").as_int();

  groq_api_key_ = get_parameter("groq_api_key").as_string();
  groq_model_ = get_parameter("groq_model").as_string();
  groq_timeout_sec_ = get_parameter("groq_timeout_sec").as_int();
  gemini_api_key_ = get_parameter("gemini_api_key").as_string();
  gemini_model_ = get_parameter("gemini_model").as_string();
  gemini_timeout_sec_ = get_parameter("gemini_timeout_sec").as_int();
  llamacpp_endpoint_ = get_parameter("llamacpp_endpoint").as_string();
  llamacpp_model_ = get_parameter("llamacpp_model").as_string();
  llamacpp_timeout_sec_ = get_parameter("llamacpp_timeout_sec").as_int();
  llama_server_binary_ = get_parameter("llama_server_binary").as_string();
  llama_model_path_ = get_parameter("llama_model_path").as_string();
  llama_port_ = get_parameter("llama_port").as_int();
  llama_ngl_ = get_parameter("llama_ngl").as_int();
  llama_ctx_size_ = get_parameter("llama_ctx_size").as_int();
  llama_threads_ = get_parameter("llama_threads").as_int();
  llama_use_mmap_ = get_parameter("llama_use_mmap").as_bool();
  llama_use_mlock_ = get_parameter("llama_use_mlock").as_bool();

  if (openai_api_key_.empty()) {
    const char * key = getenv("OPENAI_API_KEY");
    if (key) {
      openai_api_key_ = key;
    } else {
      const char * github_token = getenv("GITHUB_TOKEN");
      if (github_token) {
        openai_api_key_ = github_token;
      } else {
        const char * gh_token = getenv("GH_TOKEN");
        if (gh_token) {
          openai_api_key_ = gh_token;
        }
      }
    }
  }
  if (groq_api_key_.empty()) {
    const char * key = getenv("GROQ_API_KEY");
    if (key) {
      groq_api_key_ = key;
    }
  }
  if (gemini_api_key_.empty()) {
    const char * gemini_key = getenv("GEMINI_API_KEY");
    if (gemini_key) {
      gemini_api_key_ = gemini_key;
    } else {
      const char * google_key = getenv("GOOGLE_API_KEY");
      if (google_key) {
        gemini_api_key_ = google_key;
      }
    }
  }

  LlmConfig config;
  config.enabled = llm_enabled_;
  config.system_prompt = system_prompt_;
  config.openai_api_key = openai_api_key_;
  config.openai_model = openai_model_;
  config.openai_url = openai_url_;
  config.openai_timeout_sec = openai_timeout_sec_;
  config.groq_api_key = groq_api_key_;
  config.groq_model = groq_model_;
  config.groq_timeout_sec = groq_timeout_sec_;
  config.gemini_api_key = gemini_api_key_;
  config.gemini_model = gemini_model_;
  config.gemini_timeout_sec = gemini_timeout_sec_;
  config.llamacpp_endpoint = llamacpp_endpoint_;
  config.llamacpp_model = llamacpp_model_;
  config.llamacpp_timeout_sec = llamacpp_timeout_sec_;
  config.provider = get_parameter("llm_provider").as_string();

  LlamaServerConfig server_cfg;
  server_cfg.server_binary = llama_server_binary_;
  server_cfg.model_path = llama_model_path_;
  server_cfg.port = llama_port_;
  server_cfg.ngl = llama_ngl_;
  server_cfg.ctx_size = llama_ctx_size_;
  server_cfg.n_threads = llama_threads_;
  server_cfg.use_mmap = llama_use_mmap_;
  server_cfg.use_mlock = llama_use_mlock_;
  server_mgr_ = std::make_shared<LlamaServerManager>(server_cfg);

  const bool has_server_binary = std::filesystem::is_regular_file(llama_server_binary_);
  const bool has_model_file = std::filesystem::is_regular_file(llama_model_path_);
  if (!has_server_binary || !has_model_file) {
    RCLCPP_INFO(
      get_logger(),
      "Skip llama.cpp autostart (binary/model not found). binary=%s model=%s",
      has_server_binary ? "ok" : "missing",
      has_model_file ? "ok" : "missing");
  } else if (!server_mgr_->start()) {
    RCLCPP_WARN(
      get_logger(), "llama.cpp server start failed: %s (fallback chain continues)",
      server_mgr_->last_error().c_str());
  } else {
    RCLCPP_INFO(get_logger(), "llama.cpp server started at %s", server_mgr_->endpoint().c_str());
  }

  engine_ = unique_ptr<LlmEngine>(new LlmEngine(config, server_mgr_));
}

void LlmNode::on_chat_text(const std_msgs::msg::String::SharedPtr msg)
{
  /// 입력 문장을 LLM 엔진에 전달하고, 성공/실패/폴백 상태를 각각 토픽으로 분리 발행
  if (!engine_) {
    RCLCPP_WARN(get_logger(), "llm engine is not initialized");
    return;
  }

  const string user_text = msg->data;
  if (user_text.empty()) {
    return;
  }

  const LlmResult res = engine_->generate(user_text);
  if (!res.ok) {
    std_msgs::msg::String dbg;
    dbg.data = "llm_failed: " + res.error;
    pub_debug_->publish(dbg);
    RCLCPP_WARN(get_logger(), "llm failed: %s", res.error.c_str());
    return;
  }

  std_msgs::msg::String out;
  out.data = res.text;
  pub_response_->publish(out);

  std_msgs::msg::String provider;
  provider.data = res.provider;
  pub_provider_->publish(provider);

  std_msgs::msg::String dbg;
  dbg.data = string("provider=") + res.provider + (res.used_fallback ? ",fallback" : ",primary");
  pub_debug_->publish(dbg);
  RCLCPP_INFO(get_logger(), "llm provider=%s%s", res.provider.c_str(), res.used_fallback ? " (fallback)" : "");
}

}  // namespace llm_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<llm_cpp::LlmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
