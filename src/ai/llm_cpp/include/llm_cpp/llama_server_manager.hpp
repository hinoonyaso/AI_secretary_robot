#pragma once

#include <sys/types.h>

#include <string>

namespace llm_cpp
{

struct LlamaServerConfig
{
  std::string server_binary = "/home/ubuntu/external/llama.cpp/build/bin/llama-server";
  std::string model_path = "/home/ubuntu/models/qwen2.5-1.5b-instruct-q4_k_m.gguf";
  int port = 8081;
  int ngl = 99;
  int ctx_size = 2048;
  int n_threads = 4;
  bool use_mmap = true;
  bool use_mlock = false;
};

class LlamaServerManager
{
public:
  explicit LlamaServerManager(const LlamaServerConfig & cfg);
  ~LlamaServerManager();

  bool start();
  bool stop();
  bool restart();
  bool is_healthy() const;

  const std::string & endpoint() const { return endpoint_; }
  const std::string & last_error() const { return last_error_; }

private:
  bool wait_for_ready(int timeout_sec = 10) const;
  std::string build_command() const;
  bool is_process_alive() const;

  LlamaServerConfig config_;
  pid_t server_pid_ = -1;
  std::string endpoint_;
  mutable std::string last_error_;
};

}  // namespace llm_cpp
