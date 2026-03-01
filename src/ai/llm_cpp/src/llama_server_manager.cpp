#include "llm_cpp/llama_server_manager.hpp"

#include <rover_common/curl_utils.hpp>

#include <curl/curl.h>

#include <chrono>
#include <csignal>
#include <cerrno>
#include <sstream>
#include <string>
#include <thread>

#include <sys/wait.h>
#include <unistd.h>

using namespace std;

namespace llm_cpp
{
namespace
{

static const rover_common::CurlGlobalGuard curl_guard;

string shell_quote(const string & in)
{
  string out;
  out.reserve(in.size() + 2);
  out.push_back('\'');
  for (const char c : in) {
    if (c == '\'') {
      out += "'\"'\"'";
    } else {
      out.push_back(c);
    }
  }
  out.push_back('\'');
  return out;
}

}  // namespace

LlamaServerManager::LlamaServerManager(const LlamaServerConfig & cfg)
: config_(cfg)
{
  ostringstream oss;
  oss << "http://127.0.0.1:" << config_.port;
  endpoint_ = oss.str();
}

LlamaServerManager::~LlamaServerManager()
{
  stop();
}

bool LlamaServerManager::start()
{
  if (server_pid_ > 0 && is_process_alive()) {
    last_error_ = "server already running";
    return false;
  }

  const string cmd = build_command();

  server_pid_ = fork();
  if (server_pid_ < 0) {
    last_error_ = "failed to fork server process";
    return false;
  }

  if (server_pid_ == 0) {
    execl("/bin/sh", "sh", "-c", cmd.c_str(), nullptr);
    _exit(127);
  }

  if (!wait_for_ready(10)) {
    stop();
    last_error_ = "server failed to start within timeout";
    return false;
  }

  return true;
}

bool LlamaServerManager::stop()
{
  if (server_pid_ <= 0) {
    return true;
  }

  if (kill(server_pid_, SIGTERM) != 0 && errno != ESRCH) {
    last_error_ = "failed to send SIGTERM";
  }

  for (int i = 0; i < 50; ++i) {
    int status = 0;
    const pid_t waited = waitpid(server_pid_, &status, WNOHANG);
    if (waited == server_pid_) {
      server_pid_ = -1;
      return true;
    }
    if (waited < 0 && errno == ECHILD) {
      server_pid_ = -1;
      return true;
    }
    this_thread::sleep_for(chrono::milliseconds(100));
  }

  if (kill(server_pid_, SIGKILL) != 0 && errno != ESRCH) {
    last_error_ = "failed to send SIGKILL";
  }
  waitpid(server_pid_, nullptr, 0);
  server_pid_ = -1;
  return true;
}

bool LlamaServerManager::restart()
{
  stop();
  this_thread::sleep_for(chrono::milliseconds(300));
  return start();
}

bool LlamaServerManager::is_healthy() const
{
  if (!is_process_alive()) {
    return false;
  }

  CURL * curl = curl_easy_init();
  if (!curl) {
    return false;
  }

  string response;
  long http_code = 0;
  const string url = endpoint_ + "/health";

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, rover_common::curl_write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 1L);

  const CURLcode rc = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_easy_cleanup(curl);

  return rc == CURLE_OK && http_code == 200;
}

bool LlamaServerManager::wait_for_ready(int timeout_sec) const
{
  const auto deadline = chrono::steady_clock::now() + chrono::seconds(timeout_sec);
  while (chrono::steady_clock::now() < deadline) {
    if (is_healthy()) {
      return true;
    }
    this_thread::sleep_for(chrono::milliseconds(250));
  }
  return false;
}

string LlamaServerManager::build_command() const
{
  ostringstream cmd;
  cmd << shell_quote(config_.server_binary)
      << " --model " << shell_quote(config_.model_path)
      << " --port " << config_.port
      << " -ngl " << config_.ngl
      << " -c " << config_.ctx_size
      << " -t " << config_.n_threads;

  if (config_.use_mmap) {
    cmd << " --mmap";
  }
  if (config_.use_mlock) {
    cmd << " --mlock";
  }

  cmd << " >> " << shell_quote("/tmp/llama_server_" + to_string(config_.port) + ".log") << " 2>&1";
  return cmd.str();
}

bool LlamaServerManager::is_process_alive() const
{
  if (server_pid_ <= 0) {
    return false;
  }
  if (kill(server_pid_, 0) == 0) {
    return true;
  }
  if (errno == EPERM) {
    return true;
  }
  return false;
}

}  // namespace llm_cpp
