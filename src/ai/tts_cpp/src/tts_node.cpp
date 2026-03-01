#include "tts_cpp/tts_node.hpp"

#include <rover_common/shell_utils.hpp>
#include <rover_common/string_utils.hpp>

#include <exception>
#include <filesystem>
#include <functional>
#include <sstream>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std;


namespace tts_cpp
{

TtsNode::TtsNode()
: Node("tts_node")
{
  declare_and_get_parameters();

  pub_audio_path_ = create_publisher<std_msgs::msg::String>(output_audio_topic_, 10);
  pub_engine_ = create_publisher<std_msgs::msg::String>(output_engine_topic_, 10);
  pub_debug_ = create_publisher<std_msgs::msg::String>(output_debug_topic_, 10);
  pub_playback_done_ = create_publisher<std_msgs::msg::Bool>(output_playback_done_topic_, 10);

  sub_text_ = create_subscription<std_msgs::msg::String>(
    input_topic_, 10, bind(&TtsNode::on_text, this, placeholders::_1));

  RCLCPP_INFO(get_logger(), "tts_cpp node started");
}

void TtsNode::declare_and_get_parameters()
{
  declare_parameter<string>("input_topic", "/llm/response");
  declare_parameter<string>("output_audio_topic", "/tts/audio_path");
  declare_parameter<string>("output_engine_topic", "/tts/engine");
  declare_parameter<string>("output_debug_topic", "/tts/debug");
  declare_parameter<string>("output_playback_done_topic", "/tts/playback_done");
  declare_parameter<bool>("auto_play", true);
  declare_parameter<string>("playback_command", "");
  declare_parameter<string>("alsa_device", "default");

  declare_parameter<bool>("tts_enabled", true);
  declare_parameter<string>("output_dir", "/tmp/tts_audio");

  declare_parameter<string>("edge_script_path", "");
  declare_parameter<string>("edge_voice", "ko-KR-SunHiNeural");
  declare_parameter<string>("edge_rate", "+0%");
  declare_parameter<string>("edge_volume", "+0%");

  declare_parameter<string>("piper_executable", "piper");
  declare_parameter<string>("piper_model_path", "");
  declare_parameter<string>("piper_config_path", "");
  declare_parameter<string>("piper_speaker", "");
  declare_parameter<string>("espeak_executable", "espeak-ng");
  declare_parameter<string>("espeak_voice", "ko");
  declare_parameter<string>("tts_engine", "auto");

  input_topic_ = get_parameter("input_topic").as_string();
  output_audio_topic_ = get_parameter("output_audio_topic").as_string();
  output_engine_topic_ = get_parameter("output_engine_topic").as_string();
  output_debug_topic_ = get_parameter("output_debug_topic").as_string();
  output_playback_done_topic_ = get_parameter("output_playback_done_topic").as_string();
  auto_play_ = get_parameter("auto_play").as_bool();
  playback_command_ = get_parameter("playback_command").as_string();
  alsa_device_ = get_parameter("alsa_device").as_string();

  TtsConfig cfg;
  cfg.enabled = get_parameter("tts_enabled").as_bool();
  cfg.output_dir = get_parameter("output_dir").as_string();
  cfg.edge_script_path = get_parameter("edge_script_path").as_string();
  cfg.edge_voice = get_parameter("edge_voice").as_string();
  cfg.edge_rate = get_parameter("edge_rate").as_string();
  cfg.edge_volume = get_parameter("edge_volume").as_string();
  cfg.piper_executable = get_parameter("piper_executable").as_string();
  cfg.piper_model_path = get_parameter("piper_model_path").as_string();
  cfg.piper_config_path = get_parameter("piper_config_path").as_string();
  cfg.piper_speaker = get_parameter("piper_speaker").as_string();
  cfg.espeak_executable = get_parameter("espeak_executable").as_string();
  cfg.espeak_voice = get_parameter("espeak_voice").as_string();
  cfg.engine = get_parameter("tts_engine").as_string();

  if (cfg.edge_script_path.empty()) {
    try {
      const string share_dir = ament_index_cpp::get_package_share_directory("tts_cpp");
      cfg.edge_script_path = (filesystem::path(share_dir) / "scripts" / "edge_tts_synth.py").string();
    } catch (const exception & e) {
      RCLCPP_WARN(get_logger(), "failed to resolve edge script path: %s", e.what());
    }
  }

  engine_ = unique_ptr<TtsEngine>(new TtsEngine(cfg));
  alsa_player_ = std::make_unique<AlsaPlayer>(alsa_device_);
}

void TtsNode::on_text(const std_msgs::msg::String::SharedPtr msg)
{
  /// 텍스트 입력 1건을 합성하고, 엔진/디버그/재생완료 상태를 함께 발행
  if (!engine_) {
    RCLCPP_WARN(get_logger(), "tts engine is not initialized");
    return;
  }
  const string text = msg->data;
  if (text.empty()) {
    return;
  }
  if (!rclcpp::ok()) {
    return;
  }

  const TtsResult res = engine_->synthesize(text);
  if (!rclcpp::ok()) {
    // SIGINT/shutdown during synthesis: avoid late playback on shutdown path
    return;
  }
  if (!res.ok) {
    std_msgs::msg::String dbg;
    dbg.data = "tts_failed: " + res.error;
    pub_debug_->publish(dbg);
    std_msgs::msg::Bool done_msg;
    done_msg.data = false;
    pub_playback_done_->publish(done_msg);
    RCLCPP_WARN(get_logger(), "tts failed: %s", res.error.c_str());
    return;
  }

  if (!res.audio_path.empty()) {
    std_msgs::msg::String path;
    path.data = res.audio_path;
    pub_audio_path_->publish(path);
  }

  std_msgs::msg::String engine_msg;
  engine_msg.data = res.engine;
  pub_engine_->publish(engine_msg);

  std_msgs::msg::String dbg;
  dbg.data = string("tts=") + res.engine + (res.used_fallback ? ",fallback" : ",primary");
  pub_debug_->publish(dbg);

  if (!rclcpp::ok()) {
    return;
  }

  if (auto_play_) {
    bool played = false;
    if (res.has_pcm() && alsa_player_ && alsa_player_->is_available()) {
      played = alsa_player_->play_pcm(res.pcm_samples, res.pcm_sample_rate, res.pcm_channels);
      if (!played) {
        RCLCPP_WARN(
          get_logger(), "alsa playback failed: %s",
          alsa_player_->last_error().c_str());
      }
    }
    if (!played && !res.audio_path.empty()) {
      played = play_audio_file(res.audio_path);
    }
    std_msgs::msg::Bool done_msg;
    done_msg.data = played;
    pub_playback_done_->publish(done_msg);
    if (!played) {
      std_msgs::msg::String play_dbg;
      play_dbg.data = "playback_failed";
      if (!res.audio_path.empty()) {
        play_dbg.data += ": " + res.audio_path;
      }
      pub_debug_->publish(play_dbg);
      if (!res.audio_path.empty()) {
        RCLCPP_WARN(get_logger(), "audio playback failed: %s", res.audio_path.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "audio playback failed: pcm buffer");
      }
    }
  } else {
    std_msgs::msg::Bool done_msg;
    done_msg.data = true;
    pub_playback_done_->publish(done_msg);
  }

  if (res.has_pcm()) {
    RCLCPP_INFO(
      get_logger(), "tts(%s%s): pcm %zu samples @ %u Hz",
      res.engine.c_str(), res.used_fallback ? ",fallback" : "",
      res.pcm_samples.size(), res.pcm_sample_rate);
  } else {
    RCLCPP_INFO(
      get_logger(), "tts(%s%s): %s", res.engine.c_str(),
      res.used_fallback ? ",fallback" : "", res.audio_path.c_str());
  }
}

bool TtsNode::play_audio_file(const string & audio_path)
{
  /// 확장자별 후보 플레이어를 순차 시도해 가장 먼저 성공한 재생 경로를 채택
  if (!playback_command_.empty()) {
    return run_playback_command(playback_command_, audio_path);
  }

  const string ext = filesystem::path(audio_path).extension().string();
  vector<string> commands;
  if (ext == ".wav") {
    commands = {
      "aplay -q {audio_path}",
      "ffplay -nodisp -autoexit -loglevel quiet {audio_path}",
    };
  } else if (ext == ".mp3") {
    commands = {
      "ffplay -nodisp -autoexit -loglevel quiet {audio_path}",
      "mpg123 -q {audio_path}",
    };
  } else {
    commands = {
      "ffplay -nodisp -autoexit -loglevel quiet {audio_path}",
      "aplay -q {audio_path}",
      "mpg123 -q {audio_path}",
    };
  }

  for (const auto & cmd : commands) {
    if (run_playback_command(cmd, audio_path)) {
      return true;
    }
  }
  return false;
}

bool TtsNode::run_playback_command(
  const string & command_template,
  const string & audio_path)
{
  /// 템플릿의 {audio_path}를 안전하게 치환하고 stderr까지 수집해 실패 원인 로깅
  string command = command_template;
  const string placeholder = "{audio_path}";
  const string escaped = rover_common::shell_escape_single_quote(audio_path);
  size_t pos = 0;
  while ((pos = command.find(placeholder, pos)) != string::npos) {
    command.replace(pos, placeholder.size(), escaped);
    pos += escaped.size();
  }
  if (command.find(placeholder) == string::npos && command.find(escaped) == string::npos) {
    command += " " + escaped;
  }
  command += " 2>&1";

  const rover_common::ShellResult shell = rover_common::run_shell_command(command);
  if (shell.ok) {
    return true;
  }

  RCLCPP_WARN(
    get_logger(), "playback command failed(exit=%d): %s | %s",
    shell.exit_code, command_template.c_str(),
    rover_common::trim(shell.output).c_str());
  return false;
}

}  // namespace tts_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<tts_cpp::TtsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
