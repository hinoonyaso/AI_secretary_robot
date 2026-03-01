#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "tts_cpp/alsa_player.hpp"
#include "tts_cpp/tts_engine.hpp"

namespace tts_cpp
{

class TtsNode : public rclcpp::Node
{
public:
  TtsNode();

private:
  void declare_and_get_parameters();
  void initialize_audio_output_with_fallback();
  void on_text(const std_msgs::msg::String::SharedPtr msg);
  bool play_audio_file(const std::string & audio_path);
  bool run_playback_command(const std::string & command_template, const std::string & audio_path);

  std::string input_topic_ = "/llm/response";
  std::string output_audio_topic_ = "/tts/audio_path";
  std::string output_engine_topic_ = "/tts/engine";
  std::string output_debug_topic_ = "/tts/debug";
  std::string output_playback_done_topic_ = "/tts/playback_done";
  bool auto_play_ = true;
  std::string playback_command_ = "";
  std::string alsa_device_ = "default";
  std::vector<std::string> alsa_fallback_devices_;

  std::unique_ptr<TtsEngine> engine_;
  std::unique_ptr<AlsaPlayer> alsa_player_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_text_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_audio_path_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_engine_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_playback_done_;
};

}  // namespace tts_cpp
