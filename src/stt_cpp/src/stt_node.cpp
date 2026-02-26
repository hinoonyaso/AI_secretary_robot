#include "stt_cpp/stt_node.hpp"

#include <cstdlib>
#include <functional>

using namespace std;


namespace stt_cpp
{

SttNode::SttNode()
: Node("stt_node")
{
  declare_and_get_parameters();

  pub_transcript_ = create_publisher<std_msgs::msg::String>("/wake_vad/transcript", 10);
  sub_audio_path_ = create_subscription<std_msgs::msg::String>(
    "/wake_vad/audio_path", 10,
    bind(&SttNode::on_audio_path, this, placeholders::_1));
  sub_audio_buf_ = create_subscription<ros_robot_controller_msgs::msg::AudioBuffer>(
    "/wake_vad/audio_buffer", 10,
    bind(&SttNode::on_audio_buffer, this, placeholders::_1));

  RCLCPP_INFO(get_logger(), "stt_cpp node started");
}

void SttNode::declare_and_get_parameters()
{
  declare_parameter<bool>("stt_enabled", true);
  declare_parameter<string>("groq_api_key", "");
  declare_parameter<string>("groq_model", "whisper-large-v3-turbo");
  declare_parameter<string>("groq_language", "ko");
  declare_parameter<int>("groq_timeout_sec", 30);

  SttConfig stt_cfg;
  stt_cfg.enabled       = get_parameter("stt_enabled").as_bool();
  stt_cfg.groq_api_key  = get_parameter("groq_api_key").as_string();
  stt_cfg.groq_model    = get_parameter("groq_model").as_string();
  stt_cfg.groq_language = get_parameter("groq_language").as_string();
  stt_cfg.groq_timeout_sec = get_parameter("groq_timeout_sec").as_int();

  if (stt_cfg.groq_api_key.empty()) {
    const char * env_key = getenv("GROQ_API_KEY");
    if (env_key) {
      stt_cfg.groq_api_key = env_key;
    }
  }

  stt_engine_ = unique_ptr<SttEngine>(new SttEngine(stt_cfg));
}

void SttNode::on_audio_path(const std_msgs::msg::String::SharedPtr msg)
{
  if (!stt_engine_) {
    RCLCPP_WARN(get_logger(), "stt engine is not initialized");
    return;
  }

  const string wav_path = msg->data;
  if (wav_path.empty()) {
    RCLCPP_WARN(get_logger(), "received empty audio path");
    return;
  }

  const SttResult stt = stt_engine_->transcribe(wav_path);
  if (!stt.ok) {
    RCLCPP_WARN(get_logger(), "stt failed for %s: %s", wav_path.c_str(), stt.error.c_str());
    return;
  }

  std_msgs::msg::String text_msg;
  text_msg.data = stt.text;
  pub_transcript_->publish(text_msg);
  RCLCPP_INFO(get_logger(), "stt(groq): %s", stt.text.c_str());
}

void SttNode::on_audio_buffer(const ros_robot_controller_msgs::msg::AudioBuffer::SharedPtr msg)
{
  if (!stt_engine_) {
    RCLCPP_WARN(get_logger(), "stt engine is not initialized");
    return;
  }
  if (msg->samples.empty()) {
    RCLCPP_WARN(get_logger(), "received empty audio buffer");
    return;
  }

  const SttResult stt = stt_engine_->transcribe_pcm(msg->samples, msg->sample_rate);
  if (!stt.ok) {
    RCLCPP_WARN(get_logger(), "stt failed for audio buffer: %s", stt.error.c_str());
    return;
  }

  std_msgs::msg::String text_msg;
  text_msg.data = stt.text;
  pub_transcript_->publish(text_msg);
  RCLCPP_INFO(get_logger(), "stt(groq): %s", stt.text.c_str());
}

}  // namespace stt_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<stt_cpp::SttNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
