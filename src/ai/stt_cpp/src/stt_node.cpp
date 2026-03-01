#include "stt_cpp/stt_node.hpp"

#include <algorithm>
#include <cctype>
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
  parameter_cb_handle_ = add_on_set_parameters_callback(
    bind(&SttNode::on_set_parameters, this, placeholders::_1));

  RCLCPP_INFO(get_logger(), "stt_cpp node started");
}

void SttNode::declare_and_get_parameters()
{
  declare_parameter<bool>("stt_enabled", true);
  // "auto" = moonshine_onnx → groq fallback
  // "moonshine_onnx" = local only (offline mode)
  // "groq" = cloud only (hybrid mode)
  declare_parameter<string>("stt_engine", "auto");

  // Groq Whisper API
  declare_parameter<string>("groq_api_key", "");
  declare_parameter<string>("groq_model", "whisper-large-v3-turbo");
  declare_parameter<string>("groq_language", "ko");
  declare_parameter<int>("groq_timeout_sec", 30);

  // Moonshine ONNX (local)
  declare_parameter<string>("onnx_encoder_path", "");
  declare_parameter<string>("onnx_decoder_path", "");
  declare_parameter<bool>("onnx_use_cuda", true);
  declare_parameter<int>("onnx_gpu_device_id", 0);
  declare_parameter<int>("onnx_intra_threads", 2);
  declare_parameter<int>("confidence_threshold", 0);
  declare_parameter<double>("seconds_per_order", 10.0);

  SttConfig stt_cfg;
  stt_cfg.enabled       = get_parameter("stt_enabled").as_bool();
  stt_cfg.engine        = get_parameter("stt_engine").as_string();

  stt_cfg.groq_api_key  = get_parameter("groq_api_key").as_string();
  stt_cfg.groq_model    = get_parameter("groq_model").as_string();
  stt_cfg.groq_language = get_parameter("groq_language").as_string();
  stt_cfg.groq_timeout_sec = get_parameter("groq_timeout_sec").as_int();

  stt_cfg.onnx_encoder_path = get_parameter("onnx_encoder_path").as_string();
  stt_cfg.onnx_decoder_path = get_parameter("onnx_decoder_path").as_string();
  stt_cfg.onnx_use_cuda     = get_parameter("onnx_use_cuda").as_bool();
  stt_cfg.onnx_gpu_device_id = static_cast<int>(get_parameter("onnx_gpu_device_id").as_int());
  stt_cfg.onnx_intra_threads = static_cast<int>(get_parameter("onnx_intra_threads").as_int());
  confidence_threshold_ = static_cast<int>(get_parameter("confidence_threshold").as_int());
  seconds_per_order_ = get_parameter("seconds_per_order").as_double();

  if (stt_cfg.groq_api_key.empty()) {
    const char * env_key = getenv("GROQ_API_KEY");
    if (env_key) {
      stt_cfg.groq_api_key = env_key;
    }
  }

  RCLCPP_INFO(get_logger(), "stt engine mode: %s", stt_cfg.engine.c_str());
  stt_engine_ = unique_ptr<SttEngine>(new SttEngine(stt_cfg));
}

rcl_interfaces::msg::SetParametersResult SttNode::on_set_parameters(
  const vector<rclcpp::Parameter> & parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  result.reason = "ok";

  for (const auto & p : parameters) {
    if (p.get_name() == "confidence_threshold" &&
      p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      const int value = static_cast<int>(p.as_int());
      if (value < 0 || value > 100) {
        result.successful = false;
        result.reason = "confidence_threshold must be 0..100";
        return result;
      }
      confidence_threshold_ = value;
    } else if (p.get_name() == "seconds_per_order" &&
      (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ||
      p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER))
    {
      const double value = (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) ?
        p.as_double() : static_cast<double>(p.as_int());
      if (value <= 0.1) {
        result.successful = false;
        result.reason = "seconds_per_order must be > 0.1";
        return result;
      }
      seconds_per_order_ = value;
    }
  }

  return result;
}

int SttNode::estimate_confidence(const string & text) const
{
  size_t non_space = 0;
  for (const char c : text) {
    if (!isspace(static_cast<unsigned char>(c))) {
      ++non_space;
    }
  }
  if (non_space == 0) {
    return 0;
  }
  const int score = static_cast<int>(min<size_t>(100, non_space * 8));
  return score;
}

bool SttNode::pass_runtime_thresholds(const SttResult & stt, double audio_seconds) const
{
  if (!stt.ok) {
    return false;
  }
  if (seconds_per_order_ > 0.1 && audio_seconds > seconds_per_order_) {
    RCLCPP_WARN(
      get_logger(), "drop transcript: audio duration %.2fs > seconds_per_order %.2fs",
      audio_seconds, seconds_per_order_);
    return false;
  }
  if (confidence_threshold_ > 0) {
    const int confidence = estimate_confidence(stt.text);
    if (confidence < confidence_threshold_) {
      RCLCPP_WARN(
        get_logger(), "drop transcript: confidence %d < threshold %d",
        confidence, confidence_threshold_);
      return false;
    }
  }
  return true;
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
  if (!pass_runtime_thresholds(stt, 0.0)) {
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
  const double audio_seconds = static_cast<double>(msg->samples.size()) /
    max(1.0, static_cast<double>(msg->sample_rate));
  if (!pass_runtime_thresholds(stt, audio_seconds)) {
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
