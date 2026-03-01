#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ros_robot_controller_msgs/msg/audio_buffer.hpp"
#include "std_msgs/msg/string.hpp"

#include "stt_cpp/stt_engine.hpp"

namespace stt_cpp
{

class SttNode : public rclcpp::Node
{
public:
  SttNode();

private:
  void declare_and_get_parameters();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
  bool pass_runtime_thresholds(const SttResult & stt, double audio_seconds) const;
  int estimate_confidence(const std::string & text) const;
  void on_audio_path(const std_msgs::msg::String::SharedPtr msg);
  void on_audio_buffer(const ros_robot_controller_msgs::msg::AudioBuffer::SharedPtr msg);

  std::string groq_api_key_;
  std::string groq_model_;
  std::string groq_language_;
  int groq_timeout_sec_;
  std::string local_stt_script_path_;
  std::string local_stt_model_;
  std::string local_stt_language_;
  std::string local_stt_device_;
  std::string local_stt_compute_type_;
  std::string onnx_encoder_path_;
  std::string onnx_decoder_path_;
  bool onnx_use_cuda_;
  int onnx_intra_threads_;
  bool stt_enabled_;
  int confidence_threshold_;
  double seconds_per_order_;

  std::unique_ptr<SttEngine> stt_engine_;
  OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_audio_path_;
  rclcpp::Subscription<ros_robot_controller_msgs::msg::AudioBuffer>::SharedPtr sub_audio_buf_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_transcript_;
};

}  // namespace stt_cpp
