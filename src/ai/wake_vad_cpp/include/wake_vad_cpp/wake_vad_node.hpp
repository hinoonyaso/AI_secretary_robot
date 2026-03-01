#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ros_robot_controller_msgs/msg/audio_buffer.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "wake_vad_cpp/audio_input.hpp"
#include "wake_vad_cpp/porcupine_engine.hpp"
#include "wake_vad_cpp/state_machine.hpp"
#include "wake_vad_cpp/vad_engine.hpp"
#include "wake_vad_cpp/wav_writer.hpp"

namespace wake_vad_cpp
{

class WakeVadNode : public rclcpp::Node
{
public:
  WakeVadNode();
  ~WakeVadNode() override;

private:
  void declare_and_get_parameters();
  std::vector<std::string> keywords_from_mic_type(const std::string & mic_type) const;
  bool start_audio_input_with_fallback();
  bool update_keyword_runtime(const std::string & keyword_path, const std::string & wake_keyword);
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
  std::string resolve_keyword_path(const std::string & configured_path) const;
  std::string resolve_keyword_by_name(const std::string & keyword_dir, const std::string & wake_keyword) const;
  std::string resolve_model_path(const std::string & configured_path) const;
  void initialize_engines();
  void on_audio_frame(const AudioInput::Frame & frame);
  void processing_loop();
  void process_waiting(const AudioInput::Frame & frame);
  void process_listening(const AudioInput::Frame & frame);
  void process_recording(const AudioInput::Frame & frame);
  void on_tts_playback_done(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_state();
  void transition_to(WakeVadState next);
  void finalize_recording();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_detected_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_audio_path_;
  rclcpp::Publisher<ros_robot_controller_msgs::msg::AudioBuffer>::SharedPtr pub_audio_buf_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_wake_prompt_tts_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tts_playback_done_;

  StateMachine state_machine_;
  AudioInput audio_input_;
  PorcupineEngine porcupine_;
  VadEngine vad_;
  WavWriter wav_writer_;

  std::string access_key_;
  std::string keyword_path_;
  std::string porcupine_model_path_;
  int audio_device_index_;
  double sensitivity_;
  double vad_threshold_;
  double silence_duration_;
  double max_record_duration_;
  std::string wav_output_dir_;
  double listen_timeout_;
  std::string vad_model_path_;
  bool wake_prompt_tts_enabled_;
  std::string wake_prompt_tts_topic_;
  std::string wake_prompt_tts_text_;
  bool wake_prompt_wait_tts_done_enabled_;
  double wake_prompt_wait_timeout_sec_;
  std::string tts_playback_done_topic_;
  bool save_wav_for_debug_;
  std::string mic_type_;
  std::string audio_device_hint_;
  std::string wake_keyword_;
  std::string keyword_dir_path_;

  std::vector<int16_t> record_buffer_;
  std::queue<AudioInput::Frame> frame_queue_;
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::thread processing_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> waiting_wake_prompt_playback_done_;

  rclcpp::Time listen_start_;
  rclcpp::Time wake_prompt_wait_start_;
  rclcpp::Time record_start_;
  rclcpp::Time silence_start_;
  bool silence_tracking_;
  OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
};

}  // namespace wake_vad_cpp
