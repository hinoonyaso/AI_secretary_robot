#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace intent_router_cpp
{

struct IntentRouteResult
{
  bool ok = false;
  std::string raw_json;
  std::string category;
  std::string command;
  std::string chat_text;
  bool needs_vision = false;
  std::string error;
};

class IntentRouterNode : public rclcpp::Node
{
public:
  IntentRouterNode();
  ~IntentRouterNode() override;

private:
  void declare_and_get_parameters();
  void on_transcript(const std_msgs::msg::String::SharedPtr msg);
  IntentRouteResult route_with_groq(const std::string & text) const;
  std::string build_prompt(const std::string & text) const;
  bool contains_any_keyword(const std::string & text, const std::vector<std::string> & keywords) const;
  void publish_string(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub,
    const std::string & data) const;
  static std::string remove_spaces(const std::string & value);

  std::vector<std::string> emergency_keywords_;
  bool groq_enabled_ = true;
  std::string groq_api_key_;
  std::string groq_model_;
  int groq_timeout_sec_ = 8;
  double groq_temperature_ = 0.1;
  int groq_max_tokens_ = 256;
  std::string groq_prompt_template_;
  std::string groq_json_schema_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_transcript_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_category_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_command_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_chat_text_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;
};

}  // namespace intent_router_cpp
