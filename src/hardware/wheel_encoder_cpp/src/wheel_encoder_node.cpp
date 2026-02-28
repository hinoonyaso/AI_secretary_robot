#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros_robot_controller_msgs/msg/motor_state.hpp"
#include "ros_robot_controller_msgs/msg/motors_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"

using namespace std;

class WheelEncoderNode : public rclcpp::Node
{
public:
  /// 모터 rps 명령을 기반으로 휠 엔코더 tick을 누적 계산해 주기 발행
  WheelEncoderNode()
  : Node("wheel_encoder_node")
  {
    declare_parameter<string>("input_motor_topic", "/ros_robot_controller/set_motor");
    declare_parameter<string>("ticks_topic", "~/ticks");
    declare_parameter<string>("velocity_topic", "~/velocity_rps");
    declare_parameter<string>("joint_state_topic", "~/joint_states");
    declare_parameter<vector<int64_t>>("motor_ids", {1, 2, 3, 4});
    declare_parameter<vector<string>>(
      "joint_names",
      {"wheel_left_front_joint", "wheel_left_back_joint", "wheel_right_front_joint", "wheel_right_back_joint"});
    declare_parameter<double>("ticks_per_revolution", 2048.0);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<double>("stale_timeout_sec", 0.5);

    input_motor_topic_ = get_parameter("input_motor_topic").as_string();
    ticks_topic_ = get_parameter("ticks_topic").as_string();
    velocity_topic_ = get_parameter("velocity_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    ticks_per_revolution_ = max(1.0, get_parameter("ticks_per_revolution").as_double());
    publish_rate_hz_ = max(1.0, get_parameter("publish_rate_hz").as_double());
    stale_timeout_sec_ = max(0.0, get_parameter("stale_timeout_sec").as_double());

    for (const auto v : get_parameter("motor_ids").as_integer_array()) {
      motor_ids_.push_back(static_cast<uint16_t>(v));
    }
    joint_names_ = get_parameter("joint_names").as_string_array();

    if (joint_names_.size() != motor_ids_.size()) {
      RCLCPP_WARN(
        get_logger(),
        "joint_names size(%zu) != motor_ids size(%zu), fallback to default wheel_i names",
        joint_names_.size(), motor_ids_.size());
      joint_names_.clear();
      for (size_t i = 0; i < motor_ids_.size(); ++i) {
        joint_names_.push_back("wheel_" + to_string(i + 1) + "_joint");
      }
    }

    for (const auto id : motor_ids_) {
      motor_rps_[id] = 0.0;
      motor_ticks_[id] = 0.0;
    }

    motor_sub_ = create_subscription<ros_robot_controller_msgs::msg::MotorsState>(
      input_motor_topic_, 20, bind(&WheelEncoderNode::on_motor_cmd, this, placeholders::_1));
    ticks_pub_ = create_publisher<std_msgs::msg::Int64MultiArray>(ticks_topic_, 20);
    velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(velocity_topic_, 20);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 20);

    last_update_time_ = now();
    last_cmd_time_ = now();

    const auto period_ms = static_cast<int>(round(1000.0 / publish_rate_hz_));
    timer_ = create_wall_timer(
      chrono::milliseconds(max(1, period_ms)),
      bind(&WheelEncoderNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "wheel_encoder_cpp started");
    RCLCPP_INFO(get_logger(), "  input_motor_topic : %s", input_motor_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  ticks_topic       : %s", ticks_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  velocity_topic    : %s", velocity_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  joint_state_topic : %s", joint_state_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  ticks_per_rev     : %.1f", ticks_per_revolution_);
    RCLCPP_INFO(get_logger(), "  publish_rate_hz   : %.1f", publish_rate_hz_);
  }

private:
  /// set_motor 명령에서 모터별 rps를 갱신
  void on_motor_cmd(const ros_robot_controller_msgs::msg::MotorsState::SharedPtr msg)
  {
    bool touched = false;
    for (const auto & m : msg->data) {
      const auto it = motor_rps_.find(m.id);
      if (it == motor_rps_.end()) {
        continue;
      }
      it->second = m.rps;
      touched = true;
    }

    if (touched) {
      last_cmd_time_ = now();
    }
  }

  /// 경과 시간(dt) 동안 rps를 적분해 tick/rad 위치를 계산하고 토픽 발행
  void on_timer()
  {
    const auto t = now();
    double dt = (t - last_update_time_).seconds();
    dt = max(0.0, dt);
    last_update_time_ = t;

    if (stale_timeout_sec_ > 0.0 && (t - last_cmd_time_).seconds() > stale_timeout_sec_) {
      for (auto & kv : motor_rps_) {
        kv.second = 0.0;
      }
    }

    std_msgs::msg::Int64MultiArray ticks_msg;
    std_msgs::msg::Float64MultiArray vel_msg;
    sensor_msgs::msg::JointState joint_msg;

    ticks_msg.data.reserve(motor_ids_.size());
    vel_msg.data.reserve(motor_ids_.size());
    joint_msg.name = joint_names_;
    joint_msg.position.reserve(motor_ids_.size());
    joint_msg.velocity.reserve(motor_ids_.size());
    joint_msg.header.stamp = t;

    for (const auto id : motor_ids_) {
      const double rps = motor_rps_[id];
      motor_ticks_[id] += rps * ticks_per_revolution_ * dt;

      const double ticks = motor_ticks_[id];
      const int64_t ticks_i64 = static_cast<int64_t>(llround(ticks));
      const double rad = (ticks / ticks_per_revolution_) * 2.0 * M_PI;
      const double vel_rad = rps * 2.0 * M_PI;

      ticks_msg.data.push_back(ticks_i64);
      vel_msg.data.push_back(rps);
      joint_msg.position.push_back(rad);
      joint_msg.velocity.push_back(vel_rad);
    }

    ticks_pub_->publish(ticks_msg);
    velocity_pub_->publish(vel_msg);
    joint_state_pub_->publish(joint_msg);
  }

  string input_motor_topic_;
  string ticks_topic_;
  string velocity_topic_;
  string joint_state_topic_;
  vector<uint16_t> motor_ids_;
  vector<string> joint_names_;

  double ticks_per_revolution_{2048.0};
  double publish_rate_hz_{50.0};
  double stale_timeout_sec_{0.5};

  unordered_map<uint16_t, double> motor_rps_;
  unordered_map<uint16_t, double> motor_ticks_;
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<ros_robot_controller_msgs::msg::MotorsState>::SharedPtr motor_sub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr ticks_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<WheelEncoderNode>());
  rclcpp::shutdown();
  return 0;
}
