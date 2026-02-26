#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "ros_robot_controller_msgs/msg/servo_position.hpp"
#include "ros_robot_controller_msgs/msg/servos_position.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;


class UdpServoBridgeNode : public rclcpp::Node {
 public:
  UdpServoBridgeNode()
      : Node("udp_servo_bridge_node"), socket_fd_(-1) {
    udp_host_ = declare_parameter<string>("udp_host", "0.0.0.0");
    udp_port_ = declare_parameter<int>("udp_port", 9999);
    output_topic_ = declare_parameter<string>(
        "output_topic", "/ros_robot_controller/bus_servo/set_position");
    // UI 기준 축별 미세 보정 tick (ui servo 1~6)
    servo_trim_ticks_ = declare_parameter<vector<int64_t>>(
        "servo_trim_ticks", vector<int64_t>{0, 0, 0, 0, 0, 0});
    // UI 축 번호를 실제 버스 서보 ID로 매핑
    servo_id_map_ = declare_parameter<vector<int64_t>>(
        "servo_id_map", vector<int64_t>{1, 2, 3, 4, 5, 10});

    publisher_ = create_publisher<ros_robot_controller_msgs::msg::ServosPosition>(
        output_topic_, 50);

    configure_socket();

    timer_ = create_wall_timer(
        chrono::milliseconds(10),
        bind(&UdpServoBridgeNode::poll_udp, this));

    RCLCPP_INFO(get_logger(), "Listening UDP %s:%d -> %s",
                udp_host_.c_str(), udp_port_, output_topic_.c_str());
  }

  ~UdpServoBridgeNode() override {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

 private:
  void configure_socket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw runtime_error("Failed to create UDP socket");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(udp_port_));

    if (inet_pton(AF_INET, udp_host_.c_str(), &addr.sin_addr) != 1) {
      throw runtime_error("Invalid udp_host IP address");
    }

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      const string reason = strerror(errno);
      throw runtime_error("Failed to bind UDP socket: " + reason);
    }
  }

  static bool parse_command(
      const string& payload,
      int& servo_id,
      double& value,
      int& duration_ms) {
    static const regex servo_id_regex(R"("servo_id"\s*:\s*(-?\d+))");
    static const regex value_regex(R"("value"\s*:\s*(-?\d+(?:\.\d+)?))");
    static const regex duration_regex(R"("duration_ms"\s*:\s*(-?\d+))");

    smatch match;
    if (!regex_search(payload, match, servo_id_regex)) {
      return false;
    }
    servo_id = stoi(match[1].str());

    if (!regex_search(payload, match, value_regex)) {
      return false;
    }
    value = stod(match[1].str());

    duration_ms = 300;
    if (regex_search(payload, match, duration_regex)) {
      duration_ms = stoi(match[1].str());
    }

    return true;
  }

  static bool get_ui_range_for_servo(int servo_id, double& ui_min, double& ui_max) {
    // JetRover 6DOF arm UI spec: all joints are controlled as 0~240 deg.
    if (servo_id < 1 || servo_id > 6) {
      return false;
    }
    ui_min = 0.0;
    ui_max = 240.0;
    return true;
  }

  int convert_ui_value_to_servo_position(int ui_servo_id, double ui_value) const {
    double ui_min = 0.0;
    double ui_max = 0.0;
    if (!get_ui_range_for_servo(ui_servo_id, ui_min, ui_max) || ui_max <= ui_min) {
      return clamp(static_cast<int>(lround(ui_value)), 0, 1000);
    }

    const double clamped_ui = clamp(ui_value, ui_min, ui_max);
    const double ratio = (clamped_ui - ui_min) / (ui_max - ui_min);
    // Convert UI angle to bus-servo pulse range (0~1000).
    int position = static_cast<int>(lround(ratio * 1000.0));

    if (ui_servo_id >= 1 && ui_servo_id <= static_cast<int>(servo_trim_ticks_.size())) {
      position += static_cast<int>(servo_trim_ticks_[static_cast<size_t>(ui_servo_id - 1)]);
    }

    return clamp(position, 0, 1000);
  }

  int map_ui_servo_id_to_bus_id(int ui_servo_id) const {
    // Resolve ui servo index -> hardware bus servo id.
    if (ui_servo_id >= 1 && ui_servo_id <= static_cast<int>(servo_id_map_.size())) {
      return clamp(
          static_cast<int>(servo_id_map_[static_cast<size_t>(ui_servo_id - 1)]), 0, 255);
    }
    return clamp(ui_servo_id, 0, 255);
  }

  void poll_udp() {
    char buffer[512];

    sockaddr_in source_addr{};
    socklen_t source_len = sizeof(source_addr);
    const int recv_len = recvfrom(
        socket_fd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
        reinterpret_cast<sockaddr*>(&source_addr), &source_len);

    if (recv_len < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "recvfrom failed: %s", strerror(errno));
      return;
    }

    buffer[recv_len] = '\0';
    const string payload(buffer);

    int servo_id = 0;
    int duration_ms = 300;
    double value = 0.0;
    if (!parse_command(payload, servo_id, value, duration_ms)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Invalid payload: %s", payload.c_str());
      return;
    }

    // Use UI joint index for conversion, then remap to actual bus id.
    const int clamped_ui_servo_id = clamp(servo_id, 1, 6);
    const int clamped_servo_id = map_ui_servo_id_to_bus_id(clamped_ui_servo_id);
    const int clamped_position = convert_ui_value_to_servo_position(clamped_ui_servo_id, value);
    const int clamped_duration_ms = max(duration_ms, 0);

    ros_robot_controller_msgs::msg::ServosPosition msg;
    msg.duration = static_cast<double>(clamped_duration_ms) / 1000.0;

    ros_robot_controller_msgs::msg::ServoPosition servo_position;
    servo_position.id = static_cast<uint16_t>(clamped_servo_id);
    servo_position.position = static_cast<uint16_t>(clamped_position);
    msg.position.push_back(servo_position);
    publisher_->publish(msg);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Published bus servo cmd id=%d position=%d duration=%dms",
        clamped_servo_id, clamped_position, clamped_duration_ms);
  }

  string udp_host_;
  int udp_port_;
  string output_topic_;
  vector<int64_t> servo_trim_ticks_;
  vector<int64_t> servo_id_map_;

  int socket_fd_;
  rclcpp::Publisher<ros_robot_controller_msgs::msg::ServosPosition>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = make_shared<UdpServoBridgeNode>();
    rclcpp::spin(node);
  } catch (const exception& e) {
    fprintf(stderr, "Failed to start node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
