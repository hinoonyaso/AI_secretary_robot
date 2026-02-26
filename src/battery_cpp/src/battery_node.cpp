#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/u_int16.hpp"

using namespace std;


class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode()
  : Node("battery_node")
  {
    declare_parameter<string>("input_battery_topic", "/ros_robot_controller/battery");
    declare_parameter<string>("output_state_topic", "~/state");
    declare_parameter<string>("output_voltage_topic", "~/voltage_mv");
    declare_parameter<double>("full_voltage", 8.4);   // 2S LiPo full
    declare_parameter<double>("empty_voltage", 6.4);  // 2S LiPo safe-empty
    declare_parameter<double>("stale_timeout_sec", 2.0);

    input_battery_topic_ = get_parameter("input_battery_topic").as_string();
    output_state_topic_ = get_parameter("output_state_topic").as_string();
    output_voltage_topic_ = get_parameter("output_voltage_topic").as_string();
    full_voltage_ = get_parameter("full_voltage").as_double();
    empty_voltage_ = get_parameter("empty_voltage").as_double();
    stale_timeout_sec_ = get_parameter("stale_timeout_sec").as_double();

    if (full_voltage_ <= empty_voltage_) {
      RCLCPP_WARN(
        get_logger(),
        "full_voltage(%.2f) must be greater than empty_voltage(%.2f), fallback to defaults",
        full_voltage_, empty_voltage_);
      full_voltage_ = 8.4;
      empty_voltage_ = 6.4;
    }

    battery_sub_ = create_subscription<std_msgs::msg::UInt16>(
      input_battery_topic_, 10, bind(&BatteryNode::on_battery, this, placeholders::_1));
    state_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(output_state_topic_, 10);
    voltage_pub_ = create_publisher<std_msgs::msg::UInt16>(output_voltage_topic_, 10);

    stale_timer_ = create_wall_timer(
      chrono::milliseconds(200), bind(&BatteryNode::publish_stale_if_needed, this));

    RCLCPP_INFO(get_logger(), "battery topic in : %s", input_battery_topic_.c_str());
    RCLCPP_INFO(get_logger(), "battery topic out: %s, %s", output_state_topic_.c_str(), output_voltage_topic_.c_str());
  }

private:
  /// 배터리 전압(mV)을 선형 스케일로 0~1 퍼센트로 환산
  double voltage_to_percentage(double voltage) const
  {
    const double ratio = (voltage - empty_voltage_) / (full_voltage_ - empty_voltage_);
    return clamp(ratio, 0.0, 1.0);
  }

  /// 배터리 입력 수신 시 BatteryState/원본 전압 토픽을 즉시 발행
  void on_battery(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    last_mv_ = msg->data;
    last_rx_time_ = now();
    has_data_ = true;

    const double voltage = static_cast<double>(msg->data) / 1000.0;
    const double percentage = voltage_to_percentage(voltage);

    sensor_msgs::msg::BatteryState state;
    state.header.stamp = now();
    state.voltage = static_cast<float>(voltage);
    state.current = numeric_limits<float>::quiet_NaN();
    state.charge = numeric_limits<float>::quiet_NaN();
    state.capacity = numeric_limits<float>::quiet_NaN();
    state.design_capacity = numeric_limits<float>::quiet_NaN();
    state.percentage = static_cast<float>(percentage);
    state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    state.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    state.present = true;
    state.location = "main_pack";
    state.serial_number = "";
    state_pub_->publish(state);

    std_msgs::msg::UInt16 raw = *msg;
    voltage_pub_->publish(raw);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "battery: %u mV (%.2f V, %.1f%%)",
      msg->data, voltage, percentage * 100.0);
  }

  /// 최근 데이터가 오래 끊기면 present=false 상태를 주기적으로 알림
  void publish_stale_if_needed()
  {
    if (!has_data_) {
      return;
    }
    if (stale_timeout_sec_ <= 0.0) {
      return;
    }
    const double age = (now() - last_rx_time_).seconds();
    if (age <= stale_timeout_sec_) {
      return;
    }

    sensor_msgs::msg::BatteryState stale;
    stale.header.stamp = now();
    stale.present = false;
    stale.voltage = static_cast<float>(static_cast<double>(last_mv_) / 1000.0);
    stale.percentage = static_cast<float>(voltage_to_percentage(stale.voltage));
    stale.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    stale.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    stale.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    stale.current = numeric_limits<float>::quiet_NaN();
    stale.charge = numeric_limits<float>::quiet_NaN();
    stale.capacity = numeric_limits<float>::quiet_NaN();
    stale.design_capacity = numeric_limits<float>::quiet_NaN();
    stale.location = "main_pack";
    stale_pub_once_per_timeout_(stale);
  }

  /// stale 상태는 경고 스팸 방지를 위해 timeout 구간당 1회만 발행
  void stale_pub_once_per_timeout_(const sensor_msgs::msg::BatteryState & msg)
  {
    const auto now_time = now();
    if ((now_time - last_stale_pub_time_).seconds() < stale_timeout_sec_) {
      return;
    }
    state_pub_->publish(msg);
    last_stale_pub_time_ = now_time;
    RCLCPP_WARN(get_logger(), "battery input stale for %.2fs (last=%u mV)", stale_timeout_sec_, last_mv_);
  }

  string input_battery_topic_;
  string output_state_topic_;
  string output_voltage_topic_;
  double full_voltage_{8.4};
  double empty_voltage_{6.4};
  double stale_timeout_sec_{2.0};

  bool has_data_{false};
  uint16_t last_mv_{0};
  rclcpp::Time last_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stale_pub_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr battery_sub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr voltage_pub_;
  rclcpp::TimerBase::SharedPtr stale_timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<BatteryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
