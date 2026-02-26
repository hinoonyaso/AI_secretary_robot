#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std;


namespace imu_bridge_cpp
{

class ImuBridgeNode : public rclcpp::Node
{
public:
  ImuBridgeNode()
  : Node("imu_bridge_node"), msg_count_(0)
  {
    /// 토픽/QoS/셀프테스트/타임아웃 파라미터를 읽고 구독-중계-상태 타이머를 초기화
    input_topic_ = declare_parameter<string>("input_topic", "/ros_robot_controller/imu_raw");
    output_topic_ = declare_parameter<string>("output_topic", "/imu/data");
    frame_id_ = declare_parameter<string>("frame_id", "imu_link");
    timeout_sec_ = declare_parameter<double>("timeout_sec", 1.0);
    publisher_reliable_ = declare_parameter<bool>("publisher_reliable", false);
    self_test_ = declare_parameter<bool>("self_test", false);
    self_test_rate_hz_ = declare_parameter<double>("self_test_rate_hz", 30.0);

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, rclcpp::SensorDataQoS(),
      bind(&ImuBridgeNode::imu_callback, this, placeholders::_1));

    auto pub_qos = publisher_reliable_ ? rclcpp::QoS(rclcpp::KeepLast(50)).reliable()
                                       : rclcpp::SensorDataQoS();
    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, pub_qos);

    timer_ = create_wall_timer(
      chrono::seconds(1), bind(&ImuBridgeNode::timer_callback, this));

    if (self_test_) {
      if (self_test_rate_hz_ <= 0.0) {
        self_test_rate_hz_ = 30.0;
      }
      const auto period = chrono::duration_cast<chrono::milliseconds>(
        chrono::duration<double>(1.0 / self_test_rate_hz_));
      self_test_timer_ = create_wall_timer(
        period, bind(&ImuBridgeNode::self_test_callback, this));
    }

    last_report_time_ = now();
    last_msg_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    RCLCPP_INFO(get_logger(), "IMU bridge started");
    RCLCPP_INFO(get_logger(), "  input_topic : %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  frame_id    : %s", frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "  publisher_reliable: %s", publisher_reliable_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  self_test   : %s", self_test_ ? "true" : "false");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    process_imu(*msg);
  }

  void self_test_callback()
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.orientation.w = 1.0;

    const double t = now().seconds();
    msg.angular_velocity.x = 0.05 * sin(t);
    msg.angular_velocity.y = 0.03 * cos(t);
    msg.angular_velocity.z = 0.02;

    msg.linear_acceleration.x = 0.1 * sin(t * 0.5);
    msg.linear_acceleration.y = 0.1 * cos(t * 0.5);
    msg.linear_acceleration.z = 9.80665;

    process_imu(msg);
  }

  void process_imu(const sensor_msgs::msg::Imu & in)
  {
    /// frame_id/stamp 누락을 보정한 뒤 publish하고, 상태 모니터링용 통계를 갱신
    sensor_msgs::msg::Imu out = in;

    if (out.header.frame_id.empty()) {
      out.header.frame_id = frame_id_;
    }

    if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
      out.header.stamp = now();
    }

    pub_->publish(out);

    last_msg_time_ = now();
    msg_count_++;

    last_linear_acc_ = sqrt(
      out.linear_acceleration.x * out.linear_acceleration.x +
      out.linear_acceleration.y * out.linear_acceleration.y +
      out.linear_acceleration.z * out.linear_acceleration.z);

    last_gyro_ = sqrt(
      out.angular_velocity.x * out.angular_velocity.x +
      out.angular_velocity.y * out.angular_velocity.y +
      out.angular_velocity.z * out.angular_velocity.z);
  }

  void timer_callback()
  {
    /// 최근 1초 처리율과 마지막 수신 지연을 기준으로 정상/타임아웃 상태를 판정
    const auto current = now();

    if (last_report_time_.nanoseconds() == 0) {
      last_report_time_ = current;
      return;
    }

    const double dt = (current - last_report_time_).seconds();
    const double rate = (dt > 0.0) ? static_cast<double>(msg_count_) / dt : 0.0;

    if (last_msg_time_.nanoseconds() == 0) {
      RCLCPP_WARN(get_logger(), "No IMU message received yet from %s", input_topic_.c_str());
    } else {
      const double since_last = (current - last_msg_time_).seconds();
      if (since_last > timeout_sec_) {
        RCLCPP_WARN(
          get_logger(),
          "IMU timeout: last message %.3f sec ago (rate %.1f Hz)", since_last, rate);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "IMU OK: rate %.1f Hz, |acc|=%.3f m/s^2, |gyro|=%.3f rad/s",
          rate, last_linear_acc_, last_gyro_);
      }
    }

    msg_count_ = 0;
    last_report_time_ = current;
  }

  string input_topic_;
  string output_topic_;
  string frame_id_;
  double timeout_sec_;
  bool publisher_reliable_;
  bool self_test_;
  double self_test_rate_hz_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr self_test_timer_;

  rclcpp::Time last_msg_time_;
  rclcpp::Time last_report_time_;
  size_t msg_count_;
  double last_linear_acc_{0.0};
  double last_gyro_{0.0};
};

}  // namespace imu_bridge_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<imu_bridge_cpp::ImuBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
