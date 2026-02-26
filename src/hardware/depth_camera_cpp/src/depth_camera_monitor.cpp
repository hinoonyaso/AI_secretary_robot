#include <chrono>
#include <cstddef>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;


namespace depth_camera_cpp
{

class DepthCameraMonitor : public rclcpp::Node
{
public:
  DepthCameraMonitor()
  : Node("depth_camera_monitor")
  {
    /// 카메라 이름으로 RGB/Depth 토픽을 조합하고, 주기 리포터 타이머를 구성
    camera_name_ = declare_parameter<string>("camera_name", "depth_cam");
    report_interval_sec_ = declare_parameter<double>("report_interval_sec", 1.0);

    const auto rgb_topic = "/" + camera_name_ + "/rgb/image_raw";
    const auto depth_topic = "/" + camera_name_ + "/depth/image_raw";

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_rgb(msg); });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_depth(msg); });

    timer_ = create_wall_timer(
      chrono::duration_cast<chrono::milliseconds>(
        chrono::duration<double>(report_interval_sec_)),
      bind(&DepthCameraMonitor::report, this));

    last_report_time_ = now();
    RCLCPP_INFO(get_logger(), "Depth camera monitor started");
    RCLCPP_INFO(get_logger(), "  camera_name: %s", camera_name_.c_str());
    RCLCPP_INFO(get_logger(), "  rgb_topic  : %s", rgb_topic.c_str());
    RCLCPP_INFO(get_logger(), "  depth_topic: %s", depth_topic.c_str());
  }

private:
  void on_rgb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    (void)msg;
    rgb_count_++;
    last_rgb_time_ = now();
  }

  void on_depth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    (void)msg;
    depth_count_++;
    last_depth_time_ = now();
  }

  void report()
  {
    /// 리포트 주기 동안의 수신 프레임 수를 Hz로 환산하고, 마지막 프레임 지연을 함께 출력
    const auto t = now();
    const auto dt = (t - last_report_time_).seconds();
    const auto rgb_hz = dt > 0.0 ? static_cast<double>(rgb_count_) / dt : 0.0;
    const auto depth_hz = dt > 0.0 ? static_cast<double>(depth_count_) / dt : 0.0;

    const auto since_rgb = last_rgb_time_.nanoseconds() > 0 ? (t - last_rgb_time_).seconds() : -1.0;
    const auto since_depth = last_depth_time_.nanoseconds() > 0 ? (t - last_depth_time_).seconds() : -1.0;

    if (last_rgb_time_.nanoseconds() == 0 || last_depth_time_.nanoseconds() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Waiting for frames: rgb_seen=%s depth_seen=%s",
        last_rgb_time_.nanoseconds() == 0 ? "false" : "true",
        last_depth_time_.nanoseconds() == 0 ? "false" : "true");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "RGB %.1f Hz (last %.2fs), DEPTH %.1f Hz (last %.2fs)",
        rgb_hz, since_rgb, depth_hz, since_depth);
    }

    rgb_count_ = 0;
    depth_count_ = 0;
    last_report_time_ = t;
  }

  string camera_name_;
  double report_interval_sec_{1.0};
  size_t rgb_count_{0};
  size_t depth_count_{0};
  rclcpp::Time last_rgb_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_depth_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_report_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace depth_camera_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<depth_camera_cpp::DepthCameraMonitor>());
  rclcpp::shutdown();
  return 0;
}
