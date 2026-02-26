#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;

class LidarNode : public rclcpp::Node
{
public:
  /// 노드 초기화: 파라미터 선언/조회 후 LaserScan 구독 시작
  LidarNode()
  : Node("lidar_node")
  {
    declare_parameter<string>("scan_topic", "/scan");
    declare_parameter<int>("log_every_n", 10);

    scan_topic_ = get_parameter("scan_topic").as_string();
    log_every_n_ = max(1, static_cast<int>(get_parameter("log_every_n").as_int()));

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, bind(&LidarNode::on_scan, this, placeholders::_1));

    RCLCPP_INFO(get_logger(), "lidar_cpp node started (topic=%s, log_every_n=%d)",
      scan_topic_.c_str(), log_every_n_);
  }

private:
  /// LaserScan 수신 콜백: 유효 거리(min/max/center) 계산 후 주기적으로 로그 출력
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    ++scan_count_;

    float min_range = numeric_limits<float>::infinity();
    float max_range = 0.0f;
    int valid_count = 0;

    for (const float r : msg->ranges) {
      if (r < msg->range_min || r > msg->range_max) {
        continue;
      }
      min_range = min(min_range, r);
      max_range = max(max_range, r);
      ++valid_count;
    }

    float center_range = -1.0f;
    if (!msg->ranges.empty()) {
      const size_t center_idx = msg->ranges.size() / 2;
      center_range = msg->ranges[center_idx];
    }

    if (scan_count_ % log_every_n_ == 0) {
      if (valid_count == 0) {
        RCLCPP_WARN(get_logger(), "scan received but no valid ranges");
        return;
      }
      RCLCPP_INFO(
        get_logger(),
        "scan[%zu] valid=%d min=%.3f max=%.3f center=%.3f frame=%s",
        scan_count_, valid_count, min_range, max_range, center_range, msg->header.frame_id.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  string scan_topic_;
  size_t scan_count_ {0};
  int log_every_n_ {10};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
