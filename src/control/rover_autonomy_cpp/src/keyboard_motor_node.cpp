#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_robot_controller_msgs/msg/motor_state.hpp"
#include "ros_robot_controller_msgs/msg/motors_state.hpp"

using namespace std;

class KeyboardMotorNode : public rclcpp::Node
{
public:
  KeyboardMotorNode()
  : Node("keyboard_motor_node")
  {
    declare_parameter<string>("output_mode", "motor");  // motor | cmd_vel
    declare_parameter<string>("motor_topic", "/ros_robot_controller/set_motor");
    declare_parameter<string>("cmd_vel_topic", "/controller/cmd_vel");

    declare_parameter<double>("forward_rps", 2.0);
    declare_parameter<double>("turn_rps", 1.6);
    declare_parameter<vector<int64_t>>("left_motor_ids", {1, 3});
    declare_parameter<vector<int64_t>>("right_motor_ids", {2, 4});
    declare_parameter<double>("left_forward_sign", 1.0);
    declare_parameter<double>("right_forward_sign", 1.0);

    declare_parameter<double>("linear_vel", 0.2);
    declare_parameter<double>("angular_vel", 0.5);

    declare_parameter<double>("idle_timeout_sec", 2.0);

    output_mode_ = get_parameter("output_mode").as_string();
    motor_topic_ = get_parameter("motor_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();

    forward_rps_ = get_parameter("forward_rps").as_double();
    turn_rps_ = get_parameter("turn_rps").as_double();
    left_forward_sign_ = get_parameter("left_forward_sign").as_double();
    right_forward_sign_ = get_parameter("right_forward_sign").as_double();

    linear_vel_ = get_parameter("linear_vel").as_double();
    angular_vel_ = get_parameter("angular_vel").as_double();

    idle_timeout_sec_ = get_parameter("idle_timeout_sec").as_double();

    for (const auto v : get_parameter("left_motor_ids").as_integer_array()) {
      left_motor_ids_.push_back(static_cast<uint16_t>(v));
    }
    for (const auto v : get_parameter("right_motor_ids").as_integer_array()) {
      right_motor_ids_.push_back(static_cast<uint16_t>(v));
    }

    motor_pub_ = create_publisher<ros_robot_controller_msgs::msg::MotorsState>(motor_topic_, 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    setup_terminal_raw_mode();
    last_key_time_ = now();

    print_help_once();
    RCLCPP_INFO(get_logger(), "output_mode: %s", output_mode_.c_str());
    RCLCPP_INFO(get_logger(), "motor topic: %s", motor_topic_.c_str());
    RCLCPP_INFO(get_logger(), "cmd_vel topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "idle timeout: %.2f sec (<=0 disables auto-stop)", idle_timeout_sec_);

    timer_ = create_wall_timer(chrono::milliseconds(50), bind(&KeyboardMotorNode::tick, this));
  }

  ~KeyboardMotorNode() override
  {
    publish_stop();
    restore_terminal_mode();
  }

private:
  enum class Mode
  {
    STOP,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
  };

  /// 키보드 입력을 논블로킹으로 읽기 위해 터미널을 raw 모드로 전환
  void setup_terminal_raw_mode()
  {
    input_fd_ = STDIN_FILENO;
    if (tcgetattr(input_fd_, &term_orig_) == -1) {
      input_fd_ = open("/dev/tty", O_RDWR | O_NONBLOCK);
      if (input_fd_ < 0 || tcgetattr(input_fd_, &term_orig_) == -1) {
        RCLCPP_WARN(get_logger(), "tcgetattr failed (stdin,/dev/tty), keyboard input may not work");
        term_ready_ = false;
        return;
      }
    }

    termios raw = term_orig_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(input_fd_, TCSANOW, &raw) == -1) {
      RCLCPP_WARN(get_logger(), "tcsetattr raw mode failed, keyboard input may not work");
      term_ready_ = false;
      return;
    }

    const int flags = fcntl(input_fd_, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(input_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    term_ready_ = true;
  }

  /// 종료 시 터미널 속성을 원복
  void restore_terminal_mode()
  {
    if (term_ready_) {
      tcsetattr(input_fd_, TCSANOW, &term_orig_);
    }
    if (input_fd_ >= 0 && input_fd_ != STDIN_FILENO) {
      close(input_fd_);
      input_fd_ = -1;
    }
  }

  /// 조작 키 안내 출력
  void print_help_once()
  {
    RCLCPP_INFO(get_logger(), "keyboard control: w=forward, s=backward, a=left, d=right, x/space=stop");
  }

  /// 타이머 루프: 키 입력 반영 + timeout 정지 + 명령 발행
  void tick()
  {
    if (input_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no keyboard input fd available (stdin and /dev/tty unavailable)");
    }

    handle_keyboard_input();

    const auto now_time = now();
    if (idle_timeout_sec_ > 0.0 && (now_time - last_key_time_).seconds() > idle_timeout_sec_) {
      mode_ = Mode::STOP;
    }

    publish_current_mode();
  }

  /// 입력 가능한 키가 있으면 모두 소비하고 마지막 유효 입력으로 모드 갱신
  void handle_keyboard_input()
  {
    char c = 0;
    bool got_key = false;
    bool mode_changed = false;

    while (input_fd_ >= 0 && read(input_fd_, &c, 1) > 0) {
      got_key = true;
      const Mode prev = mode_;
      switch (c) {
        case 'w':
        case 'W':
          mode_ = Mode::FORWARD;
          break;
        case 's':
        case 'S':
          mode_ = Mode::BACKWARD;
          break;
        case 'a':
        case 'A':
          mode_ = Mode::TURN_LEFT;
          break;
        case 'd':
        case 'D':
          mode_ = Mode::TURN_RIGHT;
          break;
        case 'x':
        case 'X':
        case ' ':
          mode_ = Mode::STOP;
          break;
        default:
          break;
      }
      if (mode_ != prev) {
        mode_changed = true;
      }
    }

    if (got_key) {
      last_key_time_ = now();
      if (mode_changed) {
        RCLCPP_INFO(get_logger(), "mode changed by key input");
      }
    }
  }

  /// 현재 모드를 선택된 출력 모드(motor/cmd_vel)로 변환 후 발행
  void publish_current_mode()
  {
    if (output_mode_ == "cmd_vel") {
      publish_cmd_vel_mode();
      return;
    }
    publish_motor_mode();
  }

  /// 현재 모드를 모터 ID별 rps 명령으로 변환해 발행
  void publish_motor_mode()
  {
    double left = 0.0;
    double right = 0.0;

    switch (mode_) {
      case Mode::FORWARD:
        left = left_forward_sign_ * forward_rps_;
        right = right_forward_sign_ * forward_rps_;
        break;
      case Mode::BACKWARD:
        left = -left_forward_sign_ * forward_rps_;
        right = -right_forward_sign_ * forward_rps_;
        break;
      case Mode::TURN_LEFT:
        left = -left_forward_sign_ * turn_rps_;
        right = right_forward_sign_ * turn_rps_;
        break;
      case Mode::TURN_RIGHT:
        left = left_forward_sign_ * turn_rps_;
        right = -right_forward_sign_ * turn_rps_;
        break;
      case Mode::STOP:
      default:
        break;
    }

    ros_robot_controller_msgs::msg::MotorsState msg;
    append_motor_group(msg, left_motor_ids_, left);
    append_motor_group(msg, right_motor_ids_, right);
    motor_pub_->publish(msg);

    if (left != 0.0 || right != 0.0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "publish motor cmd left=%.3f right=%.3f", left, right);
    }
  }

  /// 현재 모드를 Twist 선속/각속으로 변환해 발행
  void publish_cmd_vel_mode()
  {
    double linear = 0.0;
    double angular = 0.0;

    switch (mode_) {
      case Mode::FORWARD:
        linear = linear_vel_;
        break;
      case Mode::BACKWARD:
        linear = -linear_vel_;
        break;
      case Mode::TURN_LEFT:
        angular = angular_vel_;
        break;
      case Mode::TURN_RIGHT:
        angular = -angular_vel_;
        break;
      case Mode::STOP:
      default:
        break;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_vel_pub_->publish(msg);

    if (linear != 0.0 || angular != 0.0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "publish cmd_vel linear=%.3f angular=%.3f", linear, angular);
    }
  }

  /// 종료 처리용 즉시 정지 발행
  void publish_stop()
  {
    ros_robot_controller_msgs::msg::MotorsState m;
    append_motor_group(m, left_motor_ids_, 0.0);
    append_motor_group(m, right_motor_ids_, 0.0);
    if (motor_pub_) {
      motor_pub_->publish(m);
    }

    geometry_msgs::msg::Twist t;
    if (cmd_vel_pub_) {
      cmd_vel_pub_->publish(t);
    }
  }

  /// 동일 rps를 모터 ID 목록에 일괄 적용
  static void append_motor_group(
    ros_robot_controller_msgs::msg::MotorsState & msg,
    const vector<uint16_t> & ids,
    double rps)
  {
    for (const auto id : ids) {
      ros_robot_controller_msgs::msg::MotorState m;
      m.id = id;
      m.rps = rps;
      msg.data.push_back(m);
    }
  }

  string output_mode_;
  string motor_topic_;
  string cmd_vel_topic_;

  double forward_rps_{2.0};
  double turn_rps_{1.6};
  vector<uint16_t> left_motor_ids_;
  vector<uint16_t> right_motor_ids_;
  double left_forward_sign_{1.0};
  double right_forward_sign_{1.0};

  double linear_vel_{0.2};
  double angular_vel_{0.5};

  double idle_timeout_sec_{2.0};
  Mode mode_{Mode::STOP};
  rclcpp::Time last_key_time_{0, 0, RCL_ROS_TIME};

  termios term_orig_{};
  bool term_ready_{false};
  int input_fd_{-1};

  rclcpp::Publisher<ros_robot_controller_msgs::msg::MotorsState>::SharedPtr motor_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<KeyboardMotorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
