#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros_robot_controller_msgs/msg/bus_servo_state.hpp"
#include "ros_robot_controller_msgs/msg/button_state.hpp"
#include "ros_robot_controller_msgs/msg/buzzer_state.hpp"
#include "ros_robot_controller_msgs/msg/led_state.hpp"
#include "ros_robot_controller_msgs/msg/motors_state.hpp"
#include "ros_robot_controller_msgs/msg/oled_state.hpp"
#include "ros_robot_controller_msgs/msg/pwm_servo_state.hpp"
#include "ros_robot_controller_msgs/msg/sbus.hpp"
#include "ros_robot_controller_msgs/msg/servos_position.hpp"
#include "ros_robot_controller_msgs/msg/set_bus_servo_state.hpp"
#include "ros_robot_controller_msgs/msg/set_pwm_servo_state.hpp"
#include "ros_robot_controller_msgs/srv/get_bus_servo_state.hpp"
#include "ros_robot_controller_msgs/srv/get_pwm_servo_state.hpp"

#include "board.hpp"

using namespace std;


class RosRobotControllerCppNode : public rclcpp::Node
{
public:
  RosRobotControllerCppNode()
  : Node("ros_robot_controller"), gravity_(9.80665)
  {
    declare_parameter<string>("device", "/dev/rrc");
    declare_parameter<int>("baudrate", 1000000);
    declare_parameter<bool>("auto_detect_serial", true);
    declare_parameter<vector<string>>(
      "device_candidates", {"/dev/rrc", "/dev/rrc_alt", "/dev/ttyCH341USB0", "/dev/ttyCH341USB1"});
    declare_parameter<vector<int64_t>>(
      "baudrate_candidates", {1000000, 921600, 460800, 256000, 115200});
    declare_parameter<string>("imu_frame", "imu_link");
    declare_parameter<bool>("init_finish", false);

    const auto device = get_parameter("device").as_string();
    const auto baudrate = static_cast<int>(get_parameter("baudrate").as_int());
    const auto auto_detect = get_parameter("auto_detect_serial").as_bool();
    const auto device_candidates = get_parameter("device_candidates").as_string_array();
    const auto baudrate_candidates = get_parameter("baudrate_candidates").as_integer_array();
    imu_frame_ = get_parameter("imu_frame").as_string();

    if (auto_detect) {
      auto_detect_board(device, baudrate, device_candidates, baudrate_candidates);
    } else {
      board_ = unique_ptr<Board>(new Board(device, baudrate));
      board_->enable_reception(true);
      selected_device_ = device;
      selected_baudrate_ = baudrate;
    }

    RCLCPP_INFO(get_logger(), "serial: %s @ %d", selected_device_.c_str(), selected_baudrate_);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/imu_raw", 1);
    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("~/joy", 1);
    sbus_pub_ = create_publisher<ros_robot_controller_msgs::msg::Sbus>("~/sbus", 1);
    button_pub_ = create_publisher<ros_robot_controller_msgs::msg::ButtonState>("~/button", 1);
    battery_pub_ = create_publisher<std_msgs::msg::UInt16>("~/battery", 1);

    led_sub_ = create_subscription<ros_robot_controller_msgs::msg::LedState>(
      "~/set_led", 5, bind(&RosRobotControllerCppNode::set_led_state, this, placeholders::_1));
    buzzer_sub_ = create_subscription<ros_robot_controller_msgs::msg::BuzzerState>(
      "~/set_buzzer", 5, bind(&RosRobotControllerCppNode::set_buzzer_state, this, placeholders::_1));
    oled_sub_ = create_subscription<ros_robot_controller_msgs::msg::OLEDState>(
      "~/set_oled", 5, bind(&RosRobotControllerCppNode::set_oled_state, this, placeholders::_1));
    motor_sub_ = create_subscription<ros_robot_controller_msgs::msg::MotorsState>(
      "~/set_motor", 10, bind(&RosRobotControllerCppNode::set_motor_state, this, placeholders::_1));
    enable_reception_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/enable_reception", 1,
      bind(&RosRobotControllerCppNode::enable_reception_callback, this, placeholders::_1));

    bus_servo_set_state_sub_ = create_subscription<ros_robot_controller_msgs::msg::SetBusServoState>(
      "~/bus_servo/set_state", 10,
      bind(&RosRobotControllerCppNode::set_bus_servo_state, this, placeholders::_1));
    bus_servo_set_position_sub_ = create_subscription<ros_robot_controller_msgs::msg::ServosPosition>(
      "~/bus_servo/set_position", 10,
      bind(&RosRobotControllerCppNode::set_bus_servo_position, this, placeholders::_1));
    pwm_servo_set_state_sub_ = create_subscription<ros_robot_controller_msgs::msg::SetPWMServoState>(
      "~/pwm_servo/set_state", 10,
      bind(&RosRobotControllerCppNode::set_pwm_servo_state, this, placeholders::_1));

    bus_servo_get_state_srv_ = create_service<ros_robot_controller_msgs::srv::GetBusServoState>(
      "~/bus_servo/get_state",
      bind(
        &RosRobotControllerCppNode::get_bus_servo_state, this, placeholders::_1,
        placeholders::_2));
    pwm_servo_get_state_srv_ = create_service<ros_robot_controller_msgs::srv::GetPWMServoState>(
      "~/pwm_servo/get_state",
      bind(
        &RosRobotControllerCppNode::get_pwm_servo_state, this, placeholders::_1,
        placeholders::_2));
    init_finish_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/init_finish",
      bind(&RosRobotControllerCppNode::get_node_state, this, placeholders::_1, placeholders::_2));

    try {
      board_->pwm_servo_set_offset(1, 0);
      board_->set_motor_speed({{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}});
    } catch (const exception & e) {
      RCLCPP_WARN(get_logger(), "initial command failed: %s", e.what());
    }

    pub_timer_ = create_wall_timer(
      chrono::milliseconds(20), bind(&RosRobotControllerCppNode::pub_callback, this));

    RCLCPP_INFO(get_logger(), "start");
  }

  ~RosRobotControllerCppNode() override
  {
    if (board_) {
      board_->enable_reception(false);
      try {
        board_->set_motor_speed({{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}});
      } catch (...) {
      }
      board_->close();
    }
  }

private:
  /// 설정된 후보(device/baudrate)를 순회하며 IMU/배터리/버튼 텔레메트리로 보드 연결을 자동 탐지
  void auto_detect_board(
    const string & preferred_device, int preferred_baudrate,
    const vector<string> & device_candidates,
    const vector<int64_t> & baudrate_candidates)
  {
    vector<string> devices{preferred_device};
    for (const auto & d : device_candidates) {
      if (d != preferred_device) {
        devices.push_back(d);
      }
    }

    vector<int> bauds{preferred_baudrate};
    for (const auto b : baudrate_candidates) {
      const int v = static_cast<int>(b);
      if (v != preferred_baudrate) {
        bauds.push_back(v);
      }
    }

    auto has_telemetry = [](Board & board) {
      const auto deadline = chrono::steady_clock::now() + chrono::milliseconds(1500);
      while (chrono::steady_clock::now() < deadline) {
        if (board.get_imu().has_value() || board.get_battery().has_value() || board.get_button().has_value()) {
          return true;
        }
        this_thread::sleep_for(chrono::milliseconds(20));
      }
      return false;
    };

    for (const auto & dev : devices) {
      for (const auto baud : bauds) {
        try {
          auto tmp = unique_ptr<Board>(new Board(dev, baud));
          tmp->enable_reception(true);
          if (has_telemetry(*tmp)) {
            board_ = move(tmp);
            selected_device_ = dev;
            selected_baudrate_ = baud;
            RCLCPP_INFO(get_logger(), "serial auto-detect matched %s @ %d", dev.c_str(), baud);
            return;
          }
          tmp->close();
        } catch (...) {
          continue;
        }
      }
    }

    RCLCPP_WARN(get_logger(), "serial auto-detect found no telemetry, fallback to configured device");
    board_ = unique_ptr<Board>(new Board(preferred_device, preferred_baudrate));
    board_->enable_reception(true);
    selected_device_ = preferred_device;
    selected_baudrate_ = preferred_baudrate;
  }

  /// 주기 타이머에서 수신 텔레메트리를 각 ROS 토픽으로 발행
  void pub_callback()
  {
    if (!enable_reception_flag_ || !board_) {
      return;
    }

    try {
      pub_button_data();
      pub_joy_data();
      pub_imu_data();
      pub_sbus_data();
      pub_battery_data();
    } catch (const exception & e) {
      RCLCPP_ERROR(get_logger(), "publisher loop error: %s", e.what());
    }
  }

  /// 외부 요청으로 보드 수신 스레드를 on/off
  void enable_reception_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    enable_reception_flag_ = msg->data;
    if (board_) {
      board_->enable_reception(msg->data);
    }
    RCLCPP_INFO(get_logger(), "enable_reception %s", msg->data ? "true" : "false");
  }

  /// LED 상태 제어 명령 전달
  void set_led_state(const ros_robot_controller_msgs::msg::LedState::SharedPtr msg)
  {
    if (board_) {
      board_->set_led(msg->on_time, msg->off_time, msg->repeat, msg->id);
    }
  }

  /// 부저 상태 제어 명령 전달
  void set_buzzer_state(const ros_robot_controller_msgs::msg::BuzzerState::SharedPtr msg)
  {
    if (board_) {
      board_->set_buzzer(msg->freq, msg->on_time, msg->off_time, msg->repeat);
    }
  }

  /// 다중 모터(rps) 제어 명령 전달
  void set_motor_state(const ros_robot_controller_msgs::msg::MotorsState::SharedPtr msg)
  {
    if (!board_) {
      return;
    }
    vector<pair<uint16_t, double>> data;
    data.reserve(msg->data.size());
    for (const auto & i : msg->data) {
      data.emplace_back(i.id, i.rps);
    }
    board_->set_motor_speed(data);
  }

  /// OLED 문자열 출력 명령 전달
  void set_oled_state(const ros_robot_controller_msgs::msg::OLEDState::SharedPtr msg)
  {
    if (board_) {
      board_->set_oled_text(static_cast<uint8_t>(msg->index), msg->text);
    }
  }

  /// PWM 서보 상태(위치/오프셋) 설정 메시지를 보드 명령으로 변환
  void set_pwm_servo_state(const ros_robot_controller_msgs::msg::SetPWMServoState::SharedPtr msg)
  {
    if (!board_) {
      return;
    }

    vector<pair<uint16_t, uint16_t>> data;
    for (const auto & i : msg->state) {
      if (!i.id.empty() && !i.position.empty()) {
        data.emplace_back(i.id[0], i.position[0]);
      }
      if (!i.id.empty() && !i.offset.empty()) {
        board_->pwm_servo_set_offset(static_cast<uint8_t>(i.id[0]), i.offset[0]);
      }
    }
    if (!data.empty()) {
      board_->pwm_servo_set_position(msg->duration, data);
    }
  }

  /// PWM 서보 상태 조회 서비스 처리
  void get_pwm_servo_state(
    const shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState::Request> request,
    shared_ptr<ros_robot_controller_msgs::srv::GetPWMServoState::Response> response)
  {
    response->state.clear();
    for (const auto & i : request->cmd) {
      ros_robot_controller_msgs::msg::PWMServoState data;
      if (i.get_position) {
        auto state = board_->pwm_servo_read_position(i.id);
        if (state.has_value()) {
          data.position.push_back(state.value());
        }
      }
      if (i.get_offset) {
        auto state = board_->pwm_servo_read_offset(i.id);
        if (state.has_value()) {
          data.offset.push_back(state.value());
        }
      }
      response->state.push_back(data);
    }
    response->success = true;
  }

  /// 버스 서보 목표 위치 일괄 명령 전달
  void set_bus_servo_position(const ros_robot_controller_msgs::msg::ServosPosition::SharedPtr msg)
  {
    if (!board_) {
      return;
    }

    vector<pair<uint16_t, uint16_t>> data;
    data.reserve(msg->position.size());
    for (const auto & i : msg->position) {
      data.emplace_back(i.id, i.position);
    }
    if (!data.empty()) {
      board_->bus_servo_set_position(msg->duration, data);
    }
  }

  /// [flag, value] 형식 필드의 enable 플래그 검사 유틸
  template<typename T>
  static bool has_flag(const vector<T> & v)
  {
    return !v.empty() && v[0] != 0;
  }

  /// [flag, value] 형식 필드가 유효한 값까지 포함하는지 검사 유틸
  template<typename T>
  static bool has_flag_and_value(const vector<T> & v)
  {
    return v.size() >= 2 && v[0] != 0;
  }

  /// 버스 서보 상태 설정 메시지를 각 하위 명령(ID/각도/토크/리밋)으로 분해해 적용
  void set_bus_servo_state(const ros_robot_controller_msgs::msg::SetBusServoState::SharedPtr msg)
  {
    if (!board_) {
      return;
    }

    vector<pair<uint16_t, uint16_t>> data;
    vector<uint8_t> servo_stop_ids;

    for (const auto & i : msg->state) {
      if (!has_flag_and_value(i.present_id)) {
        continue;
      }

      const uint8_t sid = static_cast<uint8_t>(i.present_id[1]);

      if (has_flag_and_value(i.target_id)) {
        board_->bus_servo_set_id(sid, static_cast<uint8_t>(i.target_id[1]));
      }
      if (has_flag_and_value(i.position)) {
        data.emplace_back(sid, static_cast<uint16_t>(i.position[1]));
      }
      if (has_flag_and_value(i.offset)) {
        board_->bus_servo_set_offset(sid, static_cast<int16_t>(i.offset[1]));
      }
      if (i.position_limit.size() >= 3 && i.position_limit[0] != 0) {
        board_->bus_servo_set_angle_limit(sid, {i.position_limit[1], i.position_limit[2]});
      }
      if (i.voltage_limit.size() >= 3 && i.voltage_limit[0] != 0) {
        board_->bus_servo_set_vin_limit(sid, {i.voltage_limit[1], i.voltage_limit[2]});
      }
      if (has_flag_and_value(i.max_temperature_limit)) {
        board_->bus_servo_set_temp_limit(sid, static_cast<uint8_t>(i.max_temperature_limit[1]));
      }
      if (has_flag_and_value(i.enable_torque)) {
        board_->bus_servo_enable_torque(sid, i.enable_torque[1] != 0);
      }
      if (has_flag(i.save_offset)) {
        board_->bus_servo_save_offset(sid);
      }
      if (has_flag(i.stop)) {
        servo_stop_ids.push_back(sid);
      }
    }

    if (!data.empty()) {
      board_->bus_servo_set_position(msg->duration, data);
    }
    if (!servo_stop_ids.empty()) {
      board_->bus_servo_stop(servo_stop_ids);
    }
  }

  /// 버스 서보 상태 조회 서비스 처리
  void get_bus_servo_state(
    const shared_ptr<ros_robot_controller_msgs::srv::GetBusServoState::Request> request,
    shared_ptr<ros_robot_controller_msgs::srv::GetBusServoState::Response> response)
  {
    response->state.clear();

    for (const auto & i : request->cmd) {
      ros_robot_controller_msgs::msg::BusServoState data;
      uint8_t sid = i.id;

      if (i.get_id) {
        auto state = board_->bus_servo_read_id(sid);
        if (state.has_value()) {
          sid = state.value();
          data.present_id.push_back(sid);
        }
      }
      if (i.get_position) {
        auto state = board_->bus_servo_read_position(sid);
        if (state.has_value()) {
          data.position.push_back(static_cast<uint16_t>(state.value()));
        }
      }
      if (i.get_offset) {
        auto state = board_->bus_servo_read_offset(sid);
        if (state.has_value()) {
          data.offset.push_back(state.value());
        }
      }
      if (i.get_voltage) {
        auto state = board_->bus_servo_read_vin(sid);
        if (state.has_value()) {
          data.voltage.push_back(state.value());
        }
      }
      if (i.get_temperature) {
        auto state = board_->bus_servo_read_temp(sid);
        if (state.has_value()) {
          data.temperature.push_back(state.value());
        }
      }
      if (i.get_position_limit) {
        auto state = board_->bus_servo_read_angle_limit(sid);
        if (state.has_value()) {
          data.position_limit.push_back(state.value()[0]);
          data.position_limit.push_back(state.value()[1]);
        }
      }
      if (i.get_voltage_limit) {
        auto state = board_->bus_servo_read_vin_limit(sid);
        if (state.has_value()) {
          data.voltage_limit.push_back(state.value()[0]);
          data.voltage_limit.push_back(state.value()[1]);
        }
      }
      if (i.get_max_temperature_limit) {
        auto state = board_->bus_servo_read_temp_limit(sid);
        if (state.has_value()) {
          data.max_temperature_limit.push_back(state.value());
        }
      }
      if (i.get_torque_state) {
        auto state = board_->bus_servo_read_torque_state(sid);
        if (state.has_value()) {
          data.enable_torque.push_back(static_cast<uint16_t>(state.value()));
        }
      }

      response->state.push_back(data);
    }

    response->success = true;
  }

  /// 배터리 전압 텔레메트리 발행
  void pub_battery_data()
  {
    auto data = board_->get_battery();
    if (data.has_value()) {
      std_msgs::msg::UInt16 msg;
      msg.data = data.value();
      battery_pub_->publish(msg);
    }
  }

  /// 버튼 상태 텔레메트리 발행
  void pub_button_data()
  {
    auto data = board_->get_button();
    if (data.has_value()) {
      ros_robot_controller_msgs::msg::ButtonState msg;
      msg.id = data->first;
      msg.state = data->second;
      button_pub_->publish(msg);
    }
  }

  /// 게임패드 축/버튼 텔레메트리 발행
  void pub_joy_data()
  {
    auto data = board_->get_gamepad();
    if (data.has_value()) {
      sensor_msgs::msg::Joy msg;
      msg.axes.assign(data->first.begin(), data->first.end());
      msg.buttons.assign(data->second.begin(), data->second.end());
      msg.header.stamp = now();
      joy_pub_->publish(msg);
    }
  }

  /// SBUS 채널 텔레메트리 발행
  void pub_sbus_data()
  {
    auto data = board_->get_sbus();
    if (data.has_value()) {
      ros_robot_controller_msgs::msg::Sbus msg;
      msg.channel = data.value();
      msg.header.stamp = now();
      sbus_pub_->publish(msg);
    }
  }

  /// IMU 텔레메트리(g, deg/s)를 ROS 표준 단위(m/s^2, rad/s)로 변환해 발행
  void pub_imu_data()
  {
    auto data = board_->get_imu();
    if (!data.has_value()) {
      const auto t = now();
      if ((t - last_imu_missing_warn_).seconds() > 5.0) {
        last_imu_missing_warn_ = t;
        RCLCPP_WARN(
          get_logger(), "no IMU telemetry on %s @ %d (topic /ros_robot_controller/imu_raw will stay empty)",
          selected_device_.c_str(), selected_baudrate_);
      }
      return;
    }

    const auto & d = data.value();
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = imu_frame_;
    msg.header.stamp = now();

    msg.orientation.w = 0.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;

    msg.linear_acceleration.x = d[0] * gravity_;
    msg.linear_acceleration.y = d[1] * gravity_;
    msg.linear_acceleration.z = d[2] * gravity_;

    msg.angular_velocity.x = d[3] * M_PI / 180.0;
    msg.angular_velocity.y = d[4] * M_PI / 180.0;
    msg.angular_velocity.z = d[5] * M_PI / 180.0;

    msg.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
    msg.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
    msg.linear_acceleration_covariance = {0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.004};

    imu_pub_->publish(msg);
  }

  /// 초기화 완료 확인 서비스(항상 success=true 반환)
  void get_node_state(
    const shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = true;
  }

private:
  double gravity_;
  bool enable_reception_flag_{true};

  string selected_device_;
  int selected_baudrate_{1000000};
  string imu_frame_;

  unique_ptr<Board> board_;
  rclcpp::Time last_imu_missing_warn_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<ros_robot_controller_msgs::msg::Sbus>::SharedPtr sbus_pub_;
  rclcpp::Publisher<ros_robot_controller_msgs::msg::ButtonState>::SharedPtr button_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr battery_pub_;

  rclcpp::Subscription<ros_robot_controller_msgs::msg::LedState>::SharedPtr led_sub_;
  rclcpp::Subscription<ros_robot_controller_msgs::msg::BuzzerState>::SharedPtr buzzer_sub_;
  rclcpp::Subscription<ros_robot_controller_msgs::msg::OLEDState>::SharedPtr oled_sub_;
  rclcpp::Subscription<ros_robot_controller_msgs::msg::MotorsState>::SharedPtr motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_reception_sub_;
  rclcpp::Subscription<ros_robot_controller_msgs::msg::SetBusServoState>::SharedPtr bus_servo_set_state_sub_;
  rclcpp::Subscription<ros_robot_controller_msgs::msg::ServosPosition>::SharedPtr bus_servo_set_position_sub_;
  rclcpp::Subscription<ros_robot_controller_msgs::msg::SetPWMServoState>::SharedPtr pwm_servo_set_state_sub_;

  rclcpp::Service<ros_robot_controller_msgs::srv::GetBusServoState>::SharedPtr bus_servo_get_state_srv_;
  rclcpp::Service<ros_robot_controller_msgs::srv::GetPWMServoState>::SharedPtr pwm_servo_get_state_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_finish_srv_;

  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<RosRobotControllerCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
