#include <algorithm>
#include <chrono>
#include <cctype>
#include <functional>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros_robot_controller_msgs/msg/servo_position.hpp"
#include "ros_robot_controller_msgs/msg/servos_position.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std;

class IntentArmBridgeNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  IntentArmBridgeNode()
  : Node("intent_arm_bridge_node")
  {
    declare_parameter<string>("intent_category_topic", "/intent_router/category");
    declare_parameter<string>("intent_command_topic", "/intent_router/robot_command");
    declare_parameter<string>("trajectory_action_name", "/arm_controller/follow_joint_trajectory");
    declare_parameter<double>("trajectory_duration_sec", 1.5);
    declare_parameter<string>("servo_position_topic", "/ros_robot_controller/bus_servo/set_position");
    declare_parameter<double>("home_pulse_duration_sec", 1.0);

    declare_parameter<vector<string>>(
      "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"});
    declare_parameter<vector<int64_t>>("servo_ids", {1, 2, 3, 4, 5, 10});
    declare_parameter<vector<int64_t>>("target_home_pulses", {500, 765, 15, 220, 500, 500});

    declare_parameter<vector<double>>("target_home", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<vector<double>>("target_ready", {0.0, -0.5, 0.9, 0.7, 0.0, 0.0});
    declare_parameter<vector<double>>("target_pick", {0.0, -0.8, 1.2, 0.6, 0.0, 0.6});
    declare_parameter<vector<double>>("target_place", {0.6, -0.5, 1.0, 0.7, 0.0, 0.1});

    intent_category_topic_ = get_parameter("intent_category_topic").as_string();
    intent_command_topic_ = get_parameter("intent_command_topic").as_string();
    action_name_ = get_parameter("trajectory_action_name").as_string();
    trajectory_duration_sec_ = max(0.2, get_parameter("trajectory_duration_sec").as_double());
    servo_position_topic_ = get_parameter("servo_position_topic").as_string();
    home_pulse_duration_sec_ = max(0.1, get_parameter("home_pulse_duration_sec").as_double());

    joint_names_ = get_parameter("joint_names").as_string_array();
    const auto servo_ids_raw = get_parameter("servo_ids").as_integer_array();
    const auto home_pulses_raw = get_parameter("target_home_pulses").as_integer_array();
    servo_ids_.assign(servo_ids_raw.begin(), servo_ids_raw.end());
    target_home_pulses_.assign(home_pulses_raw.begin(), home_pulses_raw.end());
    target_home_ = get_parameter("target_home").as_double_array();
    target_ready_ = get_parameter("target_ready").as_double_array();
    target_pick_ = get_parameter("target_pick").as_double_array();
    target_place_ = get_parameter("target_place").as_double_array();

    if (!validate_target_size(target_home_, "target_home") ||
      !validate_target_size(target_ready_, "target_ready") ||
      !validate_target_size(target_pick_, "target_pick") ||
      !validate_target_size(target_place_, "target_place"))
    {
      throw runtime_error("target vector size mismatch with joint_names");
    }
    if (servo_ids_.size() != joint_names_.size() || target_home_pulses_.size() != joint_names_.size()) {
      throw runtime_error("servo_ids/target_home_pulses size mismatch with joint_names");
    }

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_name_);
    pulse_pub_ = create_publisher<ros_robot_controller_msgs::msg::ServosPosition>(servo_position_topic_, 10);

    sub_category_ = create_subscription<std_msgs::msg::String>(
      intent_category_topic_, 10,
      bind(&IntentArmBridgeNode::on_category, this, placeholders::_1));

    sub_robot_command_ = create_subscription<std_msgs::msg::String>(
      intent_command_topic_, 10,
      bind(&IntentArmBridgeNode::on_robot_command, this, placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "intent_arm_bridge ready: category=%s command=%s action=%s",
      intent_category_topic_.c_str(), intent_command_topic_.c_str(), action_name_.c_str());
  }

private:
  /// 전달받은 목표 각도 벡터 길이가 조인트 개수와 일치하는지 검증
  bool validate_target_size(const vector<double> & target, const string & name) const
  {
    if (target.size() == joint_names_.size()) {
      return true;
    }
    RCLCPP_ERROR(
      get_logger(), "%s size mismatch: expected=%zu actual=%zu",
      name.c_str(), joint_names_.size(), target.size());
    return false;
  }

  /// intent category 최신 값을 저장(안전 명령 우선 처리에 사용)
  void on_category(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_category_ = trim(msg->data);
  }

  /// robot_command(JSON 문자열)를 받아 사전 정의 자세로 변환 후 팔 액션 목표 전송
  void on_robot_command(const std_msgs::msg::String::SharedPtr msg)
  {
    const string payload = trim(msg->data);
    if (payload.empty()) {
      return;
    }

    if (latest_category_ == "safety_command" || payload == "STOP") {
      RCLCPP_WARN(get_logger(), "safety command received, skip arm motion");
      return;
    }

    string command = extract_json_string(payload, "command");
    if (command.empty()) {
      command = payload;
    }

    if (is_home_command(command, payload)) {
      publish_home_pulse_goal();
      return;
    }

    const vector<double> * target = resolve_target(command, payload);
    if (target == nullptr) {
      RCLCPP_WARN(get_logger(), "unsupported robot command: %s", payload.c_str());
      return;
    }

    send_trajectory_goal(*target, command);
  }

  /// 명령 키워드 기반으로 목표 자세(home/ready/pick/place) 선택
  const vector<double> * resolve_target(const string & command, const string & payload) const
  {
    const string merged = normalize_text(command + " " + payload);

    if (contains_any(
        merged,
        {
          "준비", "대기", "ready", "standby",
          "팔 들어", "팔들어", "팔 올려", "팔올려", "들어", "들어줘", "올려", "올려줘",
          "들어 올려", "들어올려", "위로 올려", "위로", "팔 펴", "펴줘"
        }))
    {
      return &target_ready_;
    }
    if (contains_any(
        merged,
        {
          "집기", "잡아", "잡아줘", "집어", "집어줘", "집어 들어", "집어들어",
          "쥐어", "쥐어줘", "물체 잡아", "pick", "grasp", "grab"
        }))
    {
      return &target_pick_;
    }
    if (contains_any(
        merged,
        {
          "놓기", "놔", "놔줘", "놓아", "놓아줘", "내려놔", "내려놔줘", "놓고",
          "풀어", "풀어줘", "place", "drop", "release"
        }))
    {
      return &target_place_;
    }

    return nullptr;
  }

  /// home 계열 명령인지 판별
  bool is_home_command(const string & command, const string & payload) const
  {
    const string merged = normalize_text(command + " " + payload);
    return contains_any(
      merged,
      {
        "원위치", "홈", "home", "초기", "초기화", "기본자세", "리셋", "reset",
        "팔 내려", "팔을 내려", "팔내려", "내려", "내려줘", "내려와", "팔 접어", "접어줘"
      });
  }

  /// home 계열은 지정 pulse 좌표로 서보를 직접 구동
  void publish_home_pulse_goal()
  {
    if (!pulse_pub_) {
      RCLCPP_ERROR(get_logger(), "pulse publisher not ready");
      return;
    }
    ros_robot_controller_msgs::msg::ServosPosition msg;
    msg.duration = home_pulse_duration_sec_;
    msg.position.reserve(servo_ids_.size());
    for (size_t i = 0; i < servo_ids_.size(); ++i) {
      ros_robot_controller_msgs::msg::ServoPosition sp;
      sp.id = static_cast<int32_t>(servo_ids_[i]);
      sp.position = static_cast<int32_t>(target_home_pulses_[i]);
      msg.position.push_back(sp);
    }
    pulse_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "published home pulse goal [500,765,15,220,500,500]");
  }

  /// FollowJointTrajectory 액션으로 1포인트 목표를 전송
  void send_trajectory_goal(const vector<double> & target, const string & reason)
  {
    if (goal_in_progress_) {
      RCLCPP_WARN(get_logger(), "previous goal is still running, skip command=%s", reason.c_str());
      return;
    }

    if (!action_client_->wait_for_action_server(chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "action server unavailable: %s", action_name_.c_str());
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target;
    point.time_from_start = rclcpp::Duration::from_seconds(trajectory_duration_sec_);
    goal.trajectory.points.push_back(point);

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback =
      bind(&IntentArmBridgeNode::on_goal_response, this, placeholders::_1);
    options.feedback_callback =
      bind(&IntentArmBridgeNode::on_feedback, this, placeholders::_1, placeholders::_2);
    options.result_callback =
      bind(&IntentArmBridgeNode::on_result, this, placeholders::_1);

    goal_in_progress_ = true;
    action_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "sent arm goal from intent command=%s", reason.c_str());
  }

  /// goal 수락/거절 결과를 기록
  void on_goal_response(GoalHandleFJT::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      goal_in_progress_ = false;
      RCLCPP_ERROR(get_logger(), "trajectory goal rejected");
      return;
    }
    RCLCPP_INFO(get_logger(), "trajectory goal accepted");
  }

  /// 액션 feedback는 필요 시 확장 가능, 현재는 과도한 로그를 피하기 위해 비워둠
  void on_feedback(
    GoalHandleFJT::SharedPtr,
    const shared_ptr<const FollowJointTrajectory::Feedback>)
  {
  }

  /// 액션 완료 상태를 기록하고 다음 명령 수락 가능 상태로 전환
  void on_result(const GoalHandleFJT::WrappedResult & result)
  {
    goal_in_progress_ = false;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "trajectory succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "trajectory aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "trajectory canceled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "trajectory finished with unknown result");
        break;
    }
  }

  /// 문자열 앞뒤 공백 제거
  static string trim(const string & in)
  {
    size_t begin = 0;
    while (begin < in.size() && isspace(static_cast<unsigned char>(in[begin])) != 0) {
      ++begin;
    }
    if (begin == in.size()) {
      return "";
    }
    size_t end = in.size() - 1;
    while (end > begin && isspace(static_cast<unsigned char>(in[end])) != 0) {
      --end;
    }
    return in.substr(begin, end - begin + 1);
  }

  /// 비교용 텍스트 정규화(영문 소문자 + 양끝 공백 제거)
  static string normalize_text(string in)
  {
    transform(in.begin(), in.end(), in.begin(), [](unsigned char c) {
      return static_cast<char>(tolower(c));
    });
    return trim(in);
  }

  /// 공백을 제거한 compact 문자열 생성(키워드 매칭 강건성 향상)
  static string remove_spaces(const string & in)
  {
    string out;
    out.reserve(in.size());
    for (char c : in) {
      if (isspace(static_cast<unsigned char>(c)) == 0) {
        out.push_back(c);
      }
    }
    return out;
  }

  /// 키워드 목록 중 하나라도 포함되는지 확인
  static bool contains_any(const string & text, const vector<string> & keywords)
  {
    const string text_norm = normalize_text(text);
    const string text_compact = remove_spaces(text_norm);
    for (const auto & keyword : keywords) {
      if (keyword.empty()) {
        continue;
      }
      const string key_norm = normalize_text(keyword);
      const string key_compact = remove_spaces(key_norm);
      if (
        text_norm.find(key_norm) != string::npos ||
        text_compact.find(key_compact) != string::npos)
      {
        return true;
      }
    }
    return false;
  }

  /// 간단한 JSON 문자열 필드 추출("field":"value")
  static string extract_json_string(const string & json_like, const string & field)
  {
    try {
      const regex pattern("\\\"" + field + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
      smatch match;
      if (regex_search(json_like, match, pattern) && match.size() >= 2) {
        return trim(match[1].str());
      }
    } catch (...) {
      return "";
    }
    return "";
  }

  string intent_category_topic_;
  string intent_command_topic_;
  string action_name_;
  string servo_position_topic_;
  double trajectory_duration_sec_{1.5};
  double home_pulse_duration_sec_{1.0};

  vector<string> joint_names_;
  vector<int64_t> servo_ids_;
  vector<int64_t> target_home_pulses_;
  vector<double> target_home_;
  vector<double> target_ready_;
  vector<double> target_pick_;
  vector<double> target_place_;

  string latest_category_;
  bool goal_in_progress_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_category_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_command_;
  rclcpp::Publisher<ros_robot_controller_msgs::msg::ServosPosition>::SharedPtr pulse_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<IntentArmBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
