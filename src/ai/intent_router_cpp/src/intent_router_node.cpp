#include "intent_router_cpp/intent_router_node.hpp"

#include <rover_common/json_utils.hpp>
#include <rover_common/curl_utils.hpp>
#include <rover_common/string_utils.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <sstream>
#include <string>

using namespace std;


namespace intent_router_cpp
{
namespace
{

string find_last_json_object(const string & text)
{
  bool in_string = false;
  bool escaping = false;
  int depth = 0;
  size_t start = string::npos;
  string last_json;

  for (size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (escaping) {
      escaping = false;
      continue;
    }
    if (c == '\\') {
      escaping = true;
      continue;
    }
    if (c == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (c == '{') {
      if (depth == 0) {
        start = i;
      }
      ++depth;
      continue;
    }
    if (c == '}' && depth > 0) {
      --depth;
      if (depth == 0 && start != string::npos) {
        last_json = text.substr(start, i - start + 1);
      }
    }
  }
  return last_json;
}

bool extract_json_bool_field(const string & json, const string & field, bool & out)
{
  const string key = "\"" + field + "\"";
  const size_t key_pos = json.find(key);
  if (key_pos == string::npos) {
    return false;
  }
  const size_t colon_pos = json.find(':', key_pos + key.size());
  if (colon_pos == string::npos) {
    return false;
  }
  size_t value_pos = colon_pos + 1;
  while (value_pos < json.size() && isspace(static_cast<unsigned char>(json[value_pos])) != 0) {
    ++value_pos;
  }
  if (json.compare(value_pos, 4, "true") == 0) {
    out = true;
    return true;
  }
  if (json.compare(value_pos, 5, "false") == 0) {
    out = false;
    return true;
  }
  return false;
}

string replace_all(string value, const string & from, const string & to)
{
  if (from.empty()) {
    return value;
  }
  size_t pos = 0;
  while ((pos = value.find(from, pos)) != string::npos) {
    value.replace(pos, from.size(), to);
    pos += to.size();
  }
  return value;
}

}  // namespace

IntentRouterNode::IntentRouterNode()
: Node("intent_router_node")
{
  declare_and_get_parameters();

  pub_category_     = create_publisher<std_msgs::msg::String>("/intent_router/category", 10);
  pub_robot_command_= create_publisher<std_msgs::msg::String>("/intent_router/robot_command", 10);
  pub_chat_text_    = create_publisher<std_msgs::msg::String>("/intent_router/chat_text", 10);
  pub_debug_        = create_publisher<std_msgs::msg::String>("/intent_router/debug", 10);

  sub_transcript_ = create_subscription<std_msgs::msg::String>(
    "/wake_vad/transcript", 10, bind(&IntentRouterNode::on_transcript, this, placeholders::_1));

  RCLCPP_INFO(get_logger(), "intent_router_cpp node started (groq_enabled=%s)",
    groq_enabled_ ? "true" : "false");
}

IntentRouterNode::~IntentRouterNode() = default;

void IntentRouterNode::declare_and_get_parameters()
{
  declare_parameter<vector<string>>(
    "emergency_keywords",
    vector<string>{"멈춰", "정지", "stop", "그만", "위험"});

  // "auto" = groq → keyword fallback | "keyword" = keyword only | "groq" = groq only
  declare_parameter<string>("intent_provider", "auto");
  declare_parameter<bool>("groq_enabled", true);
  declare_parameter<string>("groq_api_key", "");
  declare_parameter<string>("groq_model", "llama-3.3-70b-versatile");
  declare_parameter<int>("groq_timeout_sec", 8);
  declare_parameter<double>("groq_temperature", 0.1);
  declare_parameter<int>("groq_max_tokens", 256);
  declare_parameter<string>(
    "groq_prompt_template",
    "당신은 로봇 명령 분석기입니다. 텍스트를 받아 JSON으로 분류하세요.\n\n"
    "[분류]\n"
    "safety_command: \"멈춰\",\"정지\",\"stop\",\"그만\",\"위험\"\n"
    "robot_command: 이동, 집기, 놓기 등 물리 동작\n"
    "chat: 정보 요청, 인사\n\n"
    "[공간]\n"
    "\"저기\"->reference:\"there\", \"여기\"->reference:\"here\"\n"
    "색상 언급 시 needs_vision:true\n\n"
    "[출력]\n"
    "{\"category\":\"safety_command|robot_command|chat\",\"command\":\"...\",\"chat_text\":\"...\",\"needs_vision\":false}\n\n"
    "입력: {input}\n"
    "출력:");
  declare_parameter<string>(
    "groq_json_schema",
    "{\"type\":\"object\",\"required\":[\"category\",\"command\",\"chat_text\",\"needs_vision\"],"
    "\"properties\":{\"category\":{\"type\":\"string\",\"enum\":[\"safety_command\",\"robot_command\",\"chat\"]},"
    "\"command\":{\"type\":\"string\"},\"chat_text\":{\"type\":\"string\"},\"needs_vision\":{\"type\":\"boolean\"}},"
    "\"additionalProperties\":false}");

  emergency_keywords_ = get_parameter("emergency_keywords").as_string_array();

  // intent_provider가 "keyword"면 groq_enabled를 false로 오버라이드
  const string intent_provider = get_parameter("intent_provider").as_string();
  if (intent_provider == "keyword") {
    groq_enabled_ = false;
  } else if (intent_provider == "groq") {
    groq_enabled_ = true;
  } else {
    groq_enabled_ = get_parameter("groq_enabled").as_bool();
  }
  groq_api_key_       = get_parameter("groq_api_key").as_string();
  if (groq_api_key_.empty()) {
    const char * env_key = getenv("GROQ_API_KEY");
    if (env_key != nullptr) {
      groq_api_key_ = env_key;
    }
  }
  groq_model_           = get_parameter("groq_model").as_string();
  groq_timeout_sec_     = get_parameter("groq_timeout_sec").as_int();
  groq_temperature_     = get_parameter("groq_temperature").as_double();
  groq_max_tokens_      = get_parameter("groq_max_tokens").as_int();
  groq_prompt_template_ = get_parameter("groq_prompt_template").as_string();
  groq_json_schema_     = get_parameter("groq_json_schema").as_string();
}

void IntentRouterNode::on_transcript(const std_msgs::msg::String::SharedPtr msg)
{
  const string text = rover_common::trim(msg->data);
  if (text.empty()) {
    return;
  }

  // 긴급 키워드: 분류 없이 즉시 정지
  if (contains_any_keyword(text, emergency_keywords_)) {
    publish_string(pub_category_, "safety_command");
    publish_string(pub_robot_command_, "STOP");
    publish_string(pub_debug_, "safety_bypass_keyword");
    RCLCPP_WARN(get_logger(), "intent=safety_command bypass text=%s", text.c_str());
    return;
  }

  if (!groq_enabled_) {
    publish_string(pub_category_, "chat");
    publish_string(pub_chat_text_, text);
    publish_string(pub_debug_, "groq_disabled_chat_fallback");
    RCLCPP_INFO(get_logger(), "groq disabled, forwarding as chat: %s", text.c_str());
    return;
  }

  const IntentRouteResult route = route_with_groq(text);
  if (!route.ok) {
    publish_string(pub_category_, "chat");
    publish_string(pub_chat_text_, text);
    publish_string(pub_debug_, "groq_route_error: " + route.error);
    RCLCPP_WARN(get_logger(), "groq route failed: %s, forwarding as chat", route.error.c_str());
    return;
  }

  if (route.category == "safety_command") {
    publish_string(pub_category_, "safety_command");
    publish_string(pub_robot_command_, "STOP");
    publish_string(pub_debug_, "groq_route_safety");
    RCLCPP_WARN(get_logger(), "intent=safety_command groq text=%s", text.c_str());
    return;
  }

  if (route.category == "robot_command") {
    publish_string(pub_category_, "robot_command");
    publish_string(pub_robot_command_, route.raw_json);
    publish_string(pub_debug_, "groq_route_robot needs_vision=" +
      string(route.needs_vision ? "true" : "false"));
    RCLCPP_INFO(get_logger(), "intent=robot_command json=%s", route.raw_json.c_str());
    return;
  }

  publish_string(pub_category_, "chat");
  publish_string(pub_chat_text_, route.chat_text.empty() ? text : route.chat_text);
  publish_string(pub_debug_, "groq_route_chat");
  RCLCPP_INFO(get_logger(), "intent=chat text=%s", text.c_str());
}

IntentRouteResult IntentRouterNode::route_with_groq(const string & text) const
{
  IntentRouteResult out;
  const string prompt = build_prompt(text);

  static rover_common::CurlGlobalGuard curl_guard;
  (void)curl_guard;

  const string url = "https://api.groq.com/openai/v1/chat/completions";
  const vector<string> headers{
    "Content-Type: application/json",
    "Authorization: Bearer " + groq_api_key_};

  ostringstream body;
  body << "{"
       << "\"model\":\"" << rover_common::json_escape(groq_model_) << "\","
       << "\"temperature\":" << groq_temperature_ << ","
       << "\"max_tokens\":" << groq_max_tokens_ << ","
       << "\"messages\":["
       << "{\"role\":\"user\",\"content\":\"" << rover_common::json_escape(prompt) << "\"}"
       << "]"
       << "}";

  long http_code = 0;
  string response;
  string error;
  if (!rover_common::perform_post_json(
      url, headers, body.str(), max(1, groq_timeout_sec_), http_code, response, error))
  {
    out.error = error;
    return out;
  }
  if (http_code != 200) {
    ostringstream err;
    err << "http_" << http_code;
    if (!response.empty()) {
      err << ":" << rover_common::trim(response);
    }
    out.error = err.str();
    return out;
  }

  string content;
  if (!rover_common::extract_json_string_field(response, "content", content)) {
    out.error = "content_not_found";
    return out;
  }

  out.raw_json = find_last_json_object(content);
  if (out.raw_json.empty()) {
    out.raw_json = find_last_json_object(response);
  }
  if (out.raw_json.empty()) {
    out.error = "json_not_found: " + rover_common::trim(content);
    return out;
  }

  if (!rover_common::extract_json_string_field(out.raw_json, "category", out.category)) {
    if (!rover_common::extract_json_string_field(out.raw_json, "intent", out.category)) {
      rover_common::extract_json_string_field(out.raw_json, "type", out.category);
    }
    if (out.category.empty()) {
      if (out.raw_json.find("safety_command") != string::npos) {
        out.category = "safety_command";
      } else if (out.raw_json.find("robot_command") != string::npos) {
        out.category = "robot_command";
      } else {
        out.category = "chat";
      }
    }
  }

  if (out.category != "safety_command" &&
      out.category != "robot_command" &&
      out.category != "chat")
  {
    out.error = "invalid_category:" + out.category;
    return out;
  }

  rover_common::extract_json_string_field(out.raw_json, "command", out.command);
  rover_common::extract_json_string_field(out.raw_json, "chat_text", out.chat_text);
  if (out.chat_text.empty() && out.category == "chat") {
    out.chat_text = text;
  }
  extract_json_bool_field(out.raw_json, "needs_vision", out.needs_vision);
  out.ok = true;
  return out;
}

string IntentRouterNode::build_prompt(const string & text) const
{
  if (groq_prompt_template_.empty()) {
    return "입력: " + text + "\n출력:";
  }
  if (groq_prompt_template_.find("{input}") != string::npos) {
    return replace_all(groq_prompt_template_, "{input}", text);
  }
  return groq_prompt_template_ + "\n입력: " + text + "\n출력:";
}

bool IntentRouterNode::contains_any_keyword(
  const string & text,
  const vector<string> & keywords) const
{
  const string lowered = rover_common::to_lower(text);
  const string compact = remove_spaces(lowered);
  for (const auto & kw : keywords) {
    const string k = remove_spaces(rover_common::to_lower(kw));
    if (k.empty()) {
      continue;
    }
    if (lowered.find(k) != string::npos || compact.find(k) != string::npos) {
      return true;
    }
  }
  return false;
}

void IntentRouterNode::publish_string(
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub,
  const string & data) const
{
  if (!pub) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = data;
  pub->publish(msg);
}

string IntentRouterNode::remove_spaces(const string & value)
{
  string out;
  out.reserve(value.size());
  for (char c : value) {
    if (!isspace(static_cast<unsigned char>(c))) {
      out.push_back(c);
    }
  }
  return out;
}

}  // namespace intent_router_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = make_shared<intent_router_cpp::IntentRouterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
