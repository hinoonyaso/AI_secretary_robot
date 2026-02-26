#pragma once

#include <string>

namespace rover_common
{

/// JSON 문자열 값에 들어갈 특수문자를 이스케이프 처리
inline std::string json_escape(const std::string & value)
{
  std::string out;
  out.reserve(value.size() + 16);
  for (const char c : value) {
    switch (c) {
      case '\"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(c);
        break;
    }
  }
  return out;
}

/// JSON 응답에서 특정 문자열 필드 값을 추출 (경량 파서, 외부 라이브러리 불필요)
/// "field":"value" 패턴을 찾아 이스케이프를 해제한 value를 out에 저장
inline bool extract_json_string_field(
  const std::string & json, const std::string & field, std::string & out)
{
  const std::string key = "\"" + field + "\"";
  const size_t key_pos = json.find(key);
  if (key_pos == std::string::npos) {
    return false;
  }

  const size_t colon_pos = json.find(':', key_pos + key.size());
  if (colon_pos == std::string::npos) {
    return false;
  }
  size_t start = json.find('"', colon_pos + 1);
  if (start == std::string::npos) {
    return false;
  }
  ++start;

  std::string value;
  bool escaping = false;
  for (size_t i = start; i < json.size(); ++i) {
    const char c = json[i];
    if (escaping) {
      switch (c) {
        case '"':
        case '\\':
        case '/':
          value.push_back(c);
          break;
        case 'b':
          value.push_back('\b');
          break;
        case 'f':
          value.push_back('\f');
          break;
        case 'n':
          value.push_back('\n');
          break;
        case 'r':
          value.push_back('\r');
          break;
        case 't':
          value.push_back('\t');
          break;
        default:
          value.push_back(c);
          break;
      }
      escaping = false;
      continue;
    }
    if (c == '\\') {
      escaping = true;
      continue;
    }
    if (c == '"') {
      out = value;
      return true;
    }
    value.push_back(c);
  }
  return false;
}

}  // namespace rover_common
