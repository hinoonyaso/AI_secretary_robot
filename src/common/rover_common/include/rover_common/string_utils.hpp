#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace rover_common
{

/// 문자열 전체를 소문자로 변환
inline std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

/// 문자열 양 끝 공백 제거
inline std::string trim(const std::string & value)
{
  size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
    ++start;
  }
  size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
    --end;
  }
  return value.substr(start, end - start);
}

/// 셸 명령용 작은따옴표 이스케이프 (예: "it's" → "'it'\''s'")
inline std::string shell_escape_single_quote(const std::string & value)
{
  std::string out = "'";
  for (char c : value) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out.push_back(c);
    }
  }
  out += "'";
  return out;
}

}  // namespace rover_common
