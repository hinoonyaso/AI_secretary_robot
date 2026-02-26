#pragma once

#include <curl/curl.h>

#include <cstddef>
#include <string>
#include <vector>

namespace rover_common
{

/// cURL 수신 데이터를 std::string 버퍼에 누적하는 콜백
inline size_t curl_write_callback(void * contents, size_t size, size_t nmemb, void * userp)
{
  const size_t total = size * nmemb;
  auto * buffer = static_cast<std::string *>(userp);
  buffer->append(static_cast<const char *>(contents), total);
  return total;
}

/// 네트워크 장애(DNS, 연결, 타임아웃 등)에 해당하는 cURL 에러 코드인지 판별
inline bool is_network_error_code(int curl_code)
{
  return curl_code == CURLE_COULDNT_RESOLVE_HOST ||
    curl_code == CURLE_COULDNT_CONNECT ||
    curl_code == CURLE_OPERATION_TIMEDOUT ||
    curl_code == CURLE_GOT_NOTHING ||
    curl_code == CURLE_SEND_ERROR ||
    curl_code == CURLE_RECV_ERROR ||
    curl_code == CURLE_SSL_CONNECT_ERROR;
}

/// RAII 방식으로 curl_global_init/cleanup을 관리 (프로세스당 1개 static 인스턴스)
class CurlGlobalGuard
{
public:
  CurlGlobalGuard()
  {
    curl_global_init(CURL_GLOBAL_DEFAULT);
  }

  ~CurlGlobalGuard()
  {
    curl_global_cleanup();
  }

  CurlGlobalGuard(const CurlGlobalGuard &) = delete;
  CurlGlobalGuard & operator=(const CurlGlobalGuard &) = delete;
};

/// JSON 본문을 HTTP POST로 전송하고 응답을 받아오는 범용 함수
/// 성공 시 true 반환, cURL 레벨 에러 시 false + out_error 설정
inline bool perform_post_json(
  const std::string & url,
  const std::vector<std::string> & header_lines,
  const std::string & body,
  long timeout_sec,
  long & out_http_code,
  std::string & out_body,
  std::string & out_error)
{
  CURL * curl = curl_easy_init();
  if (!curl) {
    out_error = "curl_init_failed";
    return false;
  }

  struct curl_slist * headers = nullptr;
  for (const auto & h : header_lines) {
    headers = curl_slist_append(headers, h.c_str());
  }

  out_body.clear();
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out_body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);

  const CURLcode rc = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &out_http_code);

  if (rc != CURLE_OK) {
    out_error = std::string("curl_error:") + curl_easy_strerror(rc);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return false;
  }

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return true;
}

}  // namespace rover_common
