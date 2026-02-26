#pragma once

#include <cstdio>
#include <string>
#include <sys/wait.h>

namespace rover_common
{

/// 셸 명령 실행 결과를 담는 구조체
struct ShellResult
{
  bool ok = false;        ///< exit_code == 0 이면 true
  int exit_code = -1;     ///< 프로세스 종료 코드 (-1이면 popen 실패)
  std::string output;     ///< stdout+stderr 캡처 내용
};

/// 셸 명령을 popen으로 실행하고 결과를 ShellResult로 반환
inline ShellResult run_shell_command(const std::string & command)
{
  ShellResult out;

  FILE * pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return out;
  }

  char buffer[512];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    out.output += buffer;
  }

  const int status = pclose(pipe);
  if (WIFEXITED(status)) {
    out.exit_code = WEXITSTATUS(status);
  } else {
    out.exit_code = status;
  }
  out.ok = out.exit_code == 0;
  return out;
}

}  // namespace rover_common
