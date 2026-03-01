# Codex Prompt: LLM - llama.cpp HTTP API 직접 호출 마이그레이션

**작성일**: 2026-03-01
**담당**: 10년차 로봇 SW 시니어 개발팀장
**난이도**: ★★★☆☆ (HTTP 클라이언트, 프로세스 관리)
**예상 소요**: 3~5시간

---

## 1. 배경 및 목표

### 1.1 현재 상태
- **구현**: `llm_cpp/src/llm_engine.cpp`에서 Ollama HTTP API 호출
- **파일**:
  - [src/ai/llm_cpp/src/llm_engine.cpp:40-41](../src/ai/llm_cpp/src/llm_engine.cpp#L40-L41) (Ollama 호출 로직)
  - [src/ai/llm_cpp/include/llm_cpp/llm_engine.hpp](../src/ai/llm_cpp/include/llm_cpp/llm_engine.hpp)
- **문제점**:
  - Planning 문서([docs/plan.md](./plan.md)) 설계와 불일치
  - Ollama 래퍼 의존으로 메모리 관리 제어권 상실
  - GPU 오프로딩 설정 불투명 (-ngl 99 명시 불가)
  - Ollama 데몬 장애 시 fallback 체인 동작 불가

### 1.2 목표
- **llama.cpp HTTP 서버** 직접 기동 및 호출
- Ollama 의존성 제거 → llama.cpp 바이너리 직접 실행
- GPU 전체 오프로딩 명시 (`-ngl 99`)
- API 인터페이스 유지 (`generate()` 시그니처 불변)
- 성능 투명성 확보 (토큰/초 측정 가능)

---

## 2. 기술 사양

### 2.1 llama.cpp 서버 아키텍처
```
llama.cpp Server
├── 서버 기동: ./llama-server --model model.gguf -ngl 99 --port 8081
├── HTTP API: POST /v1/chat/completions
├── 요청 형식: OpenAI Chat Completions API 호환
├── GPU 오프로딩: -ngl 99 (전체 레이어 GPU)
└── 메모리 관리: mmap, mlock 옵션 제어
```

### 2.2 Ollama vs llama.cpp 비교
| 항목 | Ollama (현재) | llama.cpp (목표) | 비고 |
|:---|:---|:---|:---|
| GPU 오프로딩 | 자동 (불투명) | `-ngl 99` 명시 | Planning 요구사항 충족 |
| 메모리 사용 | 1.2GB (Ollama 래퍼 포함) | 0.9GB (순수 모델) | 25% 절감 |
| 프로세스 관리 | systemd 의존 | ROS2 노드에서 직접 관리 | 장애 대응 용이 |
| API 형식 | Ollama 전용 | OpenAI 호환 | 표준 준수 |
| 추론 속도 | 15~25ms/토큰 | 15~25ms/토큰 (동등) | llama.cpp 기반이므로 동일 |

### 2.3 의존성
```bash
# llama.cpp 설치 (Jetson Orin Nano 기준)
cd /home/ubuntu/external/
git clone https://github.com/ggerganov/llama.cpp.git
cd llama.cpp
mkdir build && cd build
cmake .. -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87
cmake --build . --config Release -j$(nproc)

# 서버 바이너리 확인
ls -lh bin/llama-server
# 예상 크기: ~50MB
```

---

## 3. 구현 계획

### 3.1 파일 구조 변경
```
src/ai/llm_cpp/
├── include/llm_cpp/
│   ├── llm_engine.hpp              # ⚠️ 수정: LlamaCppConfig 추가
│   ├── llm_node.hpp                # ⚠️ 수정: 서버 프로세스 관리 로직 추가
│   └── llama_server_manager.hpp   # ⭐ 신규: llama.cpp 서버 생명주기 관리
├── src/
│   ├── llm_engine.cpp              # ⚠️ 수정: Ollama → llama.cpp HTTP API
│   ├── llm_node.cpp                # ⚠️ 수정: 서버 헬스체크 추가
│   └── llama_server_manager.cpp   # ⭐ 신규: 서버 시작/정지/재시작
├── scripts/
│   └── start_llama_server.sh       # ⭐ 신규: llama-server 실행 스크립트
├── config/
│   └── params.yaml                 # ⚠️ 수정: llama.cpp 서버 설정 추가
├── CMakeLists.txt                  # 변경 없음 (libcurl 이미 사용 중)
└── package.xml                     # 변경 없음
```

### 3.2 단계별 작업

#### **STEP 1: llama_server_manager.hpp 헤더 생성**
**목표**: llama.cpp 서버 프로세스 관리 클래스

```cpp
// llama_server_manager.hpp
#pragma once

#include <memory>
#include <string>

namespace llm_cpp {

struct LlamaServerConfig {
  std::string server_binary = "/home/ubuntu/external/llama.cpp/build/bin/llama-server";
  std::string model_path = "/home/ubuntu/models/qwen2.5-1.5b-instruct-q4_k_m.gguf";
  int port = 8081;
  int ngl = 99;                  // GPU 레이어 오프로딩 (99 = 전체)
  int ctx_size = 2048;           // Context 윈도우
  int n_threads = 4;             // CPU 스레드 수
  bool use_mmap = true;          // 메모리 맵 사용
  bool use_mlock = false;        // 메모리 lock (선택)
};

class LlamaServerManager {
public:
  explicit LlamaServerManager(const LlamaServerConfig& cfg);
  ~LlamaServerManager();

  bool start();                  // 서버 시작
  bool stop();                   // 서버 정지
  bool restart();                // 서버 재시작
  bool is_healthy() const;       // 헬스체크 (HTTP GET /health)

  const std::string& endpoint() const { return endpoint_; }
  const std::string& last_error() const { return last_error_; }

private:
  bool wait_for_ready(int timeout_sec = 10) const;  // 서버 준비 대기
  std::string build_command() const;                // 실행 명령 생성

  LlamaServerConfig config_;
  pid_t server_pid_ = -1;        // 서버 프로세스 PID
  std::string endpoint_;         // "http://localhost:8081"
  mutable std::string last_error_;
};

}  // namespace llm_cpp
```

---

#### **STEP 2: llama_server_manager.cpp 구현**
**목표**: 서버 생명주기 관리 로직

```cpp
// llama_server_manager.cpp
#include "llm_cpp/llama_server_manager.hpp"

#include <rover_common/curl_utils.hpp>

#include <curl/curl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <sstream>
#include <thread>

using namespace std;

namespace llm_cpp {

LlamaServerManager::LlamaServerManager(const LlamaServerConfig& cfg)
: config_(cfg) {
  ostringstream oss;
  oss << "http://localhost:" << config_.port;
  endpoint_ = oss.str();
}

LlamaServerManager::~LlamaServerManager() {
  stop();
}

bool LlamaServerManager::start() {
  if (server_pid_ > 0) {
    last_error_ = "Server already running (PID " + to_string(server_pid_) + ")";
    return false;
  }

  const string cmd = build_command();

  // fork + exec로 서버 프로세스 실행
  server_pid_ = fork();
  if (server_pid_ < 0) {
    last_error_ = "Failed to fork server process";
    return false;
  }

  if (server_pid_ == 0) {
    // 자식 프로세스: llama-server 실행
    execl("/bin/sh", "sh", "-c", cmd.c_str(), nullptr);
    _exit(1);  // exec 실패 시
  }

  // 부모 프로세스: 서버 준비 대기
  if (!wait_for_ready(10)) {
    stop();
    last_error_ = "Server failed to start within 10 seconds";
    return false;
  }

  return true;
}

bool LlamaServerManager::stop() {
  if (server_pid_ <= 0) {
    return true;  // 이미 정지됨
  }

  // SIGTERM 전송 (graceful shutdown)
  kill(server_pid_, SIGTERM);

  // 최대 5초 대기
  for (int i = 0; i < 50; ++i) {
    int status;
    pid_t result = waitpid(server_pid_, &status, WNOHANG);
    if (result == server_pid_) {
      server_pid_ = -1;
      return true;
    }
    this_thread::sleep_for(chrono::milliseconds(100));
  }

  // 강제 종료 (SIGKILL)
  kill(server_pid_, SIGKILL);
  waitpid(server_pid_, nullptr, 0);
  server_pid_ = -1;
  return true;
}

bool LlamaServerManager::restart() {
  stop();
  this_thread::sleep_for(chrono::milliseconds(500));
  return start();
}

bool LlamaServerManager::is_healthy() const {
  if (server_pid_ <= 0) {
    return false;
  }

  // HTTP GET /health 요청
  CURL* curl = curl_easy_init();
  if (!curl) return false;

  const string health_url = endpoint_ + "/health";
  string response;

  curl_easy_setopt(curl, CURLOPT_URL, health_url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, rover_common::curl_write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_easy_cleanup(curl);

  return (res == CURLE_OK && http_code == 200);
}

bool LlamaServerManager::wait_for_ready(int timeout_sec) const {
  const auto deadline = chrono::steady_clock::now() + chrono::seconds(timeout_sec);
  while (chrono::steady_clock::now() < deadline) {
    if (is_healthy()) {
      return true;
    }
    this_thread::sleep_for(chrono::milliseconds(500));
  }
  return false;
}

string LlamaServerManager::build_command() const {
  ostringstream cmd;
  cmd << config_.server_binary
      << " --model " << config_.model_path
      << " --port " << config_.port
      << " -ngl " << config_.ngl
      << " -c " << config_.ctx_size
      << " -t " << config_.n_threads;

  if (config_.use_mmap) {
    cmd << " --mmap";
  }
  if (config_.use_mlock) {
    cmd << " --mlock";
  }

  cmd << " 2>&1 | tee /tmp/llama_server_" << config_.port << ".log";
  return cmd.str();
}

}  // namespace llm_cpp
```

**핵심 로직**:
1. `fork()` + `execl()`로 llama-server 프로세스 시작
2. `/health` 엔드포인트로 준비 상태 대기
3. `SIGTERM` → `SIGKILL` 순서로 graceful shutdown
4. PID 기반 프로세스 관리

---

#### **STEP 3: llm_engine.cpp 수정**
**목표**: Ollama 호출 → llama.cpp HTTP API 호출

```cpp
// llm_engine.hpp (Before)
class LlmEngine {
private:
  LlmResult generate_ollama(const std::string& user_text) const;
  LlmConfig config_;
};

// llm_engine.hpp (After)
#include "llm_cpp/llama_server_manager.hpp"

class LlmEngine {
public:
  explicit LlmEngine(
      const LlmConfig& config,
      std::shared_ptr<LlamaServerManager> server_mgr);  // ⭐ 신규 파라미터

private:
  LlmResult generate_llamacpp(const std::string& user_text) const;  // ⭐ 신규
  LlmConfig config_;
  std::shared_ptr<LlamaServerManager> server_mgr_;
};
```

```cpp
// llm_engine.cpp (generate 메서드)
LlmResult LlmEngine::generate(const std::string& user_text) const {
  LlmResult out;
  if (!config_.enabled) {
    out.error = "llm disabled";
    return out;
  }

  // 1차: llama.cpp (로컬)
  if (server_mgr_ && server_mgr_->is_healthy()) {
    LlmResult local = generate_llamacpp(user_text);
    if (local.ok) {
      return local;
    }
  }

  // 2차: OpenAI (fallback)
  LlmResult openai = generate_openai(user_text);
  openai.used_fallback = true;
  if (openai.ok) {
    return openai;
  }

  // 3차: Groq (fallback)
  LlmResult groq = generate_groq(user_text);
  groq.used_fallback = true;
  if (groq.ok) {
    return groq;
  }

  // 최종 실패
  LlmResult merged;
  merged.ok = false;
  merged.provider = "llamacpp+openai+groq";
  merged.error = "All providers failed";
  return merged;
}
```

```cpp
// llm_engine.cpp (generate_llamacpp 구현)
LlmResult LlmEngine::generate_llamacpp(const std::string& user_text) const {
  LlmResult out;
  out.provider = "llama.cpp";

  if (!server_mgr_) {
    out.error = "llama.cpp server manager not initialized";
    return out;
  }

  CURL* curl = curl_easy_init();
  if (!curl) {
    out.error = "curl_init_failed";
    return out;
  }

  // OpenAI Chat Completions API 호환 요청
  ostringstream json_req;
  json_req << R"({
    "model": "qwen2.5-1.5b-instruct",
    "messages": [{"role": "user", "content": ")" << user_text << R"("}],
    "temperature": 0.7,
    "max_tokens": 512
  })";

  string response_body;
  const string url = server_mgr_->endpoint() + "/v1/chat/completions";

  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_req.str().c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, rover_common::curl_write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

  CURLcode rc = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  if (rc != CURLE_OK) {
    out.error = "curl_error: " + string(curl_easy_strerror(rc));
  } else if (http_code != 200) {
    out.error = "http_" + to_string(http_code);
  } else {
    // JSON 파싱: choices[0].message.content
    string content;
    if (rover_common::extract_openai_chat_content(response_body, content)) {
      out.ok = true;
      out.text = content;
    } else {
      out.error = "invalid_json_response";
    }
  }

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return out;
}
```

---

#### **STEP 4: llm_node.cpp 수정**
**목표**: 노드 초기화 시 llama.cpp 서버 자동 시작

```cpp
// llm_node.cpp (생성자)
LlmNode::LlmNode(const rclcpp::NodeOptions& options)
: Node("llm_node", options) {
  // 1. llama.cpp 서버 설정 로드
  LlamaServerConfig server_cfg;
  server_cfg.server_binary = this->declare_parameter<std::string>(
      "llama_server_binary",
      "/home/ubuntu/external/llama.cpp/build/bin/llama-server");
  server_cfg.model_path = this->declare_parameter<std::string>(
      "llama_model_path",
      "/home/ubuntu/models/qwen2.5-1.5b-instruct-q4_k_m.gguf");
  server_cfg.port = this->declare_parameter<int>("llama_port", 8081);
  server_cfg.ngl = this->declare_parameter<int>("llama_ngl", 99);
  server_cfg.ctx_size = this->declare_parameter<int>("llama_ctx_size", 2048);
  server_cfg.n_threads = this->declare_parameter<int>("llama_threads", 4);

  // 2. 서버 매니저 생성 및 시작
  server_mgr_ = std::make_shared<LlamaServerManager>(server_cfg);
  if (!server_mgr_->start()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start llama.cpp server: %s",
                 server_mgr_->last_error().c_str());
    // Fallback 체인이 있으므로 치명적 에러는 아님
  } else {
    RCLCPP_INFO(this->get_logger(), "llama.cpp server started at %s",
                server_mgr_->endpoint().c_str());
  }

  // 3. LlmEngine 생성 (서버 매니저 전달)
  LlmConfig engine_cfg;
  // ... 기존 설정 로드 ...
  llm_engine_ = std::make_unique<LlmEngine>(engine_cfg, server_mgr_);

  // 4. 헬스체크 타이머 (1분마다)
  health_timer_ = this->create_wall_timer(
      std::chrono::minutes(1),
      [this]() {
        if (server_mgr_ && !server_mgr_->is_healthy()) {
          RCLCPP_WARN(this->get_logger(), "llama.cpp server unhealthy, restarting...");
          server_mgr_->restart();
        }
      });
}

LlmNode::~LlmNode() {
  if (server_mgr_) {
    server_mgr_->stop();
  }
}
```

---

#### **STEP 5: config/params.yaml 수정**
**목표**: llama.cpp 서버 파라미터 추가

```yaml
# config/params.yaml
llm_node:
  ros__parameters:
    # llama.cpp 서버 설정 (⭐ 신규)
    llama_server_binary: "/home/ubuntu/external/llama.cpp/build/bin/llama-server"
    llama_model_path: "/home/ubuntu/models/qwen2.5-1.5b-instruct-q4_k_m.gguf"
    llama_port: 8081
    llama_ngl: 99              # GPU 전체 레이어 오프로딩
    llama_ctx_size: 2048       # Context 윈도우
    llama_threads: 4           # CPU 스레드 수

    # 기존 fallback 설정 유지
    openai_api_key: ""
    groq_api_key: ""
    gemini_api_key: ""
```

---

#### **STEP 6: 검증 테스트**
**목표**: 서버 관리 및 추론 성능 확인

```bash
# 1. 빌드
cd ~/dev_ws/AI_secretary_robot
colcon build --packages-select llm_cpp

# 2. 노드 실행
ros2 launch llm_cpp llm.launch.py

# 기대 출력 (로그):
# [INFO] [llm_node]: llama.cpp server started at http://localhost:8081
# [INFO] [llm_node]: LLM engine ready (provider: llama.cpp)

# 3. 추론 테스트 (별도 터미널)
ros2 topic pub /intent_router/chat_text std_msgs/String \
  "data: '안녕하세요'" --once

# 기대 출력:
# /llm/response: "안녕하세요! 무엇을 도와드릴까요?"
# /llm/provider: "llama.cpp"
# 추론 시간: 15~25ms/토큰

# 4. 서버 헬스체크 (수동)
curl http://localhost:8081/health
# 기대 출력: {"status":"ok"}

# 5. GPU 사용량 확인
nvidia-smi
# 기대: GPU-Util ~30%, Memory-Usage 1.2GB (VRAM)

# 6. 프로세스 확인
ps aux | grep llama-server
# 기대: llama-server 프로세스 실행 중, -ngl 99 인자 확인
```

---

## 4. 위험 관리

### 4.1 주요 리스크
| 리스크 | 완화 방안 |
|:---|:---|
| llama.cpp 빌드 실패 | CUDA 11.4+ 확인, CMake 옵션 검증 |
| 서버 시작 실패 | 로그 파일 확인 (`/tmp/llama_server_8081.log`) |
| 포트 충돌 | `netstat -tuln \| grep 8081` 확인, 포트 변경 |
| 메모리 부족 | GGUF 모델 크기 확인 (Q4_K_M < 1.5GB) |
| 서버 크래시 | 헬스체크 타이머 + 자동 재시작 구현됨 |

### 4.2 롤백 계획
```bash
# Ollama로 복구
# 1. params.yaml 수정
llm_node:
  ros__parameters:
    provider: "ollama"  # ← "auto"에서 변경

# 2. Ollama 서버 시작
systemctl start ollama

# 3. 노드 재시작
ros2 launch llm_cpp llm.launch.py
```

---

## 5. 완료 조건

### 5.1 기능 검증
- ✅ llama.cpp 서버 자동 시작/정지
- ✅ `/llm/response` 정확도 동등 이상
- ✅ GPU 오프로딩 확인 (`-ngl 99` 적용, VRAM 1.2GB)
- ✅ 추론 속도 동등 이상 (15~25ms/토큰)

### 5.2 코드 품질
- ✅ `LlmEngine` API 인터페이스 불변
- ✅ 서버 크래시 시 자동 재시작 (헬스체크 타이머)
- ✅ 메모리 누수 없음 (서버 프로세스 종료 확인)

### 5.3 문서화
- ✅ `src/ai/llm_cpp/README.md` 업데이트 (llama.cpp 빌드 가이드)
- ✅ `AGENTS.md`에 변경 로그 추가

---

## 6. 참고 자료

### 6.1 공식 문서
- llama.cpp HTTP 서버: https://github.com/ggerganov/llama.cpp/blob/master/examples/server/README.md
- OpenAI Chat Completions API: https://platform.openai.com/docs/api-reference/chat

### 6.2 모델 다운로드
```bash
# Qwen2.5-1.5B-Instruct GGUF (Q4_K_M)
cd /home/ubuntu/models/
wget https://huggingface.co/Qwen/Qwen2.5-1.5B-Instruct-GGUF/resolve/main/qwen2.5-1.5b-instruct-q4_k_m.gguf
```

---

## 7. 최종 체크리스트

**마이그레이션 전**:
- [ ] llama.cpp 바이너리 빌드 완료
- [ ] Qwen2.5-1.5B GGUF 모델 다운로드
- [ ] 포트 8081 사용 가능 확인 (`netstat -tuln`)
- [ ] 현재 브랜치 백업 (`git checkout -b backup/llm-before-migration`)

**마이그레이션 중**:
- [ ] `llama_server_manager.hpp`, `llama_server_manager.cpp` 생성
- [ ] `llm_engine.cpp` 수정 (Ollama → llama.cpp)
- [ ] `llm_node.cpp` 수정 (서버 자동 시작)
- [ ] `params.yaml` 수정 (llama.cpp 설정 추가)

**마이그레이션 후**:
- [ ] 빌드 성공 (`colcon build --packages-select llm_cpp`)
- [ ] 노드 실행 성공 (서버 시작 로그 확인)
- [ ] 추론 테스트 통과 (정확도, 속도)
- [ ] GPU 사용량 확인 (`nvidia-smi`)
- [ ] 서버 재시작 테스트 (kill -9 후 자동 복구)
- [ ] 문서 업데이트 (`README.md`, `AGENTS.md`)

**최종 승인**:
- [ ] 팀장 코드 리뷰 통과
- [ ] GPU 오프로딩 검증 (`-ngl 99` 적용 확인)
- [ ] Planning 문서([docs/plan.md](./plan.md))와 일치 확인

---

**작성자 노트**:
llama.cpp는 Ollama 대비 투명성과 제어권을 제공합니다. `-ngl 99`로 전체 GPU 오프로딩을 명시하면 Planning 문서의 설계 의도를 정확히 구현할 수 있습니다. 서버 프로세스 관리는 ROS2 노드 생명주기와 통합하여 안정성을 확보하세요. Ollama는 fallback 체인에 남겨두어도 무방하지만, Planning 문서의 아키텍처 일관성을 위해 완전히 제거하는 것을 권장합니다.
