# llm_cpp

ROS2 C++ LLM node with provider fallback.

Fallback order:
1. llama.cpp local HTTP (`/v1/chat/completions`)
2. OpenAI (`gpt-4o`)
3. Groq (`llama-3.1-8b-instant`)
4. Gemini (`gemini-2.5-flash`)

llama.cpp server lifecycle is managed by `llm_node`:
- start on node init
- health check every 1 minute (`/health`)
- restart on unhealthy

Default llama.cpp runtime options:
- `-ngl 99`
- `--port 8081`
- `-c 2048`
- `-t 4`

Topics:
- Input: `/intent_router/chat_text`
- Output:
  - `/llm/response`
  - `/llm/provider`
  - `/llm/debug`

## Build

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select llm_cpp
source ~/rover_ws/install/setup.bash
```

## Run

```bash
ros2 launch llm_cpp llm.launch.py
```

## llama.cpp 준비

기본 경로:
- server binary: `/home/ubuntu/external/llama.cpp/build/bin/llama-server`
- model: `/home/ubuntu/models/qwen2.5-1.5b-instruct-q4_k_m.gguf`

필요 시 `config/params.yaml`의 `llama_*` 파라미터로 경로/포트/오프로딩을 조정하세요.

## Full voice pipeline

```bash
ros2 launch llm_cpp voice_pipeline_with_llm.launch.py
```
