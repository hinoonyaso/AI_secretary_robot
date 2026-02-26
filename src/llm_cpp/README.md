# llm_cpp

ROS2 C++ LLM node with provider fallback.

Fallback order:
1. Groq (`llama-3.1-8b-instant`)
2. Gemini (`gemini-2.5-flash`)
3. Ollama (`qwen2.5:1.5b`) - 1.5B / 1.2GB / 양호 / 빠름

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

## Full voice pipeline

```bash
ros2 launch llm_cpp voice_pipeline_with_llm.launch.py
```
