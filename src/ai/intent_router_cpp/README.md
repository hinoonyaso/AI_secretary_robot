# intent_router_cpp

ROS2 C++ intent router node.

- Input: `/wake_vad/transcript`
- Output:
  - `/intent_router/category` (`safety_command`, `robot_command`, `chat`)
  - `/intent_router/robot_command`
  - `/intent_router/chat_text`
  - `/intent_router/debug`

Routing policy:
- Safety bypass: keyword check before LLM call (immediate STOP)
- LLM route: `llama.cpp` (`llama-cli`) + Qwen GGUF, JSON classification
- Chat fallback: when LLM fails or returns invalid JSON

### 3.3 의도 분석 및 임베딩 (KoSimCSE-roberta)

- 110M 파라미터, KorSTS 83.65%
- 유사 명령어 매핑: "가져다줘" ≈ "줘" ≈ "전달해"

## Build

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select intent_router_cpp
source ~/rover_ws/install/setup.bash
```

## Run

```bash
ros2 launch intent_router_cpp intent_router.launch.py
```

## Full voice pipeline

```bash
ros2 launch intent_router_cpp voice_pipeline_with_router.launch.py
```

## LLM dependency

Install `llama.cpp` binary (`llama-cli`) and model:

```bash
wget https://huggingface.co/Qwen/Qwen2.5-3B-Instruct-GGUF/resolve/main/qwen2.5-3b-instruct-q4_k_m.gguf
```

Configure in `config/params.yaml`:
- `llama_cli_path` (default: `/usr/local/bin/llama-cli`)
- `llama_model_path` (default: `/home/ubuntu/rover_ws/models/qwen2.5-3b-instruct-q4_k_m.gguf`)
- `llama_n_gpu_layers` (default: `35`)
- `llama_temperature` (default: `0.1`)
- `llama_max_tokens` (default: `256`)
- `llama_timeout_ms` (default: `300`)
