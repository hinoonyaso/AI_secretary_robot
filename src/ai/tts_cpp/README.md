# tts_cpp

ROS2 C++ TTS node with fallback order:
1. edge-tts (`ko-KR-SunHiNeural`)
2. Piper TTS (local ONNX)
3. espeak-ng (local fallback)

Topics:
- Input: `/llm/response`
- Output:
  - `/tts/audio_path`
  - `/tts/engine`
  - `/tts/debug`

Playback:
- By default, `tts_node` now auto-plays synthesized audio.
- If `playback_command` is empty, it tries:
  - WAV: `aplay` -> `ffplay`
  - MP3: `ffplay` -> `mpg123`
- You can override with `playback_command` (supports `{audio_path}` placeholder).

## Build

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tts_cpp
source ~/rover_ws/install/setup.bash
```

## Run

```bash
ros2 launch tts_cpp tts.launch.py
```

## Full voice pipeline

```bash
ros2 launch tts_cpp voice_pipeline_with_tts.launch.py
```

## Dependencies

Install edge-tts once:

```bash
python3 -m pip install --user edge-tts
```

Local playback tools (install at least one):

```bash
sudo apt-get install -y alsa-utils ffmpeg mpg123
```
