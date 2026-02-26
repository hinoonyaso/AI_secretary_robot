# wake_vad_cpp

`wake_vad_py` migration skeleton package.

## Build

```bash
cd ~/rover_ws
colcon build --packages-select wake_vad_cpp
source ~/rover_ws/install/setup.bash
```

## Run

```bash
ros2 launch wake_vad_cpp wake_vad.launch.py
```

Current C++ files are skeletons for:
- audio input adapter
- Porcupine adapter
- VAD adapter
- state machine
- WAV writer
- ROS2 node integration
