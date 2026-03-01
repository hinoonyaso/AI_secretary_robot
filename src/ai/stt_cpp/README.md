# stt_cpp

ROS2 C++ STT node package.

- Subscribe: `/wake_vad/audio_path`
- Publish: `/wake_vad/transcript`
- Primary: Groq Whisper (`whisper-large-v3-turbo`)
- Fallback: local `Moonshine` (`moonshine-tiny-ko`)
- Local Moonshine backend: `sherpa-onnx` C API (`moonshine_onnx` engine)

Local STT model:
- `moonshine-tiny-ko` | 26M | 0.5GB | CER 6.46% | 5~15x | ✅ 최선

## Build

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select stt_cpp
source ~/rover_ws/install/setup.bash
```

## sherpa-onnx 설치(요약)

```bash
git clone https://github.com/k2-fsa/sherpa-onnx.git
cd sherpa-onnx
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DSHERPA_ONNX_ENABLE_GPU=ON ..
make -j$(nproc)
sudo make install
```

## Run

```bash
ros2 launch stt_cpp stt.launch.py
```
