# JetRover "Rover" — 경량 Docker 컨테이너 구축 Codex 프롬프트 모음

> **실행 순서**: Prompt 1 → 2 → 3 → 4 → 5 → 6 (순서대로 진행)
> **목표**: 15GB `l4t-pytorch` 이미지 제거, 1~2GB 경량 Ubuntu+ROS2 이미지로 전환
> **환경**: Jetson Orin Nano 8GB / JetPack 6.x / CUDA 12.2 / ARM64 / Ubuntu 22.04

---

## 📦 Prompt 1: Dockerfile — 기반 레이어 (Base + System Deps + ROS 2 Humble)

```
[Context]
- Target hardware: NVIDIA Jetson Orin Nano 8GB (ARM64/aarch64)
- JetPack version: 6.x (Ubuntu 22.04, CUDA 12.2, cuDNN 8.9, TensorRT 8.6)
- Goal: Replace the heavy `nvcr.io/nvidia/l4t-pytorch:r36.3.0-pth2.4-py3` image (15GB)
         with a minimal base that has zero PyTorch dependency.
- Output file: `docker/Dockerfile` (multi-stage build, stage name: `base`)

[Critical Design Decision]
ROS 2 Humble은 반드시 이 첫 번째 스테이지(base)에 설치해야 합니다.
이유:
  1. 이후 스테이지(Prompt 2~4)에서 빌드하는 C++ 패키지들이 rclcpp, std_msgs 등
     ROS 2 헤더/라이브러리에 의존합니다. CMakeLists.txt의 find_package(rclcpp)가
     base 이미지에 ROS 2가 없으면 실패합니다.
  2. 컨테이너 안의 모든 AI 노드(STT, LLM, TTS, Vision)는 ROS 2 토픽으로 서로
     통신합니다. ROS 2 DDS(FastDDS)가 base부터 설치되어야 네트워크 설정이 통일됩니다.
  3. host ROS 2 Humble과 컨테이너 내부가 동일한 ROS_DOMAIN_ID(42)로 통신하려면
     동일한 DDS 구현체(rmw_fastrtps_cpp)가 base에서부터 환경변수로 고정되어야 합니다.

[Task]
Write the FIRST STAGE of a multi-stage Dockerfile for Jetson Orin Nano.

Requirements:
1. FROM: Use `nvcr.io/nvidia/l4t-base:r36.4.3` as the base image
   (NGC ARM64 base image for JetPack 6.0+, ~0.8GB, no PyTorch, no CUDA devel)
   ⚠️ 태그 안정성 주의: r36.4.3 태그가 NGC에서 가끔 삭제되는 사례가 있습니다.
   Dockerfile 최상단에 ARG로 fallback을 반드시 구현하세요:
     ARG L4T_TAG=r36.4.3
     FROM nvcr.io/nvidia/l4t-base:${L4T_TAG}
   빌드 시 fallback: docker build --build-arg L4T_TAG=r36.3.0 ...
   Alternative (includes CUDA devel tools): `nvcr.io/nvidia/l4t-cuda:12.2.12-devel`
   Best alternative (ROS 2 already included): `dustynv/ros:humble-ros-base-l4t-r36.3.0`
   NOTE: If using dustynv base, skip Block B entirely.

2. System packages to install (apt-get, no recommended, clean cache):
   - Build tools: cmake, ninja-build, build-essential, git, wget, curl, pkg-config
   - Audio: libasound2-dev, libpulse-dev, alsa-utils, portaudio19-dev
   - Networking: libcurl4-openssl-dev, libssl-dev
   - Utilities: python3-pip, python3-dev, sqlite3, libsqlite3-dev
   - Media: ffmpeg, libavcodec-dev, libavformat-dev, libswresample-dev
   - USB/devices: libusb-1.0-0-dev, udev
   - OpenCV (no contrib): libopencv-dev
   - ROS 2 colcon build tools: python3-colcon-common-extensions, python3-rosdep

3. [Block B] ROS 2 Humble 설치 (apt, ARM64):
   a. Add ROS 2 APT repository:
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
      echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list

   b. Install ROS 2 Humble packages (ros-base, NOT desktop):
      - ros-humble-ros-base           # rclcpp, std_msgs, geometry_msgs 등 핵심
      - ros-humble-rmw-fastrtps-cpp   # DDS 구현체 (FastDDS)
      - ros-humble-ament-cmake        # colcon 빌드 시스템
      - ros-humble-ament-index-cpp    # 패키지 인덱스 C++ API
      - ros-humble-tf2-ros            # 좌표계 변환 (MoveIt, Nav2 필수)
      - ros-humble-sensor-msgs        # Image, PointCloud2, Imu 메시지
      - ros-humble-geometry-msgs      # Pose, Twist, Point 메시지
      - ros-humble-nav-msgs           # OccupancyGrid (SLAM 지도)
      - ros-humble-action-msgs        # 액션 인터페이스
      - ros-humble-lifecycle-msgs     # 노드 수명주기 관리
      NOTE: ros-humble-moveit, ros-humble-nav2-bringup는 Prompt 4에서 추가

   c. Initialize rosdep:
      rosdep init && rosdep update --rosdistro humble

   d. Source ROS 2 in /etc/bash.bashrc:
      echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

4. Environment variables to set:
   - CUDA_HOME=/usr/local/cuda
   - PATH=$PATH:$CUDA_HOME/bin
   - LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64
   - DEBIAN_FRONTEND=noninteractive
   - LANG=ko_KR.UTF-8
   - ROS_DISTRO=humble
   - AMENT_PREFIX_PATH=/opt/ros/humble
   - ROS_PYTHON_VERSION=3
   - RMW_IMPLEMENTATION=rmw_fastrtps_cpp      # host와 동일한 DDS 구현체
   - ROS_DOMAIN_ID=42                          # host ROS 2와 동일한 도메인 ID
   - FASTRTPS_DEFAULT_PROFILES_FILE=/opt/rover/config/fastdds.xml

5. Install Korean locale support (locales package, generate ko_KR.UTF-8)

6. Create directory structure inside the image:
   /opt/rover/models/stt/       # Moonshine ONNX models
   /opt/rover/models/embedding/ # KoSimCSE ONNX models
   /opt/rover/models/llm/       # GGUF models (bind-mounted from SSD at runtime)
   /opt/rover/models/tts/       # Piper TTS ONNX voices
   /opt/rover/models/vision/    # YOLO11n TensorRT engine
   /opt/rover/db/               # SQLite database
   /opt/rover/ws/               # ROS 2 workspace (colcon install 결과물 mount)
   /opt/rover/scripts/          # Runtime scripts
   /opt/rover/config/           # FastDDS XML, ALSA 설정 등

7. [Block C] FastDDS XML 설정 파일 작성 (/opt/rover/config/fastdds.xml):
   목적: 컨테이너 내부 ↔ 호스트 ROS 2 Humble 간 DDS 통신 활성화
   설정 내용:
   - transport: UDPv4 (network_mode: host 사용 시 자동으로 host 네트워크 공유)
   - participant name: "rover_container"
   - builtin discovery: SIMPLE (기본값 유지)
   - history depth: 10 (일반 토픽), 1 (센서 토픽)
   예시 구조:
     <profiles>
       <participant profile_name="default_profile" is_default_profile="true">
         <rtps>
           <name>rover_container</name>
           <builtin><discovery_config>...</discovery_config></builtin>
         </rtps>
       </participant>
     </profiles>

8. Add a build ARG: `JETPACK_VERSION=36.4.3`

[Constraints]
- NO pip install torch, torchvision, torchaudio — zero PyTorch allowed
- NO conda
- Use --no-install-recommends for all apt-get
- RUN layers must be merged where possible to minimize layer count
- Add inline comments explaining each major block
- ROS 2는 ros-humble-ros-base만 설치 (desktop, rviz, rqt 금지 — 용량 절감)
- Final image layer must not exceed 600MB beyond the base image (ROS 2 포함)

[Verification step — RUN layer로 포함할 것]
RUN bash -c "source /opt/ros/humble/setup.bash && \
             ros2 --version && \
             python3 -c 'import rclpy; print(\"rclpy OK\")'"

[Output format]
- Single Dockerfile content for stage `base`
- Include a comment block at the top with image size estimate
- Include `LABEL` metadata: version, maintainer, description
- Include /opt/rover/config/fastdds.xml content in a heredoc within the Dockerfile
```

---

## 🧠 Prompt 2: Dockerfile — AI 엔진 레이어 (ONNX Runtime + llama.cpp)

```
[Context — 이전 Prompt 1의 결과물 위에 이어서 작성]
- Previous stage: `base` (l4t-base:r36.4.3 + system deps + ROS 2 Humble)
- This stage adds the AI inference runtimes (NO Python ML frameworks, NO PyTorch)
- Output: adds stage `ai-runtime` to `docker/Dockerfile`

[Task]
Write the SECOND STAGE (`FROM base AS ai-runtime`) of the multi-stage Dockerfile.

Block A — ONNX Runtime GPU (ARM64 / JetPack 6):
  ⚠️ IMPORTANT: ORT 1.17+ 소스 빌드는 CMake 3.26+가 필요합니다.
     jammy 기본 CMake(3.22.1)이므로 Kitware APT로 먼저 업그레이드해야 합니다.

  1. TensorRT 런타임/개발 패키지 사전 설치:
     apt-get install -y --no-install-recommends \
       libnvinfer8 libnvinfer-dev libnvinfer-plugin8 libnvinfer-plugin-dev \
       libnvparsers8 libnvparsers-dev libnvonnxparsers8 libnvonnxparsers-dev

  2. CMake 업그레이드 (Kitware APT):
     wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
       | gpg --dearmor > /usr/share/keyrings/kitware-archive-keyring.gpg
     echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' \
       > /etc/apt/sources.list.d/kitware.list
     apt-get update && apt-get install -y --no-install-recommends cmake
     cmake --version   # 3.26+ 확인

  3. dusty-nv/jetson-containers로 ORT wheel 자동 빌드:
     git clone https://github.com/dusty-nv/jetson-containers /tmp/jc
     cd /tmp/jc/packages/ml/onnxruntime
     export UV_SYSTEM_PYTHON=1 ONNXRUNTIME_VERSION=1.17.0 ONNXRUNTIME_BRANCH=v1.17.0 ONNXRUNTIME_FLAGS=--allow_running_as_root CUDA_ARCHITECTURES=87
     ./build.sh --use-cache --no-deps

  4. ⚠️ --no-deps 이후 필수 의존성 수동 설치 (없으면 import 시 ModuleNotFoundError):
     pip3 install \
       numpy==1.24.4 \        # ort numpy backend (1.25+ 와 호환성 이슈 있음)
       onnx==1.15.0 \         # onnx protobuf model 로딩용
       protobuf==3.20.3 \     # onnx 의존성 (4.x와 충돌 주의)
       coloredlogs \          # ort 런타임 로깅
       flatbuffers            # ort 내부 직렬화
     # torch, torchvision은 절대 포함하지 말 것

  5. Verify:
     python3 -c "import onnxruntime as ort; print('SUCCESS:', ort.__version__, ort.get_device())"
     → 반드시 'GPU' 출력 확인 (CPU면 wheel이 잘못된 것)

  6. Cleanup: rm -rf /tmp/jc /root/.cache/pip

  7. Environment variables:
     ORT_TENSORRT_ENGINE_CACHE_ENABLE=1
     ORT_TENSORRT_ENGINE_CACHE_PATH=/opt/rover/models/.trt_cache

Block B — llama.cpp (CUDA backend, GGUF support):
  ⚠️ IMPORTANT: v0.0.1은 오래된 태그입니다. 최신 stable 커밋을 사용하세요.

  1. Clone llama.cpp and checkout latest stable:
     git clone https://github.com/ggerganov/llama.cpp /tmp/llama.cpp
     cd /tmp/llama.cpp
     # 최신 stable tag 자동 선택 (예: b3670 이상):
     LATEST_TAG=$(git tag --sort=-version:refname | grep '^b[0-9]' | head -1)
     git checkout $LATEST_TAG

  2. Build with CMake (Jetson Orin = Ampere SM87):
     cmake -B build \
           -DGGML_CUDA=ON \
           -DCMAKE_CUDA_ARCHITECTURES="87" \
           -DLLAMA_CURL=ON \
           -DCMAKE_BUILD_TYPE=Release \
           -DLLAMA_NATIVE=OFF \
           -DGGML_CUDA_FORCE_MMQ=ON \
           -DGGML_CUDA_ENABLE_UNIFIED_MEMORY=ON
     # ↑ Jetson 핵심 최적화: Orin은 CPU/GPU가 통합 메모리(Unified Memory)를 공유합니다.
     #   이 옵션을 켜면 llama.cpp가 CPU↔GPU 간 메모리 복사를 생략하고
     #   단일 메모리 풀을 직접 참조 → mmap + SIGSTOP/SIGCONT 조합과 시너지 극대화
     cmake --build build --config Release -j$(nproc)

  3. Install binaries to /usr/local/bin/:
     llama-server, llama-cli, llama-embedding

  4. Install llama.cpp C shared library to /usr/local/lib/:
     libllama.so, libggml.so, libggml-cuda.so (from build/src/ and build/ggml/src/)

  5. Install C headers to /usr/local/include/llama/

  6. Run ldconfig && verify: llama-cli --version must succeed

  7. Cleanup: rm -rf /tmp/llama.cpp (source 삭제로 레이어 크기 최소화)

Block C — Piper TTS (C++ ONNX-based TTS, ~50MB RAM):
  1. Download piper pre-built binary for aarch64:
     https://github.com/rhasspy/piper/releases/
     Target: piper_linux_aarch64.tar.gz
  2. Extract to /opt/piper/
  3. Symlink: /usr/local/bin/piper -> /opt/piper/piper
  4. Download Korean voice model:
     ko_KR-kss-medium.onnx + ko_KR-kss-medium.onnx.json
     Place in /opt/rover/models/tts/
  5. Test (build-time smoke test):
     echo "안녕하세요" | piper --model /opt/rover/models/tts/ko_KR-kss-medium.onnx \
       --output_file /tmp/test.wav && rm /tmp/test.wav

[Constraints]
- CUDA_ARCHITECTURES must be "87" — Jetson Orin (Ampere SM). DO NOT use "8.7" format.
- llama.cpp must be built with -DLLAMA_CURL=ON for HTTP model serving
- Do NOT install Python bindings for llama.cpp (use llama-server HTTP API only)
- All downloads must use wget with --progress=dot:giga
- Add sha256 checksum verification for Piper binary
- Source build directories must be deleted after install to minimize layer size

[Output format]
- Dockerfile content for stage `ai-runtime` only
- Each block (A, B, C) in a separate RUN layer group with comments
- Include a HEALTHCHECK that tests:
  llama-cli --version && python3 -c "import onnxruntime as ort; assert ort.get_device()=='GPU'"
```

---

## 👁️ Prompt 3: Dockerfile — 비전 레이어 (TensorRT C++ Runtime + YOLO11n 래퍼)

```
[Context — Prompt 2의 `ai-runtime` 스테이지 위에 이어서 작성]
- Previous stage: `ai-runtime` (ONNX Runtime + llama.cpp + Piper)
- This stage adds TensorRT C++ runtime for YOLO11n real-time object detection
- Output: adds stage `vision-runtime` to `docker/Dockerfile`
- Key constraint: YOLO11n runs on TensorRT (Tensor Cores), consuming 0 RAM (GPU only)

⚠️ CRITICAL DESIGN DECISION — YOLO11n Engine 변환은 이 이미지에서 하지 않습니다:
  - Ultralytics YOLO export(format="engine")는 내부적으로 PyTorch를 필수로 요구합니다.
  - PyTorch를 이 이미지에 설치하면 15GB 문제가 재발합니다.
  - 해결책: TRT 엔진 변환(.pt → .engine)은 일회성 작업으로,
    Prompt 6의 first_run_setup.sh에서 별도의 일회용 NGC 컨테이너
    (nvcr.io/nvidia/l4t-pytorch:r36.x.x-pth2.x-py3)를 띄워 수행합니다.
  - 이 이미지는 이미 변환 완료된 .engine 파일을 로딩하는 C++ 코드만 포함합니다.

[Task]
Write the THIRD STAGE (`FROM ai-runtime AS vision-runtime`) of the multi-stage Dockerfile.

Block A — TensorRT C++ 개발 헤더 설치 (JetPack 6 내장 TRT 활용):
  JetPack 6.x는 호스트에 TensorRT 8.6이 이미 설치되어 있습니다.
  컨테이너 안에서 C++ 코드를 컴파일하려면 헤더와 링크 라이브러리만 필요합니다.

  1. apt-get install (no recommended):
     - libnvinfer8                   # TensorRT 런타임 라이브러리
     - libnvinfer-dev                # C++ 헤더 (nvinfer.h 등)
     - libnvinfer-plugin8            # TRT 플러그인 런타임
     - libnvinfer-plugin-dev         # 플러그인 헤더
     - libnvonnxparsers8             # ONNX → TRT 파서 (선택적)
     - libnvonnxparsers-dev
     - cuda-cupti-dev-12-2           # CUDA profiling (선택적)

  2. Verify TRT headers exist:
     test -f /usr/include/aarch64-linux-gnu/NvInfer.h && echo "TRT headers OK"

  3. Verify TRT version: /usr/lib/aarch64-linux-gnu/libnvinfer.so.8 should exist

  NOTE: pip install tensorrt 또는 pip install ultralytics는 절대 하지 마세요.
        PyTorch 의존성이 딸려 들어옵니다.

Block B — YOLO11n C++ 추론 래퍼 (TensorRT 직접 호출, PyTorch 완전 배제):
  Write a C++ header-only library at /opt/rover/include/yolo_trt.hpp that:
  1. Loads yolo11n_fp16.engine using TensorRT C++ API (NvInfer.h, NvInferRuntime.h)
  2. Engine path taken from env var: YOLO_ENGINE (default: /opt/rover/models/vision/yolo11n_fp16.engine)
  3. Exposes a simple inference function:
     struct Detection { float x, y, w, h, conf; int class_id; std::string class_name; };
     class YoloTRT {
       public:
         YoloTRT(const std::string& engine_path);
         std::vector<Detection> detect(const uint8_t* rgb_data, int width, int height,
                                        float conf_thresh = 0.5f, float nms_thresh = 0.45f);
       private:
         // TensorRT engine, context, CUDA buffers (pre-allocated at init)
     };
  4. Uses CUDA streams for async inference (cudaStreamCreate)
  5. Pre-allocates input/output CUDA buffers at constructor time (no per-frame malloc)
  6. Implements NMS (Non-Maximum Suppression) in CPU after GPU inference
  7. Target latency: < 10ms per 640x640 frame on Jetson Orin (125 FPS)
  8. COCO class names array included (80 classes)

Block C — ROS 2 Vision 노드 (depth_camera_cpp 통합형):
  Write a complete C++ ROS 2 node source at /opt/rover/src/yolo_ros_node.cpp that:
  1. Includes yolo_trt.hpp
  2. Node class: YoloDetectorNode : public rclcpp::Node
  3. Subscribes to /camera/color/image_raw (sensor_msgs/msg/Image, BGR8)
     - Converts ROS Image msg to uint8_t* (no cv_bridge PyTorch backend)
     - Uses only libopencv-dev (already installed in base)
  4. Publishes /vision/detections (std_msgs/msg/String, JSON)
     JSON format: {
       "timestamp": 1234567890.123,
       "frame_id": "camera_color_optical_frame",
       "detections": [{"class": "cup", "conf": 0.92, "bbox": [x,y,w,h], "center": [cx,cy]}]
     }
  5. Publishes /vision/image_annotated (sensor_msgs/msg/Image) with bounding boxes drawn
     (OpenCV rectangle + text, no PyTorch)
  6. Parameter: ~conf_threshold (default 0.5), ~engine_path

  Also write the CMakeLists.txt for this node that:
  - find_package: rclcpp, sensor_msgs, std_msgs, OpenCV
  - target_link_libraries: libnvinfer, libcudart (TensorRT + CUDA)
  - Copies yolo_trt.hpp to include/

[Constraints]
- NO pip install, NO Python code in this stage (C++ only for TRT)
- NO ultralytics, NO torch imports anywhere
- The .engine file is NOT baked into the image — it is mounted from host SSD at runtime
  via: -v /home/ubuntu/AI_secretary_robot/models/vision:/opt/rover/models/vision:ro
- The C++ wrapper must use raw CUDA buffers, NOT torch::Tensor
- NvInfer.h must be found at build time (verify in Dockerfile RUN)

[Output format]
- Dockerfile content for stage `vision-runtime`
- Complete /opt/rover/include/yolo_trt.hpp
- Complete /opt/rover/src/yolo_ros_node.cpp
- Complete /opt/rover/src/CMakeLists_yolo.txt (for the yolo node)
```

---

## 🤖 Prompt 4: Dockerfile — ROS 2 Heavy 패키지 + workspace 빌드

```
[Context — Prompt 3의 `vision-runtime` 스테이지 위에 이어서 작성]
- Previous stage: `vision-runtime` (ONNX Runtime + llama.cpp + Piper + TRT C++ headers)
- ROS 2 Humble base (ros-humble-ros-base)는 Prompt 1에서 이미 설치되어 있습니다.
- 이 스테이지에서는 ROS 2의 무거운 패키지(MoveIt2, Nav2)와 workspace 빌드만 수행합니다.
- Output: adds stage `ros2-runtime` to `docker/Dockerfile`

[Existing workspace C++ packages to build — located at /home/ubuntu/AI_secretary_robot/src/]:
  - wake_vad_cpp      # Wake word + VAD (openwakeword, webrtcvad)
  - stt_cpp           # Moonshine ONNX STT node (libcurl for model API)
  - intent_router_cpp # KoSimCSE ONNX intent classifier
  - llm_cpp           # LLM node (calls llama-server via HTTP/curl)
  - tts_cpp           # TTS node (Piper via subprocess + ALSA playback)
  - ros_robot_controller_msgs  # Custom message types
  - ros_robot_controller_cpp   # Hardware controller (UART/USB)
  - jetrover_arm_moveit         # MoveIt 2 arm planning
  - depth_camera_cpp            # Orbbec Dabai DCW (USB 2.0 RGB-D)
  - imu_bridge_cpp              # MPU6050 IMU bridge
  - battery_cpp                 # Battery monitor
  - rover_autonomy_cpp          # Nav2 + SLAM integration
  - rover_common                # Shared utilities

[Task]
Write the FOURTH STAGE (`FROM vision-runtime AS ros2-runtime`) of the Dockerfile.

Block A — ROS 2 Heavy 패키지 추가 설치:
  ⚠️ ros-humble-ros-base는 이미 Prompt 1(base stage)에 설치됨. 중복 설치 금지.
  ⚠️ rosdep init도 이미 완료. rosdep update만 재실행.

  1. apt-get install (추가 패키지만):
     - ros-humble-moveit                    # MoveIt 2 (팔 제어)
     - ros-humble-nav2-bringup              # Nav2 전체 스택
     - ros-humble-slam-toolbox              # SLAM (지도 작성)
     - ros-humble-trac-ik-kinematics-plugin # IK 솔버 (역기구학)
     - ros-humble-ros2-control              # 하드웨어 인터페이스
     - ros-humble-ros2-controllers          # 모터 컨트롤러
     - ros-humble-joint-state-publisher     # URDF 관절 상태

  2. Orbbec Dabai DCW 드라이버 설치:
     방법 A (apt 패키지가 있는 경우 시도):
       apt-get install ros-humble-orbbec-camera || BUILD_FROM_SOURCE=true

     방법 B (apt 실패 시 소스 빌드 — fallback):
       git clone https://github.com/orbbec/OrbbecSDK_ROS2.git /tmp/orbbec_ros2
       cd /tmp/orbbec_ros2
       source /opt/ros/humble/setup.bash
       colcon build --base-paths . --cmake-args -DCMAKE_BUILD_TYPE=Release
       cp -r install/* /opt/ros/humble/
       rm -rf /tmp/orbbec_ros2

     ⚠️ 소스 빌드 시 /opt/rover/ws/src에 포함하지 말고 /opt/ros/humble/에 직접 설치.
        이렇게 해야 source /opt/ros/humble/setup.bash 한 번으로 자동 인식.

Block B — workspace 빌드:
  1. COPY the entire workspace src/ tree into /opt/rover/ws/src/
     (At build time: COPY --chown=root:root ./src /opt/rover/ws/src)

  2. Update rosdep (init은 이미 완료):
     rosdep update --rosdistro humble

  3. Install rosdep dependencies for workspace:
     source /opt/ros/humble/setup.bash
     cd /opt/rover/ws
     rosdep install --from-paths src --ignore-src -r -y \
       --skip-keys "orbbec_camera orbbec_camera_msgs"
     # orbbec는 Block A에서 별도 설치했으므로 skip

  4. Build with colcon:
     source /opt/ros/humble/setup.bash
     cd /opt/rover/ws
     colcon build \
       --base-paths src \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
       --parallel-workers $(nproc) \
       --event-handlers console_direct+

  5. Source the install:
     echo "source /opt/rover/ws/install/setup.bash" >> /etc/bash.bashrc

  6. Remove build artifacts to reduce layer size:
     rm -rf /opt/rover/ws/build /opt/rover/ws/log

Block C — 오디오 디바이스 설정:
  1. Create ALSA config at /etc/asound.conf:
     - Set default PCM to ReSpeaker 6-Mic Array (USB audio, vendor 2886:0018)
     - Set default CTL for volume control
     - Add a loopback device for TTS playback monitoring
  2. Add udev rule for ReSpeaker: /etc/udev/rules.d/99-respeaker.rules
     SUBSYSTEM=="sound", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", \
     SYMLINK+="respeaker", MODE="0666"
  3. Add udev rule for Orbbec Dabai DCW: /etc/udev/rules.d/99-orbbec.rules
     SUBSYSTEM=="usb", ATTRS{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
     (Orbbec USB vendor ID: 0x2bc5)

[Constraints]
- ros-humble-ros-base 재설치 금지 (이미 Prompt 1 base stage에 있음)
- colcon build: --cmake-args -DCMAKE_BUILD_TYPE=Release (debug symbols off)
- Build/log 디렉토리 삭제로 이미지 레이어 크기 최소화
- model 파일은 이미지에 포함하지 않음 (host SSD에서 bind-mount)
- 완성된 install/ 결과물은 이미지 레이어에 포함됨

[Output format]
- Dockerfile content for stage `ros2-runtime`
- /etc/asound.conf (ALSA config for ReSpeaker 6-mic + 3W speaker)
- /etc/udev/rules.d/99-respeaker.rules
- /etc/udev/rules.d/99-orbbec.rules
- Comment at top: expected final image size (target: < 4GB total)
```

---

## 🔀 Prompt 5: Python 오케스트레이터 — 3-Track 파이프라인 매니저

```
[Context]
이미 구축된 Docker 이미지 안에서 실행될 메인 파이프라인 오케스트레이터를 작성합니다.
이 스크립트는 모든 ROS 2 노드를 관리하고, 3-Track 분기 로직을 구현합니다.

[Task]
Write `/opt/rover/scripts/pipeline_manager.py` — the main orchestration process.

This script runs INSIDE the container and manages the full data pipeline:

=== ARCHITECTURE ===

Always-ON (상시 대기조, RAM < 300MB total):
  - wake_vad_cpp node   → detects wake word "자비스"
  - stt_cpp node        → Moonshine ONNX, transcribes after wake word
  - intent_router_cpp   → KoSimCSE ONNX, classifies intent (Track A/B/C)
  - tts_cpp node        → Piper TTS, always listening for /tts/text topic
  - yolo_ros_node       → YOLO11n TRT, always processing camera frames (Track A)

On-Demand (수요 대기조, SSD mmap → RAM):
  - llama-server (LLM)  → Qwen 1.5B GGUF, loaded only for Track B/C
  - llama-server (VLM)  → Moondream 1.8B GGUF, loaded only for Track B

=== TRACK ROUTING LOGIC ===

The intent_router_cpp publishes to /intent/classification with JSON:
  {"track": "A"|"B"|"C", "confidence": 0.95, "utterance": "...", "params": {...}}

Track A — Fast Path (simple navigation/manipulation):
  Triggers: navigate, pick, place, move, go, stop
  Flow: intent → YOLO detection → coordinate extraction → /arm/goal or /nav/goal
  Latency target: < 200ms total
  LLM/VLM: STAY ASLEEP

Track B — VLM Track (visual complex reasoning):
  Triggers: describe, find by color/shape, "which one", "what is"
  Flow: intent → [wake VLM] → capture frame → VLM inference → coordinate → arm/nav
  Latency target: < 3s total (VLM load: ~1s, inference: ~2s)
  VLM lifecycle:
    1. Send SIGCONT to llama-server (VLM) process → it mmap-loads from SSD
    2. POST to http://localhost:8081/v1/chat/completions with image+text
    3. Parse response for coordinates/actions
    4. After response: send SIGSTOP to llama-server (VLM) → returns SSD
    NOTE: Use SIGSTOP/SIGCONT for instant freeze/resume (no restart overhead)

Track C — LLM Track (conversation/questions):
  Triggers: question, explain, what, why, how, weather, schedule
  Flow: intent → [wake LLM] → text inference → TTS response
  Latency target: < 1.5s first token
  LLM lifecycle: same SIGSTOP/SIGCONT pattern as VLM

=== PROCESS MANAGEMENT ===

class LlamaServerManager:
  """Manages llama-server processes with SIGSTOP/SIGCONT for zero-restart lazy loading."""

  def __init__(self, model_path: str, port: int, ctx_size: int = 2048):
      self.model_path = model_path   # GGUF file on NVMe SSD (bind-mounted)
      self.port = port
      self.ctx_size = ctx_size
      self.process: subprocess.Popen = None
      self.state = "stopped"  # stopped | sleeping | running

  def start(self):
      """Start llama-server with mmap enabled (models stay on SSD until needed)."""
      cmd = [
          "llama-server",
          "--model", self.model_path,
          "--port", str(self.port),
          "--ctx-size", str(self.ctx_size),
          "--n-gpu-layers", "99",   # All layers to GPU
          "--mmap",                  # KEY: memory-mapped file (lazy load from SSD)
          "--no-mlock",              # Allow OS to page out mmap pages → RAM 절약
          "--threads", "4",
          "--batch-size", "512",
          "--host", "127.0.0.1",
      ]
      self.process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      # Wait for server ready: poll GET http://localhost:{port}/health
      self._wait_for_health()
      self.state = "sleeping"
      os.kill(self.process.pid, signal.SIGSTOP)  # Freeze immediately after ready

  def wake(self):
      """Resume the frozen process (mmap pages load from SSD on demand)."""
      if self.state == "sleeping":
          os.kill(self.process.pid, signal.SIGCONT)
          self.state = "running"
          self._wait_for_health()  # Re-check health after SIGCONT

  def sleep(self):
      """Freeze the process (OS can evict mmap pages to reclaim RAM)."""
      if self.state == "running":
          os.kill(self.process.pid, signal.SIGSTOP)
          self.state = "sleeping"

  def _wait_for_health(self, timeout: int = 30):
      """Poll /health endpoint until server is ready."""
      # HTTP GET http://127.0.0.1:{port}/health, retry every 0.5s, timeout after 30s

=== MEMORY MANAGEMENT ===

Memory budget (8GB total):
  Always-ON stack:    ~300MB RAM
  ROS 2 + Nav2:      ~1.5GB RAM
  SLAM + MoveIt:     ~1.0GB RAM
  LLM (when active): ~0.8GB RAM (Qwen 1.5B Q4_K_M, GPU layers)
  VLM (when active): ~0.9GB RAM (Moondream 1.8B GGUF, GPU layers)
  Free buffer:       ~3.5GB (no OOM)

Add a memory watchdog thread that:
  - Polls /proc/meminfo every 5 seconds
  - If MemAvailable < 512MB: force-sleep both LLM and VLM servers, log warning
  - Publishes /system/memory_status (std_msgs/msg/String, JSON)
  JSON: {"mem_total_mb": 8192, "mem_available_mb": 3500, "llm_state": "sleeping", ...}

=== ROS 2 INTEGRATION ===

The orchestrator must:
  1. source /opt/ros/humble/setup.bash and /opt/rover/ws/install/setup.bash before launch
  2. Launch all always-ON nodes via subprocess (ros2 run ...)
  3. Subscribe to /intent/classification via rclpy (in a dedicated thread)
  4. Route to Track A/B/C based on classification
  5. Publish action goals to MoveIt (/arm/goal) and Nav2 (/nav/goal)
  6. Error handling: if Track B VLM fails → fallback to Track A (YOLO only)
  7. On SIGTERM: send SIGTERM to all child processes, wait, then exit cleanly

=== DATABASE INTEGRATION ===

Use sqlite3 (stdlib, thread-safe mode) to log every interaction:
  - Log conversation to `conversations` table (see schema in README.md)
  - Update `objects` table when YOLO/VLM detects a new object
  - Update `object_locations` when arm successfully picks an object
  DB path from env: ROVER_DB_PATH (default: /opt/rover/db/rover.db)

[Output]
- /opt/rover/scripts/pipeline_manager.py (full implementation, ~400 lines)
- /opt/rover/scripts/launch_all.sh (bash script to start the full pipeline)
  This script must:
    1. source /opt/ros/humble/setup.bash
    2. source /opt/rover/ws/install/setup.bash
    3. export all required env vars (ROVER_MODEL_DIR, ROVER_DB_PATH, etc.)
    4. exec python3 /opt/rover/scripts/pipeline_manager.py
    5. Trap SIGTERM/SIGINT to trigger clean shutdown
```

---

## 🚀 Prompt 6: Docker Compose + 배포 스크립트 (최종 조립)

```
[Context]
모든 Dockerfile 스테이지(Prompt 1~4)와 오케스트레이터(Prompt 5)가 완성된 상태입니다.
이제 전체 시스템을 하나의 명령으로 실행하는 docker-compose.yml과 배포 스크립트를 작성합니다.

Host environment:
  - Host OS: Ubuntu 22.04, JetPack 6.x
  - Docker runtime: nvidia-container-runtime (already configured)
  - Models on host NVMe: /home/ubuntu/AI_secretary_robot/models/
  - rover_ws on host: /home/ubuntu/rover_ws/
  - Audio: ReSpeaker 6-Mic USB (vendor 2886:0018), 3W speaker via 3.5mm jack
  - Camera: Orbbec Dabai DCW (USB 2.0, vendor 2bc5)
  - Robot: /dev/ttyUSB0 (UART to robot controller)

[Task]

--- File 1: docker-compose.yml ---
Write a docker-compose.yml with ONE service: `rover`

service `rover`:
  image: jetrover/rover:latest
  container_name: jetrover_rover
  restart: unless-stopped

  runtime: nvidia
  environment:
    - NVIDIA_VISIBLE_DEVICES=all
    - NVIDIA_DRIVER_CAPABILITIES=compute,utility,video
    - ROS_DOMAIN_ID=42
    - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    - FASTRTPS_DEFAULT_PROFILES_FILE=/opt/rover/config/fastdds.xml
    - PULSE_SERVER=unix:/run/user/1000/pulse/native
    - ROVER_MODEL_DIR=/opt/rover/models
    - ROVER_DB_PATH=/opt/rover/db/rover.db
    - LLM_MODEL=qwen2.5-1.5b-instruct-q4_k_m.gguf
    - VLM_MODEL=moondream2-1.8b-f16.gguf
    - YOLO_ENGINE=/opt/rover/models/vision/yolo11n_fp16.engine

  volumes:
    # Models on NVMe SSD (read-only, mmap from here)
    - /home/ubuntu/AI_secretary_robot/models:/opt/rover/models:ro
    # Persistent DB (read-write, bind-mounted from host)
    - /home/ubuntu/AI_secretary_robot/db:/opt/rover/db:rw
    # ROS 2 workspace install (host build sync용, read-only)
    # NOTE: 컨테이너 내부에 이미 빌드된 /opt/rover/ws/install이 있으나,
    #       host에서 재빌드 후 동기화가 필요한 경우 아래 주석을 해제:
    # - /home/ubuntu/rover_ws/install:/opt/rover/ws/install:ro
    # Audio (PulseAudio socket)
    - /run/user/1000/pulse:/run/user/1000/pulse

  devices:
    - /dev/snd:/dev/snd           # ALSA raw audio
    - /dev/ttyUSB0:/dev/ttyUSB0  # Robot UART controller

  device_cgroup_rules:
    - 'c 189:* rmw'   # USB devices (ReSpeaker mic, Orbbec camera)
    - 'c 81:* rmw'    # Video devices (V4L2)

  network_mode: host   # ROS 2 DDS requires host networking (ROS_DOMAIN_ID 공유)

  ipc: host            # Shared memory for ROS 2 zero-copy transport

  ulimits:
    memlock: -1
    stack: 67108864

  privileged: false
  cap_add:
    - SYS_NICE          # For real-time scheduling (ROS 2 executor)
    - NET_ADMIN         # For ROS 2 DDS multicast

  command: /opt/rover/scripts/launch_all.sh

  healthcheck:
    test: ["CMD", "bash", "-c",
           "source /opt/ros/humble/setup.bash && ros2 node list | grep -q stt_node"]
    interval: 30s
    timeout: 10s
    retries: 3
    start_period: 90s   # MoveIt 로딩 시간 고려


--- File 2: scripts/build_image.sh ---
Write a bash script to build the final Docker image:
  1. cd to project root (where docker/Dockerfile lives)
  2. Build:
     docker build --platform linux/arm64 \
       --build-arg JETPACK_VERSION=36.4.3 \
       -t jetrover/rover:latest \
       -f docker/Dockerfile \
       --progress=plain \
       .
  3. Print image size: docker image inspect jetrover/rover:latest --format='{{.Size}}'
  4. Smoke test:
     docker run --rm --runtime nvidia jetrover/rover:latest \
       bash -c "source /opt/ros/humble/setup.bash && \
                ros2 --version && \
                llama-cli --version && \
                python3 -c 'import onnxruntime as ort; assert ort.get_device()==\"GPU\"'"
  5. If --export flag given: docker save jetrover/rover:latest -o /tmp/rover_image.tar


--- File 3: scripts/first_run_setup.sh ---
Write a bash script for first-time setup on the Jetson HOST (not inside container):
  1. Check prerequisites: docker, nvidia-container-runtime, nvcc

  2. Create host directories:
       /home/ubuntu/AI_secretary_robot/models/llm/
       /home/ubuntu/AI_secretary_robot/models/vlm/
       /home/ubuntu/AI_secretary_robot/models/vision/
       /home/ubuntu/AI_secretary_robot/db/

  3. Download YOLO11n weights (.pt file) on HOST (not in container):
       pip3 install ultralytics --quiet
       python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
       find ~/.cache/ultralytics -name "yolo11n.pt" -exec cp {} \
         /home/ubuntu/AI_secretary_robot/models/vision/ \;

  4. ⚠️ YOLO11n → TensorRT Engine 변환 (일회성 작업):
     일회용 l4t-pytorch 컨테이너를 띄워 변환 수행 (rover 이미지에는 PyTorch 없음):

     docker run --rm --runtime nvidia \
       -v /home/ubuntu/AI_secretary_robot/models:/models \
       nvcr.io/nvidia/l4t-pytorch:r36.4.3-pth2.4-py3 \
     # ↑ JetPack 6.0+ 최신 공식 태그: r36.4.3-pth2.4-py3 (PyTorch 2.4, Python 3.10)
     #   fallback: r36.3.0-pth2.3-py3 (태그 확인: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-pytorch)
       python3 -c "
         from ultralytics import YOLO
         import os
         model = YOLO('/models/vision/yolo11n.pt')
         model.export(
           format='engine',
           device=0,
           half=True,
           workspace=2,
           simplify=True,
           imgsz=640
         )
         import shutil
         shutil.move('/models/vision/yolo11n.engine',
                     '/models/vision/yolo11n_fp16.engine')
         print('Engine saved to /models/vision/yolo11n_fp16.engine')
         print('Size:', os.path.getsize('/models/vision/yolo11n_fp16.engine'), 'bytes')
       "

     ⚠️ 이 단계는 최초 1회만 실행. 완료 후 l4t-pytorch 컨테이너는 자동 삭제됨(--rm).

  5. Initialize SQLite DB schema:
     docker run --rm \
       -v /home/ubuntu/AI_secretary_robot/db:/opt/rover/db \
       jetrover/rover:latest \
       python3 /opt/rover/scripts/init_db.py

  6. Print summary and next steps:
     "✅ Setup complete!"
     "Engine: /home/ubuntu/AI_secretary_robot/models/vision/yolo11n_fp16.engine"
     "DB: /home/ubuntu/AI_secretary_robot/db/rover.db"
     "Run: docker-compose up -d"


--- File 4: scripts/init_db.py ---
Write a Python script to initialize the SQLite database schema:
  1. DB path from env ROVER_DB_PATH (default: /opt/rover/db/rover.db)
  2. Use the full schema defined in README.md:
     conversations, objects, locations, object_locations,
     system_logs, users, task_queue, embedding_cache
  3. Create SQLite FTS5 virtual table for full-text search:
     CREATE VIRTUAL TABLE conversations_fts USING fts5(user_utterance, tts_response, ...)
  4. Create indexes: idx_timestamp, idx_session, idx_intent
  5. Insert default data:
     - 5 common Korean household objects in `objects` table
     - 1 default user profile in `users` table
  6. Print: "✅ Database initialized: <path> — <N> tables created"


[Output]
- docker-compose.yml
- docker/Dockerfile final stage: `FROM ros2-runtime AS final`
  COPY scripts/ /opt/rover/scripts/
  RUN chmod +x /opt/rover/scripts/*.sh /opt/rover/scripts/*.py
  EXPOSE 8080   # llama-server LLM (Qwen)
  EXPOSE 8081   # llama-server VLM (Moondream)
  EXPOSE 8765   # AI WebSocket API (optional monitoring)
  WORKDIR /opt/rover
  ENTRYPOINT ["/opt/rover/scripts/launch_all.sh"]
- scripts/build_image.sh
- scripts/first_run_setup.sh  (⚠️ YOLO TRT 변환은 l4t-pytorch 일회용 컨테이너 사용)
- scripts/init_db.py
```

---

## 📋 실행 순서 요약

| 단계 | 프롬프트 | 산출물 | 예상 용량 |
|:---:|:---|:---|---:|
| 1 | Base + ROS 2 | `Dockerfile` (base stage) | ~2.0GB |
| 2 | AI Runtime | `Dockerfile` (ai-runtime stage) | +800MB |
| 3 | Vision (TRT C++) | `Dockerfile` (vision-runtime stage) | +200MB |
| 4 | MoveIt/Nav2 + 빌드 | `Dockerfile` (ros2-runtime stage) | +1.0GB |
| 5 | Orchestrator | `pipeline_manager.py`, `launch_all.sh` | - |
| 6 | Deploy | `docker-compose.yml`, 빌드/배포 스크립트 | - |
| **합계** | | **최종 이미지** | **~4GB** |

> 기존 l4t-pytorch 이미지(15GB) 대비 **약 73% 절감**
> YOLO11n TRT 변환은 최초 1회만 일회용 l4t-pytorch 컨테이너로 수행

---

## ⚠️ 각 프롬프트 사용 팁

1. **Prompt 1을 먼저 실행**하고 base stage가 ARM64에서 정상 빌드되는지 확인 후 진행

2. **각 프롬프트 실행 전**: 이전 프롬프트의 산출물(파일)을 Context에 붙여넣기 — AI가 일관성 유지

3. **Prompt 2 실행 시**: ONNX Runtime은 CMake 3.26+ 선반영 후 소스 빌드
   - libnvinfer8 등 TRT 패키지 설치 → Kitware APT로 CMake 업그레이드 → `packages/ml/onnxruntime/build.sh` 실행
   - 참고: https://github.com/dusty-nv/jetson-containers (packages/ml/onnxruntime)
   - 실패 로그에 `CMake 3.26 or higher is required`가 나오면 CMake 업그레이드 누락

4. **Prompt 3 실행 시**: CUDA_ARCHITECTURES="87" 고정 (Jetson Orin Ampere)
   - ultralytics, torch 설치 시도하면 즉시 거부할 것

5. **Prompt 6 실행 후**: first_run_setup.sh의 YOLO 변환 단계에서
   nvcr.io/nvidia/l4t-pytorch 이미지가 15GB이므로 충분한 저장 공간 확보 필요.
   변환 완료 후 `docker rmi nvcr.io/nvidia/l4t-pytorch:...` 로 삭제하면 공간 회수.

6. **Prompt 5 실행 시**: SIGSTOP/SIGCONT 기반 lazy loading은 llama.cpp의 --mmap 플래그와
   조합해야 효과 극대화. --mlock 없이 실행해야 OS가 mmap 페이지를 evict할 수 있음.

---

## 🦾 보너스: jetrover_arm_moveit — TRAC-IK 역기구학 솔버 설정

MoveIt2 arm planning에 사용되는 역기구학(IK) 솔버를 기본 KDL에서 TRAC-IK로 전환하여 성능을 향상시킵니다.

**상세 프롬프트 문서**: [`docs/codex_prompt_trac_ik.md`](./codex_prompt_trac_ik.md)

**Quick Setup**:
```bash
# 1. TRAC-IK 플러그인 설치 (이미 package.xml에 의존성 포함됨)
sudo apt install -y ros-humble-trac-ik-kinematics-plugin

# 2. 이미 적용된 설정 확인
cat src/control/jetrover_arm_moveit/config/kinematics.yaml

# 3. 빌드 및 테스트
cd /home/ubuntu/AI_secretary_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select jetrover_arm_moveit
source install/setup.bash
ros2 launch jetrover_arm_moveit moveit_demo.launch.py
```

**주요 변경사항**:
- `kinematics_solver`: `kdl_kinematics_plugin/KDLKinematicsPlugin` → `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`
- `solve_type`: `Speed` (실시간 제어 최적화)
- `kinematics_solver_timeout`: 0.05초 → 0.01초 (10ms, Jetson Orin 최적화)
- `position_tolerance`: 0.001m (1mm 정밀도)
- `orientation_tolerance`: 0.05 rad (~3° 정밀도)

**예상 성능 향상**:
- IK 계산 시간: 15~30ms (KDL) → 3~5ms (TRAC-IK Speed)
- 성공률 (일반): 60~70% → 85~95%
- 성공률 (특이점 근처): 10~20% → 60~80%

**상세 파라미터 튜닝 및 Troubleshooting**: [`codex_prompt_trac_ik.md`](./codex_prompt_trac_ik.md) 참조
