# ----------------------------------------------------------------------------
# Jetson Orin Nano (ARM64, JetPack 6.x) base stage
# Estimated size impact over l4t-base: ~450-600MB (apt + ROS 2 ros-base + tools)
# Notes:
# - Uses ROS 2 Humble ros-base only (no desktop/rviz/rqt) for size savings
# - Zero PyTorch / Zero Conda by design
# ----------------------------------------------------------------------------

ARG L4T_TAG=r36.4.3
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:${L4T_TAG}
FROM ${BASE_IMAGE} AS base

ARG JETPACK_VERSION=36.4.3

LABEL maintainer="AI Secretary Robot Team" \
      version="1.0.0" \
      jetpack.version="${JETPACK_VERSION}" \
      description="Minimal JetPack 6 ARM64 base with ROS 2 Humble ros-base and FastDDS defaults"

# Core environment (CUDA, locale, ROS 2, DDS)
ENV DEBIAN_FRONTEND=noninteractive \
    CUDA_HOME=/usr/local/cuda \
    PATH=${PATH}:${CUDA_HOME}/bin \
    LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CUDA_HOME}/lib64 \
    LANG=ko_KR.UTF-8 \
    ROS_DISTRO=humble \
    AMENT_PREFIX_PATH=/opt/ros/humble \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=42 \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/rover/config/fastdds.xml

# System deps + ROS 2 (install only when missing) + locale + rover dirs
RUN set -eux; \
    rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      gpg \
      locales \
      cmake \
      ninja-build \
      build-essential \
      git \
      wget \
      pkg-config \
      libasound2-dev \
      libpulse-dev \
      alsa-utils \
      portaudio19-dev \
      libcurl4-openssl-dev \
      libssl-dev \
      python3-pip \
      python3-dev \
      sqlite3 \
      libsqlite3-dev \
      ffmpeg \
      libavcodec-dev \
      libavformat-dev \
      libswresample-dev \
      libusb-1.0-0-dev \
      udev \
      python3-colcon-common-extensions \
      python3-rosdep && \
    if ! dpkg -s opencv-dev >/dev/null 2>&1 && ! dpkg -s libopencv-dev >/dev/null 2>&1; then \
      apt-get install -y --no-install-recommends libopencv-dev; \
    fi && \
    locale-gen ko_KR.UTF-8 && \
    update-locale LANG=ko_KR.UTF-8 && \
    ROS_SETUP=/opt/ros/humble/setup.bash; \
    if [ -f /opt/ros/humble/install/setup.bash ]; then ROS_SETUP=/opt/ros/humble/install/setup.bash; fi; \
    if [ ! -f "${ROS_SETUP}" ]; then \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg; \
      echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list; \
      apt-get update; \
      apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-ament-cmake \
        ros-humble-ament-index-cpp \
        ros-humble-tf2-ros \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-action-msgs \
        ros-humble-lifecycle-msgs; \
    fi && \
    rosdep init || true && \
    rosdep update --rosdistro humble && \
    ROS_SETUP=/opt/ros/humble/setup.bash; \
    if [ -f /opt/ros/humble/install/setup.bash ]; then ROS_SETUP=/opt/ros/humble/install/setup.bash; fi; \
    echo "source ${ROS_SETUP}" >> /etc/bash.bashrc && \
    mkdir -p \
      /opt/rover/models/stt \
      /opt/rover/models/embedding \
      /opt/rover/models/llm \
      /opt/rover/models/tts \
      /opt/rover/models/vision \
      /opt/rover/db \
      /opt/rover/ws \
      /opt/rover/scripts \
      /opt/rover/config && \
    rm -rf /var/lib/apt/lists/*

# FastDDS profile for host<->container ROS 2 communication (FastDDS/RMW)
RUN cat > /opt/rover/config/fastdds.xml <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <participant profile_name="default_profile" is_default_profile="true">
      <rtps>
        <name>rover_container</name>
        <useBuiltinTransports>true</useBuiltinTransports>
        <builtin>
          <discovery_config>
            <discoveryProtocol>SIMPLE</discoveryProtocol>
          </discovery_config>
        </builtin>
      </rtps>
    </participant>

    <!-- General topics: KEEP_LAST depth 10 -->
    <publisher profile_name="default_publisher_profile" is_default_profile="true">
      <qos>
        <history>
          <kind>KEEP_LAST</kind>
          <depth>10</depth>
        </history>
      </qos>
    </publisher>

    <!-- Sensor topics: KEEP_LAST depth 1 -->
    <publisher profile_name="sensor_publisher_profile">
      <qos>
        <history>
          <kind>KEEP_LAST</kind>
          <depth>1</depth>
        </history>
      </qos>
    </publisher>
  </profiles>
</dds>
XML

# Verification: ROS 2 and rclpy availability
RUN bash -c "if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; \
             elif [ -f /opt/ros/humble/install/setup.bash ]; then source /opt/ros/humble/install/setup.bash; \
             else echo 'ROS setup.bash not found' && exit 1; fi && \
             (ros2 --version || ros2 -h >/dev/null) && \
             python3 -c 'import rclpy; print(\"rclpy OK\")'"

FROM base AS ai-runtime

ARG PIPER_TAR_URL="https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_linux_aarch64.tar.gz"
ARG PIPER_SHA256="fea0fd2d87c54dbc7078d0f878289f404bd4d6eea6e7444a77835d1537ab88eb"
ARG ORT_WHL_URL_PRIMARY="https://pypi.jetson-ai-lab.io/jp6/cu126/%2Bf/4eb/e6a8902dc7708/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl"

ENV ORT_TENSORRT_ENGINE_CACHE_ENABLE=1 \
    ORT_TENSORRT_ENGINE_CACHE_PATH=/opt/rover/models/.trt_cache

# Block A: ONNX Runtime GPU (JetPack 6 / aarch64)
RUN set -eux; \
    mkdir -p /opt/rover/models/.trt_cache /tmp/ort && \
    apt-get update && apt-get install -y --no-install-recommends \
      libnvinfer8 libnvinfer-dev libnvinfer-plugin8 libnvinfer-plugin-dev \
      libnvparsers8 libnvparsers-dev libnvonnxparsers8 libnvonnxparsers-dev && \
    if ! wget --user-agent='pip/24.0' --progress=dot:giga -O /tmp/ort/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl "${ORT_WHL_URL_PRIMARY}"; then \
      python3 -m pip download --no-deps \
        --index-url https://pypi.jetson-ai-lab.io/jp6/cu126/+simple \
        onnxruntime-gpu==1.23.0 \
        -d /tmp/ort; \
    fi && \
    pip3 install --no-cache-dir --no-deps /tmp/ort/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl && \
    pip3 install --no-cache-dir --index-url https://pypi.org/simple \
      numpy==1.24.4 \
      onnx==1.15.0 \
      protobuf==3.20.3 \
      coloredlogs \
      flatbuffers && \
    python3 -c "import onnxruntime as ort; d=ort.get_device(); print(d); assert d=='GPU'" && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/ort

# Block B: llama.cpp (CUDA, SM87, HTTP enabled)
RUN set -eux; \
    git clone https://github.com/ggerganov/llama.cpp /tmp/llama.cpp && \
    cd /tmp/llama.cpp && \
    LATEST_TAG="$(git tag --sort=-version:refname | grep '^b[0-9]' | head -1)" && \
    test -n "${LATEST_TAG}" && \
    git checkout "${LATEST_TAG}" && \
    cmake -B build \
      -DGGML_CUDA=ON \
      -DCMAKE_CUDA_ARCHITECTURES="87" \
      -DLLAMA_CURL=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DLLAMA_NATIVE=OFF \
      -DGGML_CUDA_FORCE_MMQ=ON \
      -DGGML_CUDA_ENABLE_UNIFIED_MEMORY=ON && \
    cmake --build build --config Release -j"$(nproc)" && \
    install -m 0755 build/bin/llama-server /usr/local/bin/llama-server && \
    install -m 0755 build/bin/llama-cli /usr/local/bin/llama-cli && \
    install -m 0755 build/bin/llama-embedding /usr/local/bin/llama-embedding && \
    install -d /usr/local/lib && \
    find build -type f \( -name 'libllama.so*' -o -name 'libggml.so*' -o -name 'libggml-cuda.so*' \) -exec cp -av {} /usr/local/lib/ \; && \
    install -d /usr/local/include/llama && \
    cp -a include/. /usr/local/include/llama/ && \
    printf '%s\n' /usr/lib/aarch64-linux-gnu/nvidia > /etc/ld.so.conf.d/99-nvidia-tegra.conf && \
    ldconfig && \
    LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/cuda/lib64:${LD_LIBRARY_PATH} llama-cli --version && \
    rm -rf /tmp/llama.cpp

# Block C: Piper TTS + Korean voice
RUN set -eux; \
    test -n "${PIPER_SHA256}" && \
    mkdir -p /opt/piper /opt/rover/models/tts /tmp/piper && \
    wget --progress=dot:giga -O /tmp/piper/piper_linux_aarch64.tar.gz "${PIPER_TAR_URL}" && \
    echo "${PIPER_SHA256}  /tmp/piper/piper_linux_aarch64.tar.gz" | sha256sum -c - && \
    tar -xzf /tmp/piper/piper_linux_aarch64.tar.gz -C /opt/piper && \
    PIPER_BIN="$(find /opt/piper -type f -name piper | head -n1)" && \
    test -n "${PIPER_BIN}" && \
    ln -sf "${PIPER_BIN}" /usr/local/bin/piper && \
    wget --progress=dot:giga -O /opt/rover/models/tts/ko_KR-kss-medium.onnx \
      https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/ko/ko_KR/kss/medium/ko_KR-kss-medium.onnx && \
    wget --progress=dot:giga -O /opt/rover/models/tts/ko_KR-kss-medium.onnx.json \
      https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/ko/ko_KR/kss/medium/ko_KR-kss-medium.onnx.json && \
    echo "안녕하세요" | piper --model /opt/rover/models/tts/ko_KR-kss-medium.onnx --output_file /tmp/test.wav && \
    rm -f /tmp/test.wav && \
    rm -rf /tmp/piper

HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
  CMD llama-cli --version && python3 -c "import onnxruntime as ort; assert ort.get_device()=='GPU'"
