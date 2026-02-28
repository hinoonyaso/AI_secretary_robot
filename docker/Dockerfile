# ----------------------------------------------------------------------------
# Jetson Orin Nano (ARM64, JetPack 6.x) base stage
# Estimated size impact over l4t-base: ~450-600MB (apt + ROS 2 ros-base + tools)
# Notes:
# - Uses ROS 2 Humble ros-base only (no desktop/rviz/rqt) for size savings
# - Zero PyTorch / Zero Conda by design
# ----------------------------------------------------------------------------

ARG L4T_TAG=r36.2.0
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:${L4T_TAG}
FROM ${BASE_IMAGE} AS base

ARG JETPACK_VERSION=36.2.0

LABEL maintainer="AI Secretary Robot Team" \
  version="1.0.0" \
  jetpack.version="${JETPACK_VERSION}" \
  description="Minimal JetPack 6 ARM64 base with ROS 2 Humble ros-base and FastDDS defaults"

# Core environment (CUDA, locale, ROS 2, DDS)
ENV DEBIAN_FRONTEND=noninteractive \
  CUDA_HOME=/usr/local/cuda \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/cuda/bin \
  LD_LIBRARY_PATH=/usr/local/cuda/lib64 \
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
  udev && \
  if ! dpkg -s opencv-dev >/dev/null 2>&1 && ! dpkg -s libopencv-dev >/dev/null 2>&1; then \
  apt-get install -y --no-install-recommends libopencv-dev; \
  fi && \
  locale-gen ko_KR.UTF-8 && \
  update-locale LANG=ko_KR.UTF-8 && \
  ROS_SETUP=""; \
  if [ -f /opt/ros/humble/setup.bash ]; then ROS_SETUP=/opt/ros/humble/setup.bash; \
  elif [ -f /opt/ros/humble/install/setup.bash ]; then ROS_SETUP=/opt/ros/humble/install/setup.bash; fi; \
  if [ -z "${ROS_SETUP}" ]; then \
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
  apt-get install -y --no-install-recommends python3-colcon-common-extensions; \
  if apt-cache show python3-rosdep >/dev/null 2>&1; then \
  apt-get install -y --no-install-recommends python3-rosdep; \
  elif apt-cache show python3-rosdep2 >/dev/null 2>&1; then \
  apt-get install -y --no-install-recommends python3-rosdep2; \
  else \
  echo "WARN: rosdep apt package not found; skipping rosdep install"; \
  fi; \
  else \
  if ! command -v colcon >/dev/null 2>&1; then \
  apt-get update; \
  apt-get install -y --no-install-recommends python3-colcon-common-extensions; \
  fi; \
  if ! command -v rosdep >/dev/null 2>&1; then \
  apt-get update; \
  if apt-cache show python3-rosdep >/dev/null 2>&1; then \
  apt-get install -y --no-install-recommends python3-rosdep; \
  elif apt-cache show python3-rosdep2 >/dev/null 2>&1; then \
  apt-get install -y --no-install-recommends python3-rosdep2; \
  else \
  echo "WARN: rosdep apt package not found; skipping rosdep install"; \
  fi; \
  fi; \
  fi && \
  if command -v rosdep >/dev/null 2>&1; then \
  rosdep init || true; \
  rosdep update --rosdistro humble; \
  else \
  echo "WARN: rosdep command unavailable; skipping rosdep init/update"; \
  fi && \
  ROS_SETUP=""; \
  if [ -f /opt/ros/humble/setup.bash ]; then ROS_SETUP=/opt/ros/humble/setup.bash; \
  elif [ -f /opt/ros/humble/install/setup.bash ]; then ROS_SETUP=/opt/ros/humble/install/setup.bash; fi; \
  if [ -z "${ROS_SETUP}" ]; then echo "ROS setup.bash not found" && exit 1; fi && \
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
# Model URLs removed — models are volume-mounted from host at runtime

ENV ORT_TENSORRT_ENGINE_CACHE_ENABLE=1 \
  ORT_TENSORRT_ENGINE_CACHE_PATH=/opt/rover/models/.trt_cache \
  STT_MODEL_DIR=/opt/rover/models/stt/moonshine-tiny-ko \
  EMBEDDING_MODEL_DIR=/opt/rover/models/embedding/kosimcse-roberta-onnx \
  LLM_MODEL_PATH=/opt/rover/models/llm/qwen2.5-3b-instruct-q4_k_m.gguf

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
  apt-get update && apt-get install -y --no-install-recommends cuda-nvcc-12-2 cuda-driver-dev-12-2 && \
  mkdir -p /usr/local/cuda/lib64/stubs && \
  [ -e /usr/local/cuda/lib64/stubs/libcuda.so.1 ] || ln -sf /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1 && \
  test -x /usr/local/cuda/bin/nvcc && /usr/local/cuda/bin/nvcc --version && \
  git clone https://github.com/ggerganov/llama.cpp /tmp/llama.cpp && \
  cd /tmp/llama.cpp && \
  LLAMA_TAG="b6500" && \
  git checkout "${LLAMA_TAG}" && \
  cmake -B build \
  -DGGML_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES="87" \
  -DLLAMA_CURL=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DLLAMA_NATIVE=OFF \
  -DGGML_CUDA_FORCE_MMQ=ON \
  -DGGML_CUDA_ENABLE_UNIFIED_MEMORY=ON \
  -DCUDAToolkit_ROOT=/usr/local/cuda \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc && \
  JOBS="$(nproc)"; [ "${JOBS}" -gt 2 ] && JOBS=2; \
  LIBRARY_PATH=/usr/local/cuda/lib64/stubs:${LIBRARY_PATH:-} \
  LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-} \
  cmake --build build --config Release -j"${JOBS}" && \
  install -m 0755 build/bin/llama-server /usr/local/bin/llama-server && \
  install -m 0755 build/bin/llama-cli /usr/local/bin/llama-cli && \
  install -m 0755 build/bin/llama-embedding /usr/local/bin/llama-embedding && \
  install -d /usr/local/lib && \
  find build -type f \( -name 'libllama.so*' -o -name 'libggml.so*' -o -name 'libggml-cuda.so*' \) -exec cp -av {} /usr/local/lib/ \; && \
  install -d /usr/local/include/llama && \
  cp -a include/. /usr/local/include/llama/ && \
  printf '%s\n' /usr/lib/aarch64-linux-gnu/nvidia > /etc/ld.so.conf.d/99-nvidia-tegra.conf && \
  ldconfig && \
  if [ -s /usr/lib/aarch64-linux-gnu/nvidia/libcuda.so.1 ]; then \
  LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-} llama-cli --version; \
  else \
  echo "Skipping llama-cli runtime check during docker build (NVIDIA driver libs not mounted)"; \
  fi && \
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
  piper --version && \
  rm -rf /tmp/piper
# TTS model (ko_KR-kss-medium.onnx) is volume-mounted from host at runtime

# Block D: Model directories (models are volume-mounted from host at runtime)
RUN mkdir -p "${STT_MODEL_DIR}" "${EMBEDDING_MODEL_DIR}" /opt/rover/models/llm

HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
  CMD llama-cli --version && python3 -c "import onnxruntime as ort; assert ort.get_device()=='GPU'"

FROM ai-runtime AS vision-runtime

# YOLO model is volume-mounted from host at runtime

ENV YOLO_ENGINE=/opt/rover/models/vision/yolo11n_fp16.engine

# TensorRT C++ runtime/devel headers only (no PyTorch/ultralytics)
RUN set -eux; \
  apt-get update && apt-get install -y --no-install-recommends \
  libnvinfer8 \
  libnvinfer-dev \
  libnvinfer-bin \
  libnvinfer-plugin8 \
  libnvinfer-plugin-dev \
  libnvonnxparsers8 \
  libnvonnxparsers-dev \
  cuda-cupti-dev-12-2 && \
  test -f /usr/include/aarch64-linux-gnu/NvInfer.h && echo "TRT headers OK" && \
  test -f /usr/lib/aarch64-linux-gnu/libnvinfer.so.8 && echo "TensorRT 8 runtime OK" && \
  mkdir -p /opt/rover/include /opt/rover/src && \
  rm -rf /var/lib/apt/lists/*

# YOLO model directory (ONNX model is volume-mounted from host, TensorRT engine built at runtime)
RUN mkdir -p /opt/rover/models/vision

RUN cat > /usr/local/bin/build_yolo_engine.sh <<'EOF' && chmod +x /usr/local/bin/build_yolo_engine.sh
#!/usr/bin/env bash
set -euo pipefail
ONNX_PATH="${1:-/opt/rover/models/vision/yolo11n.onnx}"
ENGINE_PATH="${2:-/opt/rover/models/vision/yolo11n_fp16.engine}"
TRTEXEC_BIN="$(command -v trtexec || true)"
if [ -z "${TRTEXEC_BIN}" ] && [ -x /usr/src/tensorrt/bin/trtexec ]; then
  TRTEXEC_BIN=/usr/src/tensorrt/bin/trtexec
fi
[ -n "${TRTEXEC_BIN}" ] || { echo "trtexec not found"; exit 1; }
test -f "${ONNX_PATH}" || { echo "ONNX not found: ${ONNX_PATH}"; exit 1; }
"${TRTEXEC_BIN}" --onnx="${ONNX_PATH}" --saveEngine="${ENGINE_PATH}" --fp16
test -s "${ENGINE_PATH}"
echo "Engine generated: ${ENGINE_PATH}"
EOF

RUN cat > /opt/rover/include/yolo_trt.hpp <<'EOF'
#pragma once

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rover_vision {

struct Detection {
  float x;
  float y;
  float w;
  float h;
  float conf;
  int class_id;
  std::string class_name;
};

class YoloTRT {
 public:
  explicit YoloTRT(const std::string& engine_path = default_engine_path())
      : runtime_(nullptr),
        engine_(nullptr),
        context_(nullptr),
        stream_(nullptr),
        input_index_(-1),
        output_index_(-1),
        input_w_(640),
        input_h_(640),
        input_c_(3),
        output_count_(0),
        output_box_count_(0),
        output_attr_count_(0),
        device_input_(nullptr),
        device_output_(nullptr),
        scale_(1.0f),
        pad_x_(0.0f),
        pad_y_(0.0f) {
    init(engine_path.empty() ? default_engine_path() : engine_path);
  }

  ~YoloTRT() { release(); }

  YoloTRT(const YoloTRT&) = delete;
  YoloTRT& operator=(const YoloTRT&) = delete;

  std::vector<Detection> detect(const uint8_t* rgb_data, int width, int height,
                                float conf_thresh = 0.5f, float nms_thresh = 0.45f) {
    if (!rgb_data) {
      throw std::invalid_argument("rgb_data is null");
    }
    if (width <= 0 || height <= 0) {
      throw std::invalid_argument("invalid image size");
    }

    preprocess(rgb_data, width, height);

    cuda_check(cudaMemcpyAsync(device_input_, host_input_.data(),
                               host_input_.size() * sizeof(float),
                               cudaMemcpyHostToDevice, stream_),
               "cudaMemcpyAsync H2D failed");

    void* bindings[2];
    bindings[input_index_] = device_input_;
    bindings[output_index_] = device_output_;

    if (!context_->enqueueV2(bindings, stream_, nullptr)) {
      throw std::runtime_error("TensorRT enqueueV2 failed");
    }

    cuda_check(cudaMemcpyAsync(host_output_.data(), device_output_,
                               host_output_.size() * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_),
               "cudaMemcpyAsync D2H failed");
    cuda_check(cudaStreamSynchronize(stream_), "cudaStreamSynchronize failed");

    return postprocess(width, height, conf_thresh, nms_thresh);
  }

  static std::string default_engine_path() {
    const char* env = std::getenv("YOLO_ENGINE");
    if (env && std::strlen(env) > 0) {
      return std::string(env);
    }
    return "/opt/rover/models/vision/yolo11n_fp16.engine";
  }

  static const std::array<const char*, 80>& coco_names() {
    static const std::array<const char*, 80> names = {
        "person",        "bicycle",      "car",           "motorcycle",
        "airplane",      "bus",          "train",         "truck",
        "boat",          "traffic light","fire hydrant",  "stop sign",
        "parking meter", "bench",        "bird",          "cat",
        "dog",           "horse",        "sheep",         "cow",
        "elephant",      "bear",         "zebra",         "giraffe",
        "backpack",      "umbrella",     "handbag",       "tie",
        "suitcase",      "frisbee",      "skis",          "snowboard",
        "sports ball",   "kite",         "baseball bat",  "baseball glove",
        "skateboard",    "surfboard",    "tennis racket", "bottle",
        "wine glass",    "cup",          "fork",          "knife",
        "spoon",         "bowl",         "banana",        "apple",
        "sandwich",      "orange",       "broccoli",      "carrot",
        "hot dog",       "pizza",        "donut",         "cake",
        "chair",         "couch",        "potted plant",  "bed",
        "dining table",  "toilet",       "tv",            "laptop",
        "mouse",         "remote",       "keyboard",      "cell phone",
        "microwave",     "oven",         "toaster",       "sink",
        "refrigerator",  "book",         "clock",         "vase",
        "scissors",      "teddy bear",   "hair drier",    "toothbrush"};
    return names;
  }

 private:
  class Logger : public nvinfer1::ILogger {
   public:
    void log(Severity severity, const char* msg) noexcept override {
      if (severity <= Severity::kWARNING) {
        std::cerr << "[TensorRT] " << msg << std::endl;
      }
    }
  };

  struct InferDeleter {
    template <typename T>
    void operator()(T* obj) const {
      if (obj) {
        obj->destroy();
      }
    }
  };

  using RuntimePtr = std::unique_ptr<nvinfer1::IRuntime, InferDeleter>;
  using EnginePtr = std::unique_ptr<nvinfer1::ICudaEngine, InferDeleter>;
  using ContextPtr = std::unique_ptr<nvinfer1::IExecutionContext, InferDeleter>;

  static int64_t volume(const nvinfer1::Dims& dims) {
    int64_t v = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
      v *= (dims.d[i] < 0) ? 1 : dims.d[i];
    }
    return v;
  }

  static float iou(const Detection& a, const Detection& b) {
    const float x1 = std::max(a.x, b.x);
    const float y1 = std::max(a.y, b.y);
    const float x2 = std::min(a.x + a.w, b.x + b.w);
    const float y2 = std::min(a.y + a.h, b.y + b.h);
    const float inter_w = std::max(0.0f, x2 - x1);
    const float inter_h = std::max(0.0f, y2 - y1);
    const float inter = inter_w * inter_h;
    const float union_area = a.w * a.h + b.w * b.h - inter;
    return (union_area > 0.0f) ? (inter / union_area) : 0.0f;
  }

  static void cuda_check(cudaError_t code, const char* msg) {
    if (code != cudaSuccess) {
      throw std::runtime_error(std::string(msg) + ": " + cudaGetErrorString(code));
    }
  }

  void init(const std::string& engine_path) {
    std::ifstream f(engine_path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("failed to open engine: " + engine_path);
    }
    f.seekg(0, std::ios::end);
    const std::streamsize size = f.tellg();
    f.seekg(0, std::ios::beg);
    if (size <= 0) {
      throw std::runtime_error("empty engine file: " + engine_path);
    }
    std::vector<char> engine_data(static_cast<size_t>(size));
    if (!f.read(engine_data.data(), size)) {
      throw std::runtime_error("failed to read engine bytes");
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("createInferRuntime failed");
    }

    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr));
    if (!engine_) {
      throw std::runtime_error("deserializeCudaEngine failed");
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("createExecutionContext failed");
    }

    const int nb = engine_->getNbBindings();
    for (int i = 0; i < nb; ++i) {
      if (engine_->bindingIsInput(i)) {
        input_index_ = i;
      } else {
        output_index_ = i;
      }
    }
    if (input_index_ < 0 || output_index_ < 0) {
      throw std::runtime_error("failed to resolve input/output bindings");
    }

    nvinfer1::Dims input_dims = engine_->getBindingDimensions(input_index_);
    if (input_dims.nbDims == 4) {
      input_c_ = input_dims.d[1] > 0 ? input_dims.d[1] : 3;
      input_h_ = input_dims.d[2] > 0 ? input_dims.d[2] : 640;
      input_w_ = input_dims.d[3] > 0 ? input_dims.d[3] : 640;
    } else if (input_dims.nbDims == 3) {
      input_c_ = input_dims.d[0] > 0 ? input_dims.d[0] : 3;
      input_h_ = input_dims.d[1] > 0 ? input_dims.d[1] : 640;
      input_w_ = input_dims.d[2] > 0 ? input_dims.d[2] : 640;
    } else {
      throw std::runtime_error("unexpected input dims");
    }

    if (input_dims.d[0] < 0) {
      nvinfer1::Dims4 profile_dims{1, input_c_, input_h_, input_w_};
      if (!context_->setBindingDimensions(input_index_, profile_dims)) {
        throw std::runtime_error("setBindingDimensions failed");
      }
    }
    if (!context_->allInputDimensionsSpecified()) {
      throw std::runtime_error("dynamic input dims unresolved");
    }

    const nvinfer1::Dims out_dims = context_->getBindingDimensions(output_index_);
    output_count_ = static_cast<size_t>(volume(out_dims));
    host_output_.assign(output_count_, 0.0f);

    if (out_dims.nbDims == 3) {
      if (out_dims.d[1] == 84 || out_dims.d[1] == 85) {
        output_attr_count_ = static_cast<size_t>(out_dims.d[1]);
        output_box_count_ = static_cast<size_t>(out_dims.d[2]);
      } else {
        output_attr_count_ = static_cast<size_t>(out_dims.d[2]);
        output_box_count_ = static_cast<size_t>(out_dims.d[1]);
      }
    } else if (out_dims.nbDims == 2) {
      output_box_count_ = static_cast<size_t>(out_dims.d[0]);
      output_attr_count_ = static_cast<size_t>(out_dims.d[1]);
    } else {
      throw std::runtime_error("unexpected output dims");
    }

    host_input_.assign(static_cast<size_t>(input_c_ * input_h_ * input_w_), 0.0f);

    cuda_check(cudaStreamCreate(&stream_), "cudaStreamCreate failed");
    cuda_check(cudaMalloc(&device_input_, host_input_.size() * sizeof(float)), "cudaMalloc input failed");
    cuda_check(cudaMalloc(&device_output_, host_output_.size() * sizeof(float)), "cudaMalloc output failed");
  }

  void release() {
    if (device_output_) {
      cudaFree(device_output_);
      device_output_ = nullptr;
    }
    if (device_input_) {
      cudaFree(device_input_);
      device_input_ = nullptr;
    }
    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    context_.reset();
    engine_.reset();
    runtime_.reset();
  }

  void preprocess(const uint8_t* rgb, int src_w, int src_h) {
    std::fill(host_input_.begin(), host_input_.end(), 0.0f);
    scale_ = std::min(static_cast<float>(input_w_) / static_cast<float>(src_w),
                      static_cast<float>(input_h_) / static_cast<float>(src_h));
    const int resized_w = static_cast<int>(std::round(src_w * scale_));
    const int resized_h = static_cast<int>(std::round(src_h * scale_));
    pad_x_ = static_cast<float>(input_w_ - resized_w) * 0.5f;
    pad_y_ = static_cast<float>(input_h_ - resized_h) * 0.5f;

    for (int dy = 0; dy < input_h_; ++dy) {
      for (int dx = 0; dx < input_w_; ++dx) {
        float src_x = (static_cast<float>(dx) - pad_x_) / scale_;
        float src_y = (static_cast<float>(dy) - pad_y_) / scale_;
        src_x = std::max(0.0f, std::min(src_x, static_cast<float>(src_w - 1)));
        src_y = std::max(0.0f, std::min(src_y, static_cast<float>(src_h - 1)));

        const int x0 = static_cast<int>(std::floor(src_x));
        const int y0 = static_cast<int>(std::floor(src_y));
        const int x1 = std::min(x0 + 1, src_w - 1);
        const int y1 = std::min(y0 + 1, src_h - 1);
        const float wx = src_x - static_cast<float>(x0);
        const float wy = src_y - static_cast<float>(y0);

        const int idx00 = (y0 * src_w + x0) * 3;
        const int idx01 = (y0 * src_w + x1) * 3;
        const int idx10 = (y1 * src_w + x0) * 3;
        const int idx11 = (y1 * src_w + x1) * 3;

        for (int c = 0; c < 3; ++c) {
          const float v00 = static_cast<float>(rgb[idx00 + c]);
          const float v01 = static_cast<float>(rgb[idx01 + c]);
          const float v10 = static_cast<float>(rgb[idx10 + c]);
          const float v11 = static_cast<float>(rgb[idx11 + c]);
          const float v0 = v00 + (v01 - v00) * wx;
          const float v1 = v10 + (v11 - v10) * wx;
          const float v = (v0 + (v1 - v0) * wy) / 255.0f;

          const size_t out_idx = static_cast<size_t>(c) * input_h_ * input_w_ +
                                 static_cast<size_t>(dy) * input_w_ + static_cast<size_t>(dx);
          host_input_[out_idx] = v;
        }
      }
    }
  }

  std::vector<Detection> postprocess(int src_w, int src_h, float conf_thresh, float nms_thresh) const {
    std::vector<Detection> candidates;
    candidates.reserve(256);
    const auto& names = coco_names();

    const bool chw_layout = (output_attr_count_ <= 128 && output_box_count_ > 100);
    for (size_t i = 0; i < output_box_count_; ++i) {
      float cx = 0.0f, cy = 0.0f, w = 0.0f, h = 0.0f;
      int best_cls = -1;
      float best_conf = 0.0f;

      if (chw_layout) {
        cx = host_output_[0 * output_box_count_ + i];
        cy = host_output_[1 * output_box_count_ + i];
        w = host_output_[2 * output_box_count_ + i];
        h = host_output_[3 * output_box_count_ + i];

        for (size_t c = 4; c < output_attr_count_; ++c) {
          const float score = host_output_[c * output_box_count_ + i];
          if (score > best_conf) {
            best_conf = score;
            best_cls = static_cast<int>(c - 4);
          }
        }
      } else {
        const size_t row = i * output_attr_count_;
        cx = host_output_[row + 0];
        cy = host_output_[row + 1];
        w = host_output_[row + 2];
        h = host_output_[row + 3];
        for (size_t c = 4; c < output_attr_count_; ++c) {
          const float score = host_output_[row + c];
          if (score > best_conf) {
            best_conf = score;
            best_cls = static_cast<int>(c - 4);
          }
        }
      }

      if (best_conf < conf_thresh || best_cls < 0 || best_cls >= static_cast<int>(names.size())) {
        continue;
      }

      float x = cx - 0.5f * w;
      float y = cy - 0.5f * h;
      x = (x - pad_x_) / scale_;
      y = (y - pad_y_) / scale_;
      w /= scale_;
      h /= scale_;

      x = std::max(0.0f, std::min(x, static_cast<float>(src_w - 1)));
      y = std::max(0.0f, std::min(y, static_cast<float>(src_h - 1)));
      w = std::max(0.0f, std::min(w, static_cast<float>(src_w) - x));
      h = std::max(0.0f, std::min(h, static_cast<float>(src_h) - y));

      candidates.push_back(Detection{x, y, w, h, best_conf, best_cls, names[best_cls]});
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Detection& a, const Detection& b) { return a.conf > b.conf; });

    std::vector<Detection> filtered;
    std::vector<bool> removed(candidates.size(), false);
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (removed[i]) {
        continue;
      }
      filtered.push_back(candidates[i]);
      for (size_t j = i + 1; j < candidates.size(); ++j) {
        if (removed[j]) {
          continue;
        }
        if (candidates[i].class_id != candidates[j].class_id) {
          continue;
        }
        if (iou(candidates[i], candidates[j]) > nms_thresh) {
          removed[j] = true;
        }
      }
    }
    return filtered;
  }

  Logger logger_;
  RuntimePtr runtime_;
  EnginePtr engine_;
  ContextPtr context_;
  cudaStream_t stream_;
  int input_index_;
  int output_index_;
  int input_w_;
  int input_h_;
  int input_c_;
  size_t output_count_;
  size_t output_box_count_;
  size_t output_attr_count_;
  void* device_input_;
  void* device_output_;
  std::vector<float> host_input_;
  std::vector<float> host_output_;
  float scale_;
  float pad_x_;
  float pad_y_;
};

}  // namespace rover_vision
EOF

RUN cat > /opt/rover/src/yolo_ros_node.cpp <<'EOF'
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <array>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "yolo_trt.hpp"

using rover_vision::Detection;
using rover_vision::YoloTRT;

class YoloDetectorNode : public rclcpp::Node {
 public:
  YoloDetectorNode() : Node("yolo_detector_node") {
    const std::string default_engine = YoloTRT::default_engine_path();
    conf_threshold_ = this->declare_parameter<double>("conf_threshold", 0.5);
    engine_path_ = this->declare_parameter<std::string>("engine_path", default_engine);

    if (!std::filesystem::exists(engine_path_)) {
      RCLCPP_WARN(this->get_logger(),
                  "YOLO engine not found: %s (node will wait until engine is mounted)",
                  engine_path_.c_str());
    } else {
      detector_ = std::make_unique<YoloTRT>(engine_path_);
    }

    det_pub_ = this->create_publisher<std_msgs::msg::String>("/vision/detections", 10);
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/image_annotated", 10);
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&YoloDetectorNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "YOLO node ready. engine=%s conf=%.2f",
                engine_path_.c_str(), conf_threshold_);
  }

 private:
  static std::string json_escape(const std::string& in) {
    std::ostringstream os;
    for (char c : in) {
      switch (c) {
        case '"':
          os << "\\\"";
          break;
        case '\\':
          os << "\\\\";
          break;
        case '\n':
          os << "\\n";
          break;
        case '\r':
          os << "\\r";
          break;
        case '\t':
          os << "\\t";
          break;
        default:
          os << c;
      }
    }
    return os.str();
  }

  static sensor_msgs::msg::Image cv_to_ros_image(const cv::Mat& mat,
                                                 const std_msgs::msg::Header& header) {
    sensor_msgs::msg::Image out;
    out.header = header;
    out.height = static_cast<uint32_t>(mat.rows);
    out.width = static_cast<uint32_t>(mat.cols);
    out.encoding = "bgr8";
    out.is_bigendian = false;
    out.step = static_cast<sensor_msgs::msg::Image::_step_type>(mat.step);
    const size_t bytes = mat.step * mat.rows;
    out.data.resize(bytes);
    std::memcpy(out.data.data(), mat.data, bytes);
    return out;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!detector_) {
      if (std::filesystem::exists(engine_path_)) {
        try {
          detector_ = std::make_unique<YoloTRT>(engine_path_);
          RCLCPP_INFO(this->get_logger(), "YOLO engine loaded: %s", engine_path_.c_str());
        } catch (const std::exception& e) {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Engine load failed: %s", e.what());
          return;
        }
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for engine file: %s", engine_path_.c_str());
        return;
      }
    }

    if (msg->encoding != "bgr8") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Unsupported encoding: %s (expected bgr8)", msg->encoding.c_str());
      return;
    }
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      return;
    }

    cv::Mat bgr(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
                const_cast<uint8_t*>(msg->data.data()), msg->step);
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

    std::vector<Detection> detections;
    const auto t0 = std::chrono::high_resolution_clock::now();
    try {
      detections = detector_->detect(rgb.data, rgb.cols, rgb.rows, static_cast<float>(conf_threshold_), 0.45f);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Detection failed: %s", e.what());
      return;
    }
    const double dt_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();
    RCLCPP_DEBUG(this->get_logger(), "Inference: %.2f ms", dt_ms);

    cv::Mat annotated = bgr.clone();
    for (const auto& d : detections) {
      const cv::Rect box(static_cast<int>(d.x), static_cast<int>(d.y),
                         static_cast<int>(d.w), static_cast<int>(d.h));
      cv::rectangle(annotated, box, cv::Scalar(0, 255, 0), 2);
      std::ostringstream label;
      label << d.class_name << " " << std::fixed << std::setprecision(2) << d.conf;
      const int base_y = std::max(0, static_cast<int>(d.y) - 8);
      cv::putText(annotated, label.str(), cv::Point(static_cast<int>(d.x), base_y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }

    const auto stamp = msg->header.stamp;
    const double timestamp = static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;

    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{";
    json << "\"timestamp\":" << timestamp << ",";
    json << "\"frame_id\":\"" << json_escape(msg->header.frame_id) << "\",";
    json << "\"detections\":[";
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto& d = detections[i];
      const float cx = d.x + d.w * 0.5f;
      const float cy = d.y + d.h * 0.5f;
      json << "{";
      json << "\"class\":\"" << json_escape(d.class_name) << "\",";
      json << "\"conf\":" << d.conf << ",";
      json << "\"bbox\":[" << d.x << "," << d.y << "," << d.w << "," << d.h << "],";
      json << "\"center\":[" << cx << "," << cy << "]";
      json << "}";
      if (i + 1 < detections.size()) {
        json << ",";
      }
    }
    json << "]";
    json << "}";

    std_msgs::msg::String det_msg;
    det_msg.data = json.str();
    det_pub_->publish(det_msg);

    sensor_msgs::msg::Image annotated_msg = cv_to_ros_image(annotated, msg->header);
    img_pub_->publish(annotated_msg);
  }

  double conf_threshold_;
  std::string engine_path_;
  std::unique_ptr<YoloTRT> detector_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YoloDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
EOF

RUN cat > /opt/rover/src/CMakeLists_yolo.txt <<'EOF'
cmake_minimum_required(VERSION 3.10)
project(yolo_trt_ros2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

file(COPY /opt/rover/include/yolo_trt.hpp DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include)

add_executable(yolo_ros_node /opt/rover/src/yolo_ros_node.cpp)
target_include_directories(yolo_ros_node PRIVATE
  /opt/rover/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  /usr/include/aarch64-linux-gnu
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(yolo_ros_node rclcpp sensor_msgs std_msgs)
target_link_libraries(yolo_ros_node
  nvinfer
  cudart
  ${OpenCV_LIBS}
)

install(TARGETS yolo_ros_node DESTINATION lib/${PROJECT_NAME})
install(FILES /opt/rover/include/yolo_trt.hpp DESTINATION include)

ament_package()
EOF

# Keep sources for runtime build; skip heavy/fragile OpenCV link check at image build time.
RUN set -eux; \
  cp /opt/rover/src/CMakeLists_yolo.txt /opt/rover/src/CMakeLists.txt

# ============================================================================
# Stage 4: VLM Runtime (Moondream2 via llama.cpp vision)
# Estimated incremental size over vision-runtime: ~0.1-0.2GB (binaries only)
# ============================================================================
FROM vision-runtime AS vlm-runtime

ENV VLM_MODEL_PATH=/opt/rover/models/vlm/moondream2-text-model-f16.gguf \
  VLM_MMPROJ_PATH=/opt/rover/models/vlm/moondream2-mmproj-f16.gguf

# Block A: VLM model directory (models are volume-mounted from host at runtime)
RUN mkdir -p /opt/rover/models/vlm

# Block B: VLM ROS2 node source
RUN set -eux; \
  mkdir -p /opt/rover/src
RUN cat > /opt/rover/src/vlm_ros_node.cpp <<'EOF'
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <vector>

#include <curl/curl.h>

class VlmNode : public rclcpp::Node {
 public:
  VlmNode() : Node("vlm_node") {
    response_pub_ = this->create_publisher<std_msgs::msg::String>("/vlm/response", 10);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&VlmNode::on_image, this, std::placeholders::_1));
    query_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/vlm/query", 10, std::bind(&VlmNode::on_query, this, std::placeholders::_1));

    start_llama_server();
    RCLCPP_INFO(this->get_logger(), "VLM node ready. model=%s mmproj=%s",
                std::getenv("VLM_MODEL_PATH") ? std::getenv("VLM_MODEL_PATH") : "",
                std::getenv("VLM_MMPROJ_PATH") ? std::getenv("VLM_MMPROJ_PATH") : "");
  }

  ~VlmNode() override { stop_llama_server(); }

 private:
  static std::string json_escape(const std::string &in) {
    std::ostringstream os;
    for (char c : in) {
      switch (c) {
        case '"':
          os << "\\\"";
          break;
        case '\\':
          os << "\\\\";
          break;
        case '\b':
          os << "\\b";
          break;
        case '\f':
          os << "\\f";
          break;
        case '\n':
          os << "\\n";
          break;
        case '\r':
          os << "\\r";
          break;
        case '\t':
          os << "\\t";
          break;
        default:
          if (static_cast<unsigned char>(c) < 0x20) {
            os << "\\u" << std::hex << std::setw(4) << std::setfill('0')
               << static_cast<int>(static_cast<unsigned char>(c)) << std::dec
               << std::setfill(' ');
          } else {
            os << c;
          }
      }
    }
    return os.str();
  }

  static std::string json_unescape(const std::string &in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
      if (in[i] == '\\' && i + 1 < in.size()) {
        const char n = in[++i];
        switch (n) {
          case '"':
            out.push_back('"');
            break;
          case '\\':
            out.push_back('\\');
            break;
          case '/':
            out.push_back('/');
            break;
          case 'b':
            out.push_back('\b');
            break;
          case 'f':
            out.push_back('\f');
            break;
          case 'n':
            out.push_back('\n');
            break;
          case 'r':
            out.push_back('\r');
            break;
          case 't':
            out.push_back('\t');
            break;
          default:
            out.push_back(n);
            break;
        }
      } else {
        out.push_back(in[i]);
      }
    }
    return out;
  }

  static std::string extract_json_string_field(const std::string &json,
                                               const std::string &key) {
    const std::string marker = "\"" + key + "\"";
    const size_t kpos = json.find(marker);
    if (kpos == std::string::npos) {
      return "";
    }
    const size_t colon = json.find(':', kpos + marker.size());
    if (colon == std::string::npos) {
      return "";
    }
    const size_t first_quote = json.find('"', colon + 1);
    if (first_quote == std::string::npos) {
      return "";
    }
    std::string raw;
    raw.reserve(256);
    bool esc = false;
    for (size_t i = first_quote + 1; i < json.size(); ++i) {
      const char c = json[i];
      if (!esc && c == '"') {
        break;
      }
      if (!esc && c == '\\') {
        esc = true;
        raw.push_back(c);
        continue;
      }
      esc = false;
      raw.push_back(c);
    }
    return json_unescape(raw);
  }

  static std::string base64_encode(const unsigned char *data, size_t len) {
    static const char table[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    size_t i = 0;
    while (i + 2 < len) {
      const uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
                         (static_cast<uint32_t>(data[i + 1]) << 8) |
                         static_cast<uint32_t>(data[i + 2]);
      out.push_back(table[(n >> 18) & 63]);
      out.push_back(table[(n >> 12) & 63]);
      out.push_back(table[(n >> 6) & 63]);
      out.push_back(table[n & 63]);
      i += 3;
    }
    if (i < len) {
      const uint32_t n = static_cast<uint32_t>(data[i]) << 16;
      out.push_back(table[(n >> 18) & 63]);
      if (i + 1 < len) {
        const uint32_t n2 = n | (static_cast<uint32_t>(data[i + 1]) << 8);
        out.push_back(table[(n2 >> 12) & 63]);
        out.push_back(table[(n2 >> 6) & 63]);
        out.push_back('=');
      } else {
        out.push_back(table[(n >> 12) & 63]);
        out.push_back('=');
        out.push_back('=');
      }
    }
    return out;
  }

  static size_t curl_write_cb(void *contents, size_t size, size_t nmemb,
                              void *userp) {
    const size_t real_size = size * nmemb;
    auto *buf = reinterpret_cast<std::string *>(userp);
    buf->append(reinterpret_cast<char *>(contents), real_size);
    return real_size;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty() || msg->height == 0 || msg->width == 0) {
      return;
    }

    cv::Mat frame;
    if (msg->encoding == "bgr8") {
      frame = cv::Mat(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
                      const_cast<uint8_t *>(msg->data.data()), msg->step).clone();
    } else if (msg->encoding == "rgb8") {
      cv::Mat rgb(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
                  const_cast<uint8_t *>(msg->data.data()), msg->step);
      cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(frame_mu_);
      latest_frame_ = frame;
      has_frame_ = true;
    }
  }

  void on_query(const std_msgs::msg::String::SharedPtr msg) {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mu_);
      if (!has_frame_ || latest_frame_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No image frame buffered yet; skipping query");
        return;
      }
      frame = latest_frame_.clone();
    }

    std::vector<uchar> jpeg;
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
    if (!cv::imencode(".jpg", frame, jpeg, params)) {
      RCLCPP_ERROR(this->get_logger(), "JPEG encoding failed");
      return;
    }

    const std::string jpg_b64 = base64_encode(jpeg.data(), jpeg.size());
    const std::string query = msg->data;
    const std::string payload = build_payload(query, jpg_b64);
    const std::string raw = call_llama_completion(payload);

    std::string answer = extract_json_string_field(raw, "content");
    if (answer.empty()) {
      answer = extract_json_string_field(raw, "response");
    }
    if (answer.empty()) {
      answer = extract_json_string_field(raw, "text");
    }
    if (answer.empty()) {
      answer = "Failed to parse llama-server response";
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{";
    out << "\"query\":\"" << json_escape(query) << "\",";
    out << "\"answer\":\"" << json_escape(answer) << "\",";
    out << "\"timestamp\":" << this->now().seconds();
    out << "}";

    std_msgs::msg::String res;
    res.data = out.str();
    response_pub_->publish(res);
  }

  std::string build_payload(const std::string &query, const std::string &jpg_b64) const {
    std::ostringstream ss;
    ss << "{";
    ss << "\"prompt\":\"USER: <image>\\n" << json_escape(query) << "\\nASSISTANT:\",";
    ss << "\"image_data\":[{\"data\":\"" << jpg_b64 << "\",\"id\":0}],";
    ss << "\"temperature\":0.1,";
    ss << "\"n_predict\":128";
    ss << "}";
    return ss.str();
  }

  std::string call_llama_completion(const std::string &payload) const {
    CURL *curl = curl_easy_init();
    if (!curl) {
      return "{\"error\":\"curl_init_failed\"}";
    }

    std::string response;
    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:8081/completion");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(payload.size()));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &VlmNode::curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 25L);

    const CURLcode rc = curl_easy_perform(curl);
    if (rc != CURLE_OK) {
      std::ostringstream err;
      err << "{\"error\":\"" << curl_easy_strerror(rc) << "\"}";
      response = err.str();
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return response;
  }

  void start_llama_server() {
    if (server_thread_.joinable()) {
      return;
    }
    server_stop_.store(false);
    server_thread_ = std::thread([this]() {
      const char *model = std::getenv("VLM_MODEL_PATH");
      const char *mmproj = std::getenv("VLM_MMPROJ_PATH");
      if (!model || !mmproj) {
        RCLCPP_ERROR(this->get_logger(), "VLM model env vars are not set");
        return;
      }

      pid_t pid = fork();
      if (pid < 0) {
        RCLCPP_ERROR(this->get_logger(), "fork() failed for llama-server");
        return;
      }

      if (pid == 0) {
        execlp("llama-server", "llama-server", "--model", model, "--mmproj", mmproj, "--port",
               "8081", "--ctx-size", "2048", "--n-gpu-layers", "99", static_cast<char *>(nullptr));
        _exit(127);
      }

      server_pid_.store(pid);
      server_started_.store(true);
      RCLCPP_INFO(this->get_logger(), "llama-server started pid=%d", static_cast<int>(pid));

      int status = 0;
      const pid_t w = waitpid(pid, &status, 0);
      if (w > 0 && !server_stop_.load()) {
        RCLCPP_ERROR(this->get_logger(), "llama-server exited unexpectedly (status=%d)", status);
      }
      server_started_.store(false);
      server_pid_.store(-1);
    });
  }

  void stop_llama_server() {
    server_stop_.store(true);
    const pid_t pid = server_pid_.load();
    if (pid > 0) {
      kill(pid, SIGTERM);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (kill(pid, 0) == 0 || errno == EPERM) {
        kill(pid, SIGKILL);
      }
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  std::mutex frame_mu_;
  cv::Mat latest_frame_;
  bool has_frame_{false};

  std::atomic<bool> server_stop_{false};
  std::atomic<bool> server_started_{false};
  std::atomic<pid_t> server_pid_{-1};
  std::thread server_thread_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr query_sub_;
};

int main(int argc, char **argv) {
  curl_global_init(CURL_GLOBAL_DEFAULT);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VlmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  curl_global_cleanup();
  return 0;
}
EOF

# Block C: CMakeLists for vlm_ros_node
RUN cat > /opt/rover/src/CMakeLists_vlm.txt <<'EOF'
cmake_minimum_required(VERSION 3.10)
project(vlm_ros2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

add_executable(vlm_ros_node /opt/rover/src/vlm_ros_node.cpp)
target_include_directories(vlm_ros_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)
ament_target_dependencies(vlm_ros_node rclcpp sensor_msgs std_msgs)
target_link_libraries(vlm_ros_node
  ${OpenCV_LIBS}
  ${CURL_LIBRARIES}
)

install(TARGETS vlm_ros_node DESTINATION lib/${PROJECT_NAME})
ament_package()
EOF

# Block D: Build and install verification
RUN set -eux; \
  mkdir -p /opt/rover/vlm_ws && \
  cp /opt/rover/src/vlm_ros_node.cpp /opt/rover/vlm_ws/vlm_ros_node.cpp && \
  cp /opt/rover/src/CMakeLists_vlm.txt /opt/rover/vlm_ws/CMakeLists.txt && \
  cd /opt/rover/vlm_ws && \
  cmake -B build -DCMAKE_PREFIX_PATH=/opt/ros/humble -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 && \
  cmake --build build --target vlm_ros_node -j2 && \
  install -m 0755 build/vlm_ros_node /usr/local/bin/vlm_ros_node

# Cleanup
RUN rm -rf /opt/rover/vlm_ws /var/lib/apt/lists/*

# Final verification (engine binaries only, not model files)
RUN bash -c "test -f /usr/local/bin/vlm_ros_node && \
  echo 'VLM stage OK'"

# ============================================================================
# Stage 5: OCR Runtime (PaddleOCR PP-OCRv3 Korean)
# Estimated incremental size over vlm-runtime: ~0.6-0.9GB (python deps + models)
# ============================================================================
FROM vlm-runtime AS ocr-runtime

# Block A: Install PaddleOCR (CPU) — models are volume-mounted from host
RUN set -eux; \
  pip3 install --no-cache-dir \
  paddlepaddle==2.6.0 \
  paddleocr==2.7.0.3 \
  shapely pyclipper lmdb tqdm; \
  mkdir -p /opt/rover/models/ocr

# Block B: OCR ROS2 node (Python)
RUN set -eux; \
  mkdir -p /opt/rover/scripts /opt/rover/launch
RUN cat > /opt/rover/scripts/ocr_node.py <<'EOF'
#!/usr/bin/env python3
import argparse
import json
from typing import Any, List

import cv2
import paddle
import rclpy
from cv_bridge import CvBridge
from paddleocr import PaddleOCR
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String


class OcrNode(Node):
    def __init__(self) -> None:
        super().__init__("ocr_node")
        self.bridge = CvBridge()
        self.latest_image = None

        self.declare_parameter("det_model_dir", "/opt/rover/models/ocr/det")
        self.declare_parameter("rec_model_dir", "/opt/rover/models/ocr/rec_korean")
        self.declare_parameter("cls_model_dir", "/opt/rover/models/ocr/cls")

        det_model_dir = self.get_parameter("det_model_dir").get_parameter_value().string_value
        rec_model_dir = self.get_parameter("rec_model_dir").get_parameter_value().string_value
        cls_model_dir = self.get_parameter("cls_model_dir").get_parameter_value().string_value
        use_gpu = paddle.device.is_compiled_with_cuda()

        self.ocr = PaddleOCR(
            det_model_dir=det_model_dir,
            rec_model_dir=rec_model_dir,
            cls_model_dir=cls_model_dir,
            use_angle_cls=True,
            lang="korean",
            use_gpu=use_gpu,
            show_log=False,
        )

        self.result_pub = self.create_publisher(String, "/ocr/result", 10)
        self.create_subscription(Image, "/camera/color/image_raw", self.on_image, 10)
        self.create_subscription(Empty, "/ocr/trigger", self.on_trigger, 10)
        self.get_logger().info(f"OCR node ready (use_gpu={use_gpu})")

    def on_image(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")

    def on_trigger(self, _msg: Empty) -> None:
        if self.latest_image is None:
            self.get_logger().warn("No image received yet")
            return

        try:
            img = self.latest_image
            result = self.ocr.ocr(img, cls=True)
            text_blocks = self._parse_result(result)
            payload = {
                "text_blocks": text_blocks,
                "timestamp": float(self.get_clock().now().nanoseconds) / 1e9,
            }
            out = String()
            out.data = json.dumps(payload, ensure_ascii=False)
            self.result_pub.publish(out)
        except Exception as exc:
            self.get_logger().error(f"OCR failed: {exc}")

    def _parse_result(self, result: Any) -> List[dict]:
        blocks: List[dict] = []
        if not result:
            return blocks

        lines = result[0] if isinstance(result, list) and len(result) > 0 else []
        for item in lines:
            if not item or len(item) < 2:
                continue
            bbox_raw, rec = item[0], item[1]
            text = rec[0] if isinstance(rec, (list, tuple)) and len(rec) > 0 else ""
            confidence = rec[1] if isinstance(rec, (list, tuple)) and len(rec) > 1 else 0.0
            bbox = []
            for p in bbox_raw:
                if isinstance(p, (list, tuple)) and len(p) >= 2:
                    bbox.append([float(p[0]), float(p[1])])
            blocks.append(
                {
                    "text": str(text),
                    "confidence": float(confidence),
                    "bbox": bbox,
                }
            )
        return blocks


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 PaddleOCR node")
    parser.parse_args()
    rclpy.init()
    node = OcrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
EOF
RUN chmod +x /opt/rover/scripts/ocr_node.py

# Block C: Launch file
RUN cat > /opt/rover/launch/ocr.launch.py <<'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='ocr_node',  # temporary package name
                executable='ocr_node.py',
                name='ocr_node',
                output='screen',
                parameters=[
                    {'det_model_dir': '/opt/rover/models/ocr/det'},
                    {'rec_model_dir': '/opt/rover/models/ocr/rec_korean'},
                    {'cls_model_dir': '/opt/rover/models/ocr/cls'},
                ],
            )
        ]
    )
EOF

# Block D: Verification
RUN set -eux; \
  python3 /opt/rover/scripts/ocr_node.py --help || echo "OCR node OK"

# Verification: PaddleOCR import check
RUN python3 -c "from paddleocr import PaddleOCR; print('PaddleOCR OK')"

# Cleanup
RUN set -eux; \
  pip3 cache purge; \
  rm -rf /var/lib/apt/lists/*

# ============================================================================
# Stage 6: Full Runtime (Embedding + Wake Word)
# Estimated incremental size over ocr-runtime: ~0.8-1.4GB
# ============================================================================
FROM ocr-runtime AS full-runtime

ARG PORCUPINE_VERSION="3.0.2"

# Block A: KoSimCSE ONNX model is volume-mounted from host (pre-converted)
# No PyTorch needed — conversion done on host via first_run_setup.sh
RUN mkdir -p /opt/rover/models/embedding

# Block B: Porcupine wake word engine
RUN set -eux; \
  mkdir -p /opt/porcupine /opt/rover/models/wake_word; \
  wget -O /tmp/porcupine.tar.gz "https://github.com/Picovoice/porcupine/archive/refs/tags/v${PORCUPINE_VERSION}.tar.gz"; \
  tar -xzf /tmp/porcupine.tar.gz -C /tmp; \
  cp "/tmp/porcupine-${PORCUPINE_VERSION}/lib/linux/arm64/libpv_porcupine.so" /usr/local/lib/; \
  cp -r "/tmp/porcupine-${PORCUPINE_VERSION}/lib/common" /opt/porcupine/; \
  cp -r "/tmp/porcupine-${PORCUPINE_VERSION}/resources" /opt/porcupine/; \
  ldconfig; \
  pip3 install --no-cache-dir "pvporcupine==${PORCUPINE_VERSION}"; \
  python3 -c "import pvporcupine; print('Porcupine OK, version:', pvporcupine.__version__)"; \
  rm -rf /tmp/porcupine*

# Block C: Embedding model directory (GGUF/ONNX volume-mounted from host)
# No download needed — models provided via docker-compose volume mount

# Block D: Final runtime environment
ENV EMBEDDING_MODEL=/opt/rover/models/embedding/kosimcse-roberta-q8_0.gguf \
  WAKE_WORD_MODEL=/opt/rover/models/wake_word/porcupine.ppn \
  PORCUPINE_LIB=/usr/local/lib/libpv_porcupine.so

# Block E: Healthcheck (FastAPI endpoint)
HEALTHCHECK --interval=30s --timeout=10s --start-period=90s --retries=3 \
  CMD curl -f http://localhost:8080/health || exit 1

# Verification (engine binaries only, not model files)
RUN set -eux; \
  test -f /usr/local/lib/libpv_porcupine.so && \
  llama-embedding --version

# Block F: SQLite schema initialization script
# Schema file lives on host at db/init_schema.sql
# At runtime, ./db:/opt/rover/db bind mount provides both schema and DB persistence
RUN set -eux; \
  mkdir -p /opt/rover/db /opt/rover/scripts
COPY db/init_schema.sql /opt/rover/db/init_schema.sql

# Block G: Entrypoint for first-run DB initialization
COPY docker-entrypoint.sh /opt/rover/scripts/
RUN chmod +x /opt/rover/scripts/docker-entrypoint.sh

# Block H: SQLite schema verification
RUN set -eux; \
  sqlite3 /tmp/test.db < /opt/rover/db/init_schema.sql && \
  sqlite3 /tmp/test.db "SELECT name FROM sqlite_master WHERE type='table';" && \
  rm /tmp/test.db

# Block I: Health API server dependencies
RUN set -eux; \
  pip3 install --no-cache-dir fastapi uvicorn psutil requests

# Block J: FastAPI health server
RUN cat > /opt/rover/scripts/health_server.py <<'EOF'
#!/usr/bin/env python3
"""
FastAPI Health Check Server for JetRover Brain Container
Endpoints:
  GET /health — Overall system health
  GET /llm/status — LLM service status
  GET /vlm/status — VLM service status
  GET /metrics — Prometheus-compatible metrics
"""

import subprocess
import time

import psutil
from fastapi import FastAPI
from fastapi.responses import JSONResponse, PlainTextResponse

app = FastAPI(title="JetRover Health API", version="1.0")
START_TIME = time.time()

# [H2 FIX] CPU measurement cache (1초 간격 갱신)
_cpu_cache = {"value": 0.0, "timestamp": 0.0}


def get_cpu_percent():
    """캐시된 CPU 사용률 반환 (1초마다 갱신)."""
    global _cpu_cache
    now = time.time()

    if now - _cpu_cache["timestamp"] > 1.0:
        # psutil.cpu_percent(interval=None)는 첫 호출 이후부터 정확함
        _cpu_cache["value"] = psutil.cpu_percent(interval=None)
        _cpu_cache["timestamp"] = now

    return _cpu_cache["value"]


def get_gpu_memory():
    """Query NVIDIA GPU memory using tegrastats."""
    try:
        result = subprocess.run(
            ["tegrastats", "--interval", "50"],  # [H3 FIX] 50ms 샘플링
            capture_output=True,
            text=True,
            timeout=0.2,  # [H3 FIX] 200ms 여유
        )
        return {"status": "ok", "raw": result.stdout.strip()}
    except subprocess.TimeoutExpired:
        return {"status": "error", "message": "tegrastats timeout"}
    except FileNotFoundError:
        return {"status": "unavailable", "message": "tegrastats not found (not Jetson)"}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}


@app.get("/health")
async def health_check():
    """Overall system health."""
    mem = psutil.virtual_memory()
    cpu = get_cpu_percent()  # [H2 FIX] 캐시된 값 사용
    gpu = get_gpu_memory()
    return JSONResponse(
        {
            "status": "ok" if mem.available > 2.5e9 else "degraded",
            "timestamp": time.time(),
            "uptime_seconds": time.time() - START_TIME,
            "memory": {
                "total_gb": round(mem.total / 1e9, 2),
                "available_gb": round(mem.available / 1e9, 2),
                "percent_used": mem.percent,
            },
            "cpu_percent": cpu,
            "gpu": gpu,
        }
    )


@app.get("/llm/status")
async def llm_status():
    """Check if LLM server is responsive."""
    try:
        import requests

        resp = requests.get("http://localhost:11434/health", timeout=2)
        return {"status": "ok" if resp.status_code == 200 else "error"}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}


@app.get("/vlm/status")
async def vlm_status():
    """Check if VLM server is responsive."""
    try:
        import requests

        resp = requests.get("http://localhost:8081/health", timeout=2)
        return {"status": "ok" if resp.status_code == 200 else "error"}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}


@app.get("/metrics")
async def prometheus_metrics():
    """Prometheus-compatible metrics."""
    mem = psutil.virtual_memory()
    cpu = get_cpu_percent()

    return PlainTextResponse(f"""# HELP rover_memory_available_bytes Available memory in bytes
# TYPE rover_memory_available_bytes gauge
rover_memory_available_bytes {mem.available}

# HELP rover_memory_percent Memory usage percentage
# TYPE rover_memory_percent gauge
rover_memory_percent {mem.percent}

# HELP rover_cpu_percent CPU usage percentage
# TYPE rover_cpu_percent gauge
rover_cpu_percent {cpu}

# HELP rover_uptime_seconds Container uptime
# TYPE rover_uptime_seconds counter
rover_uptime_seconds {time.time() - START_TIME}
""")


if __name__ == "__main__":
    import uvicorn

    # [H2 FIX] 서버 시작 전 CPU 측정 초기화
    psutil.cpu_percent(interval=None)  # 첫 호출 (baseline)
    time.sleep(0.1)
    psutil.cpu_percent(interval=None)  # 두 번째 호출부터 정확함

    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")
EOF
RUN chmod +x /opt/rover/scripts/health_server.py

# Block K: ROS2 heartbeat node (1 Hz)
RUN set -eux; \
  mkdir -p /opt/rover/src /opt/rover/heartbeat_ws
RUN cat > /opt/rover/src/heartbeat_node.cpp <<'EOF'
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class HeartbeatNode : public rclcpp::Node {
 public:
  HeartbeatNode() : Node("heartbeat_node") {
    publisher_ = this->create_publisher<std_msgs::msg::Header>("/heartbeat", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&HeartbeatNode::publish_heartbeat, this));
    RCLCPP_INFO(this->get_logger(), "Heartbeat node started (1 Hz)");
  }

 private:
  void publish_heartbeat() {
    auto msg = std_msgs::msg::Header();
    msg.stamp = this->now();
    msg.frame_id = "brain_container";
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeartbeatNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
EOF
RUN cat > /opt/rover/heartbeat_ws/CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.10)
project(heartbeat_ws LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(heartbeat_node /opt/rover/src/heartbeat_node.cpp)
ament_target_dependencies(heartbeat_node rclcpp std_msgs)
EOF
RUN set -eux; \
  cd /opt/rover/heartbeat_ws && \
  cmake -B build -DCMAKE_PREFIX_PATH=/opt/ros/humble && \
  cmake --build build --target heartbeat_node -j2 && \
  install -m 0755 build/heartbeat_node /usr/local/bin/heartbeat_node && \
  test -x /usr/local/bin/heartbeat_node

# Block L: Supervisor process manager
RUN set -eux; \
  apt-get update && \
  apt-get install -y --no-install-recommends supervisor && \
  mkdir -p /etc/supervisor/conf.d /var/log/supervisor

# [H1 FIX] Supervisord main configuration
RUN cat > /etc/supervisor/supervisord.conf <<'EOF'
[supervisord]
nodaemon=true
logfile=/var/log/supervisor/supervisord.log
pidfile=/var/run/supervisord.pid
childlogdir=/var/log/supervisor
loglevel=info

[unix_http_server]
file=/var/run/supervisor.sock
chmod=0700

[supervisorctl]
serverurl=unix:///var/run/supervisor.sock

[rpcinterface:supervisor]
supervisor.rpcinterface_factory = supervisor.rpcinterface:make_main_rpcinterface

[include]
files = /etc/supervisor/conf.d/*.conf
EOF

# [M1 FIX] Program configurations with log rotation
RUN cat > /etc/supervisor/conf.d/jetrover.conf <<'EOF'
[program:health_server]
command=python3 /opt/rover/scripts/health_server.py
autostart=true
autorestart=true
priority=10
stdout_logfile=/var/log/supervisor/health_server.log
stdout_logfile_maxbytes=10MB
stdout_logfile_backups=3
stderr_logfile=/var/log/supervisor/health_server_err.log
stderr_logfile_maxbytes=10MB
stderr_logfile_backups=3

[program:heartbeat_node]
command=/bin/bash -c "source /opt/ros/humble/setup.bash && /usr/local/bin/heartbeat_node"
autostart=true
autorestart=true
priority=20
stdout_logfile=/var/log/supervisor/heartbeat.log
stdout_logfile_maxbytes=10MB
stdout_logfile_backups=3
stderr_logfile=/var/log/supervisor/heartbeat_err.log
stderr_logfile_maxbytes=10MB
stderr_logfile_backups=3
EOF

# Block M: Health server verification
RUN set -eux; \
  python3 -m py_compile /opt/rover/scripts/health_server.py; \
  test -f /usr/local/bin/heartbeat_node

# Cleanup
RUN set -eux; \
  pip3 cache purge; \
  rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/opt/rover/scripts/docker-entrypoint.sh"]
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
