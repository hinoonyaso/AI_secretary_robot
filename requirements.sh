#!/bin/bash
set -ex

# Update package lists
sudo apt update

# Install dependencies for llama-cpp-python and other tools
sudo apt install -y build-essential cmake ninja-build git python3-pip python3-dev libcurl4-openssl-dev curl jq

# Install ROS2 dependencies if rosdep is available
if command -v rosdep &> /dev/null; then
  sudo rosdep init || true
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
fi

# Upgrade pip
python3 -m pip install --upgrade pip setuptools wheel

# Install required python packages
python3 -m pip install -r requirements.txt || true # If requirements.txt exists

# Install llama-cpp-python with CUDA support for Jetson (CUDA 11/12)
export FORCE_CMAKE=1
export CUDACXX=/usr/local/cuda/bin/nvcc
export CMAKE_ARGS="-DGGML_CUDA=on -DCMAKE_CUDA_ARCHITECTURES=87"
python3 -m pip install --no-cache-dir --force-reinstall --no-binary=llama-cpp-python -i https://pypi.org/simple llama-cpp-python

echo "Requirements installed successfully."
