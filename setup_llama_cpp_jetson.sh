#!/usr/bin/env bash
set -euo pipefail

# Run inside dustynv/l4t-pytorch:r36.2.0 container.
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  build-essential cmake ninja-build git python3-dev

python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install --upgrade -i https://pypi.org/simple "huggingface_hub<1.0"

export FORCE_CMAKE=1
export CUDACXX=/usr/local/cuda/bin/nvcc
export CMAKE_ARGS="-DGGML_CUDA=on -DCMAKE_CUDA_ARCHITECTURES=87"

python3 -m pip install --no-cache-dir --force-reinstall --no-binary=llama-cpp-python \
  -i https://pypi.org/simple llama-cpp-python

mkdir -p /app/models/llm
huggingface-cli download Qwen/Qwen2.5-1.5B-Instruct-GGUF \
  qwen2.5-1.5b-instruct-q4_k_m.gguf \
  --local-dir /app/models/llm \
  --local-dir-use-symlinks False

ln -sf /app/models/llm/qwen2.5-1.5b-instruct-q4_k_m.gguf \
  /app/models/llm/Qwen2.5-1.5B-Instruct-Q4_K_M.gguf

echo "Done:"
echo "  /app/models/llm/Qwen2.5-1.5B-Instruct-Q4_K_M.gguf"
