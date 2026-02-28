#!/bin/bash
# JetRover first-run host setup
# - Downloads model files on host filesystem
# - Keeps Docker image slim (models mounted via volume)

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODELS_DIR="${PROJECT_ROOT}/models"

LLM_DIR="${MODELS_DIR}/llm"
VLM_DIR="${MODELS_DIR}/vlm"
EMBED_DIR="${MODELS_DIR}/embedding"
WAKE_DIR="${MODELS_DIR}/wake_word"
STT_DIR="${MODELS_DIR}/stt"
TTS_DIR="${MODELS_DIR}/tts"
OCR_DIR="${MODELS_DIR}/ocr"
VISION_DIR="${MODELS_DIR}/vision"

mkdir -p "${LLM_DIR}" "${VLM_DIR}" "${EMBED_DIR}" "${WAKE_DIR}" "${STT_DIR}" "${TTS_DIR}" "${OCR_DIR}" "${VISION_DIR}"

download_if_missing() {
  local url="$1"
  local out="$2"
  local name="$3"

  if [ -f "${out}" ]; then
    echo "[SKIP] ${name}: ${out}"
    return 0
  fi

  if [ -z "${url}" ]; then
    echo "[WARN] ${name} URL is empty. skip."
    return 0
  fi

  echo "[GET] ${name}"
  wget --progress=dot:giga -O "${out}" "${url}"
  test -s "${out}"
  echo "[OK] ${name}: ${out}"
}

echo "========================================="
echo "JetRover First Run Host Setup"
echo "Project root: ${PROJECT_ROOT}"
echo "Models dir  : ${MODELS_DIR}"
echo "========================================="

# Required by docker-compose llm_service / vlm_service
LLM_URL="${LLM_URL:-https://huggingface.co/bartowski/Qwen2.5-3B-Instruct-GGUF/resolve/main/Qwen2.5-3B-Instruct-Q4_K_M.gguf}"
VLM_TEXT_URL="${VLM_TEXT_URL:-https://huggingface.co/moondream/moondream2-gguf/resolve/main/moondream2-text-model-f16.gguf}"
VLM_MMPROJ_URL="${VLM_MMPROJ_URL:-https://huggingface.co/moondream/moondream2-gguf/resolve/main/moondream2-mmproj-f16.gguf}"
WAKE_WORD_PPN_URL="${WAKE_WORD_PPN_URL:-https://raw.githubusercontent.com/Picovoice/porcupine/v3.0/resources/keyword_files/linux/porcupine_linux.ppn}"

# Optional (set EMBEDDING_GGUF_URL if you have a KoSimCSE GGUF source)
EMBEDDING_GGUF_URL="${EMBEDDING_GGUF_URL:-}"

# STT: Moonshine Tiny Korean
STT_URL="${STT_URL:-https://huggingface.co/UsefulSensors/moonshine/resolve/main/onnx/base/preprocess.onnx}"
STT_ENCODER_URL="${STT_ENCODER_URL:-https://huggingface.co/UsefulSensors/moonshine/resolve/main/onnx/base/encode.onnx}"
STT_UNCACHED_DECODER_URL="${STT_UNCACHED_DECODER_URL:-https://huggingface.co/UsefulSensors/moonshine/resolve/main/onnx/base/uncached_decode.onnx}"
STT_CACHED_DECODER_URL="${STT_CACHED_DECODER_URL:-https://huggingface.co/UsefulSensors/moonshine/resolve/main/onnx/base/cached_decode.onnx}"

# TTS: Piper Korean Voice
TTS_MODEL_URL="${TTS_MODEL_URL:-https://huggingface.co/rhasspy/piper-voices/resolve/main/ko/ko_KR/kss/medium/ko_KR-kss-medium.onnx}"
TTS_CONFIG_URL="${TTS_CONFIG_URL:-https://huggingface.co/rhasspy/piper-voices/resolve/main/ko/ko_KR/kss/medium/ko_KR-kss-medium.onnx.json}"

# OCR: PaddleOCR Korean (PP-OCRv3)
OCR_DET_URL="${OCR_DET_URL:-https://paddleocr.bj.bcebos.com/PP-OCRv3/chinese/ch_PP-OCRv3_det_infer.tar}"
OCR_REC_URL="${OCR_REC_URL:-https://paddleocr.bj.bcebos.com/PP-OCRv3/korean/korean_PP-OCRv3_rec_infer.tar}"
OCR_CLS_URL="${OCR_CLS_URL:-https://paddleocr.bj.bcebos.com/dygraph_v2.0/ch/ch_ppocr_mobile_v2.0_cls_infer.tar}"

# Vision: YOLO11n ONNX
YOLO_URL="${YOLO_URL:-https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx}"

download_if_missing "${LLM_URL}" "${LLM_DIR}/qwen2.5-3b-instruct-q4_k_m.gguf" "LLM GGUF"
download_if_missing "${VLM_TEXT_URL}" "${VLM_DIR}/moondream2-text-model-f16.gguf" "VLM text GGUF"
download_if_missing "${VLM_MMPROJ_URL}" "${VLM_DIR}/moondream2-mmproj-f16.gguf" "VLM mmproj GGUF"
download_if_missing "${WAKE_WORD_PPN_URL}" "${WAKE_DIR}/porcupine.ppn" "Wake word PPN"
download_if_missing "${EMBEDDING_GGUF_URL}" "${EMBED_DIR}/kosimcse-roberta-q8_0.gguf" "Embedding GGUF (optional)"

echo ""
echo "=== Downloading STT Models ==="
mkdir -p "${STT_DIR}/moonshine-tiny-ko"
download_if_missing "${STT_URL}" "${STT_DIR}/moonshine-tiny-ko/preprocess.onnx" "STT preprocess"
download_if_missing "${STT_ENCODER_URL}" "${STT_DIR}/moonshine-tiny-ko/encode.onnx" "STT encoder"
download_if_missing "${STT_UNCACHED_DECODER_URL}" "${STT_DIR}/moonshine-tiny-ko/uncached_decode.onnx" "STT uncached decoder"
download_if_missing "${STT_CACHED_DECODER_URL}" "${STT_DIR}/moonshine-tiny-ko/cached_decode.onnx" "STT cached decoder"

echo ""
echo "=== Downloading TTS Models ==="
download_if_missing "${TTS_MODEL_URL}" "${TTS_DIR}/ko_KR-kss-medium.onnx" "TTS Korean voice"
download_if_missing "${TTS_CONFIG_URL}" "${TTS_DIR}/ko_KR-kss-medium.onnx.json" "TTS voice config"

echo ""
echo "=== Downloading OCR Models ==="
mkdir -p "${OCR_DIR}/det" "${OCR_DIR}/rec_korean" "${OCR_DIR}/cls"

# OCR Detection model (tar extraction required)
if [ ! -f "${OCR_DIR}/det/inference.pdmodel" ]; then
  echo "[GET] OCR Detection model"
  wget --progress=dot:giga -O /tmp/ocr_det.tar "${OCR_DET_URL}"
  tar -xf /tmp/ocr_det.tar -C /tmp/
  mv /tmp/ch_PP-OCRv3_det_infer/* "${OCR_DIR}/det/"
  rm -rf /tmp/ocr_det.tar /tmp/ch_PP-OCRv3_det_infer
  echo "[OK] OCR Detection model"
else
  echo "[SKIP] OCR Detection model exists"
fi

# OCR Recognition model (Korean)
if [ ! -f "${OCR_DIR}/rec_korean/inference.pdmodel" ]; then
  echo "[GET] OCR Recognition model (Korean)"
  wget --progress=dot:giga -O /tmp/ocr_rec.tar "${OCR_REC_URL}"
  tar -xf /tmp/ocr_rec.tar -C /tmp/
  mv /tmp/korean_PP-OCRv3_rec_infer/* "${OCR_DIR}/rec_korean/"
  rm -rf /tmp/ocr_rec.tar /tmp/korean_PP-OCRv3_rec_infer
  echo "[OK] OCR Recognition model"
else
  echo "[SKIP] OCR Recognition model exists"
fi

# OCR Classification model
if [ ! -f "${OCR_DIR}/cls/inference.pdmodel" ]; then
  echo "[GET] OCR Classification model"
  wget --progress=dot:giga -O /tmp/ocr_cls.tar "${OCR_CLS_URL}"
  tar -xf /tmp/ocr_cls.tar -C /tmp/
  mv /tmp/ch_ppocr_mobile_v2.0_cls_infer/* "${OCR_DIR}/cls/"
  rm -rf /tmp/ocr_cls.tar /tmp/ch_ppocr_mobile_v2.0_cls_infer
  echo "[OK] OCR Classification model"
else
  echo "[SKIP] OCR Classification model exists"
fi

echo ""
echo "=== Downloading Vision Models ==="
download_if_missing "${YOLO_URL}" "${VISION_DIR}/yolo11n.onnx" "YOLO11n ONNX"

echo ""
echo "Host model setup complete."
echo "Now run: docker compose up -d"
