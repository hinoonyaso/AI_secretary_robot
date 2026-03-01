#!/bin/bash
# TTS 의존성 자동 설치 스크립트
# 작성일: 2026-03-01
# 용도: espeak-ng, edge-tts, piper 자동 설치

set -e  # 에러 발생 시 즉시 종료

echo "========================================="
echo "TTS 의존성 설치 스크립트"
echo "========================================="
echo ""

# ============================================
# 1. espeak-ng 설치 (필수 fallback 엔진)
# ============================================
echo "[1/3] espeak-ng 설치 중..."
if command -v espeak-ng &> /dev/null; then
    echo "✅ espeak-ng 이미 설치됨: $(espeak-ng --version | head -1)"
else
    sudo apt update
    sudo apt install -y espeak-ng espeak-ng-data
    echo "✅ espeak-ng 설치 완료"
fi

# espeak-ng 테스트
echo "🔊 espeak-ng 테스트 (한국어 음성 재생)..."
espeak-ng -v ko "안녕하세요, 로버입니다" --stdout | aplay 2>/dev/null || echo "⚠️  ALSA 재생 실패 (정상: 헤드리스 환경)"

echo ""

# ============================================
# 2. edge-tts 설치 (선택적 클라우드 TTS)
# ============================================
echo "[2/3] edge-tts Python 모듈 설치 중..."
if python3 -c "import edge_tts" &> /dev/null; then
    echo "✅ edge-tts 이미 설치됨"
else
    pip3 install --user edge-tts
    echo "✅ edge-tts 설치 완료"
fi

# edge-tts 테스트
echo "🔊 edge-tts 테스트 (인터넷 필요)..."
if edge-tts --voice ko-KR-SunHiNeural --text "엣지 티티에스 테스트" --write-media /tmp/test_edge.mp3 &> /dev/null; then
    echo "✅ edge-tts 테스트 성공"
    rm -f /tmp/test_edge.mp3
else
    echo "⚠️  edge-tts 테스트 실패 (인터넷 연결 확인)"
fi

echo ""

# ============================================
# 3. Piper 바이너리 설치
# ============================================
echo "[3/3] Piper TTS 바이너리 설치 중..."
if command -v piper &> /dev/null; then
    echo "✅ piper 이미 설치됨: $(piper --version 2>&1 | head -1)"
else
    echo "📦 Piper 사전 빌드 바이너리 다운로드 중..."

    # 임시 디렉토리 생성
    TMP_DIR=$(mktemp -d)
    cd "$TMP_DIR"

    # ARM64 바이너리 다운로드 (Jetson Orin Nano)
    wget -q --show-progress https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_arm64.tar.gz

    # 압축 해제
    tar -xzf piper_arm64.tar.gz

    # 바이너리 설치
    sudo cp piper/piper /usr/local/bin/
    sudo chmod +x /usr/local/bin/piper

    # 정리
    cd -
    rm -rf "$TMP_DIR"

    echo "✅ piper 설치 완료"
fi

# Piper 테스트 (기존 모델 사용)
echo "🔊 Piper 테스트 (한국어 음성 생성)..."
PIPER_MODEL="/home/sang/dev_ws/AI_secretary_robot/models/tts/neurlang_piper_onnx_kss_korean/piper-kss-korean.onnx"
PIPER_CONFIG="/home/sang/dev_ws/AI_secretary_robot/models/tts/neurlang_piper_onnx_kss_korean/piper-kss-korean.onnx.json"

if [ -f "$PIPER_MODEL" ]; then
    echo "안녕하세요, 파이퍼 티티에스입니다" | piper \
        --model "$PIPER_MODEL" \
        --config "$PIPER_CONFIG" \
        --output_file /tmp/test_piper.wav 2>/dev/null

    if [ -f /tmp/test_piper.wav ]; then
        echo "✅ Piper 음성 생성 성공: /tmp/test_piper.wav"
        aplay /tmp/test_piper.wav 2>/dev/null || echo "⚠️  ALSA 재생 실패 (정상: 헤드리스 환경)"
        rm -f /tmp/test_piper.wav
    else
        echo "❌ Piper 음성 생성 실패"
    fi
else
    echo "⚠️  Piper 모델 없음: $PIPER_MODEL"
fi

echo ""
echo "========================================="
echo "✅ TTS 의존성 설치 완료!"
echo "========================================="
echo ""
echo "설치된 TTS 엔진:"
echo "  1. espeak-ng: $(which espeak-ng)"
echo "  2. edge-tts:  $(python3 -c 'import edge_tts; print(edge_tts.__file__)' 2>/dev/null || echo 'NOT FOUND')"
echo "  3. piper:     $(which piper)"
echo ""
echo "다음 단계:"
echo "  cd /home/sang/dev_ws/AI_secretary_robot"
echo "  source install/setup.bash"
echo "  ros2 launch tts_cpp tts.launch.py"
echo ""
