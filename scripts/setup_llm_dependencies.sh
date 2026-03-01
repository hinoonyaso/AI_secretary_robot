#!/bin/bash
# LLM 의존성 자동 설치 스크립트
# 작성일: 2026-03-01
# 용도: Ollama, llama.cpp, Qwen2.5 모델 자동 설치

set -e  # 에러 발생 시 즉시 종료

echo "========================================="
echo "LLM 의존성 설치 스크립트"
echo "========================================="
echo ""

# ============================================
# 1. Ollama 설치 (즉시 fallback 복구)
# ============================================
echo "[1/3] Ollama 설치 중..."
if command -v ollama &> /dev/null; then
    echo "✅ Ollama 이미 설치됨: $(ollama --version)"
else
    echo "📦 Ollama 공식 설치 스크립트 실행 중..."
    curl -fsSL https://ollama.com/install.sh | sh
    echo "✅ Ollama 설치 완료"
fi

# Ollama 서비스 시작
echo "🚀 Ollama 서비스 시작 중..."
sudo systemctl start ollama || echo "⚠️  systemd 서비스 시작 실패, 수동 실행 필요"
sudo systemctl enable ollama || true

# Ollama 준비 대기
echo "⏳ Ollama 서버 준비 대기 중..."
for i in {1..10}; do
    if curl -s http://localhost:11434/api/tags &> /dev/null; then
        echo "✅ Ollama 서버 준비 완료"
        break
    fi
    sleep 1
done

# Qwen2.5-1.5B 모델 다운로드
echo "📥 Qwen2.5:1.5b 모델 다운로드 중 (약 1.2GB, 수 분 소요)..."
if ollama list | grep -q "qwen2.5:1.5b"; then
    echo "✅ Qwen2.5:1.5b 이미 다운로드됨"
else
    ollama pull qwen2.5:1.5b
    echo "✅ Qwen2.5:1.5b 다운로드 완료"
fi

# Ollama 테스트
echo "🧪 Ollama 추론 테스트..."
RESPONSE=$(ollama run qwen2.5:1.5b "안녕" --verbose 2>&1 | head -20)
if echo "$RESPONSE" | grep -q "안녕"; then
    echo "✅ Ollama 추론 테스트 성공"
else
    echo "⚠️  Ollama 응답: $RESPONSE"
fi

echo ""

# ============================================
# 2. llama.cpp 빌드 (Planning 문서 준수)
# ============================================
echo "[2/3] llama.cpp 빌드 중..."

LLAMA_DIR="/home/sang/dev_ws/AI_secretary_robot/external/llama.cpp"

if [ -f "$LLAMA_DIR/build/bin/llama-server" ]; then
    echo "✅ llama-server 이미 빌드됨: $LLAMA_DIR/build/bin/llama-server"
else
    # 의존성 설치
    echo "📦 빌드 의존성 설치 중..."
    sudo apt update
    sudo apt install -y build-essential cmake git

    # 소스 클론
    if [ ! -d "$LLAMA_DIR" ]; then
        echo "📥 llama.cpp 소스 클론 중..."
        mkdir -p /home/sang/dev_ws/AI_secretary_robot/external
        cd /home/sang/dev_ws/AI_secretary_robot/external
        git clone https://github.com/ggerganov/llama.cpp.git
    fi

    cd "$LLAMA_DIR"

    # CUDA 지원 빌드 (Jetson Orin Nano)
    echo "🔨 llama.cpp 빌드 중 (CUDA 지원, 약 30분 소요)..."
    mkdir -p build
    cd build

    cmake .. \
        -DGGML_CUDA=ON \
        -DCMAKE_CUDA_ARCHITECTURES=87 \
        -DBUILD_SHARED_LIBS=ON

    cmake --build . --config Release -j$(nproc)

    echo "✅ llama.cpp 빌드 완료"
fi

# 심볼릭 링크 생성
if [ -f "$LLAMA_DIR/build/bin/llama-server" ]; then
    sudo ln -sf "$LLAMA_DIR/build/bin/llama-server" /usr/local/bin/llama-server
    echo "✅ llama-server 심볼릭 링크 생성: /usr/local/bin/llama-server"
fi

echo ""

# ============================================
# 3. Qwen2.5-1.5B GGUF 모델 다운로드
# ============================================
echo "[3/3] Qwen2.5-1.5B GGUF 모델 다운로드 중..."

MODEL_DIR="/home/sang/dev_ws/AI_secretary_robot/models/llm"
MODEL_FILE="$MODEL_DIR/qwen2.5-1.5b-instruct-q4_k_m.gguf"

mkdir -p "$MODEL_DIR"

if [ -f "$MODEL_FILE" ]; then
    echo "✅ GGUF 모델 이미 다운로드됨: $MODEL_FILE"
else
    echo "📥 GGUF 모델 다운로드 중 (약 900MB, 수 분 소요)..."
    cd "$MODEL_DIR"

    wget -q --show-progress \
        https://huggingface.co/Qwen/Qwen2.5-1.5B-Instruct-GGUF/resolve/main/qwen2.5-1.5b-instruct-q4_k_m.gguf

    echo "✅ GGUF 모델 다운로드 완료"
fi

# llama-server 테스트
echo "🧪 llama-server 테스트..."
if [ -f "/usr/local/bin/llama-server" ] && [ -f "$MODEL_FILE" ]; then
    echo "🚀 llama-server 백그라운드 실행 중..."
    llama-server \
        --model "$MODEL_FILE" \
        --port 8081 \
        -ngl 99 \
        -c 2048 \
        -t 4 \
        &> /tmp/llama_server_test.log &

    LLAMA_PID=$!

    # 서버 준비 대기
    echo "⏳ llama-server 준비 대기 중..."
    for i in {1..20}; do
        if curl -s http://localhost:8081/health &> /dev/null; then
            echo "✅ llama-server 준비 완료"
            break
        fi
        sleep 1
    done

    # 추론 테스트
    echo "🧪 llama-server 추론 테스트..."
    RESPONSE=$(curl -s http://localhost:8081/v1/chat/completions \
        -H "Content-Type: application/json" \
        -d '{
            "messages": [{"role": "user", "content": "안녕하세요"}],
            "temperature": 0.7,
            "max_tokens": 50
        }' 2>&1)

    if echo "$RESPONSE" | grep -q "choices"; then
        echo "✅ llama-server 추론 테스트 성공"
    else
        echo "⚠️  llama-server 응답: $RESPONSE"
    fi

    # 서버 종료
    kill $LLAMA_PID 2>/dev/null || true
    echo "🛑 llama-server 테스트 종료"
else
    echo "⚠️  llama-server 또는 모델 파일 없음, 테스트 스킵"
fi

echo ""
echo "========================================="
echo "✅ LLM 의존성 설치 완료!"
echo "========================================="
echo ""
echo "설치된 LLM 엔진:"
echo "  1. Ollama:        $(which ollama)"
echo "  2. llama-server:  $(which llama-server 2>/dev/null || echo 'NOT FOUND')"
echo ""
echo "모델 파일:"
echo "  - Ollama:   $(ollama list | grep qwen2.5 || echo 'NOT FOUND')"
echo "  - GGUF:     $MODEL_FILE"
echo ""
echo "다음 단계:"
echo "  # Ollama 모드 (즉시 사용 가능)"
echo "  ros2 launch llm_cpp llm.launch.py"
echo ""
echo "  # llama.cpp 모드 (params.yaml 수정 필요)"
echo "  # src/ai/llm_cpp/config/params.yaml 에서 llama_server_binary 경로 설정"
echo ""
