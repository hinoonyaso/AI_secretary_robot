#!/bin/bash
# JetRover Complete System Startup Script
# Follows plan.md section 6.4 execution sequence

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

echo "========================================="
echo "JetRover System Startup"
echo "========================================="

# Step 1: Jetson power mode
echo "[1/7] Setting Jetson power mode (MAXN)..."
sudo nvpmodel -m 2 || echo "[WARN] nvpmodel failed (ignore if not on Jetson)"
nvpmodel -q || true

# Step 2: Fan control (optional)
echo "[2/7] Setting fan speed to max..."
FAN_PWM="/sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1"
if ls $FAN_PWM 1> /dev/null 2>&1; then
    echo 255 | sudo tee $FAN_PWM > /dev/null
    echo "[OK] Fan set to 100%"
else
    echo "[WARN] Fan control not available"
fi

# Step 3: ZRAM (optional)
echo "[3/7] Checking ZRAM..."
swapon -s | grep zram || echo "[WARN] ZRAM not configured"

# Step 4: Check models exist
echo "[4/7] Verifying model files..."
MODELS_DIR="$PROJECT_ROOT/models"
MISSING_MODELS=0

check_model() {
    if [ -f "$1" ]; then
        echo "[OK] $2"
    else
        echo "[WARN] Missing: $1 — service may fail to start"
        MISSING_MODELS=$((MISSING_MODELS + 1))
    fi
}

check_model "$MODELS_DIR/llm/qwen2.5-3b-instruct-q4_k_m.gguf" "LLM model"
# check_model "$MODELS_DIR/vlm/moondream2-q4_k_m.gguf" "VLM model"
# check_model "$MODELS_DIR/vision/yolo11n_fp16.engine" "YOLO engine"

if [ "$MISSING_MODELS" -gt 0 ]; then
    echo "[WARN] $MISSING_MODELS model(s) missing — continuing anyway"
fi

# Step 5: Start Docker containers
echo "[5/7] Starting Docker containers..."
docker compose up -d

# Wait for containers to start
sleep 10

# Step 6: Health check
echo "[6/7] Running health checks..."
docker compose ps

echo ""
echo "Checking Brain Core health..."
curl -f http://localhost:8080/health || echo "[WARN] Health check failed"

# Step 7: Verify ROS2 heartbeat
echo "[7/7] Verifying ROS2 heartbeat..."
timeout 5 ros2 topic echo /heartbeat --once || echo "[WARN] Heartbeat not detected"

echo ""
echo "========================================="
echo "System Startup Complete!"
echo "========================================="
echo ""
echo "Monitor logs: docker compose logs -f"
echo "Stop system: docker compose down"
echo "Health check: curl http://localhost:8080/health"
