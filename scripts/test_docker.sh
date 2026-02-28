#!/bin/bash
# JetRover Docker Image Test Script
# Runs all tests and reports total pass/fail count at the end.

IMAGE_NAME="jetrover/brain:latest"
PASS=0
FAIL=0

run_test() {
    local test_num="$1"
    local test_name="$2"
    shift 2

    if "$@" > /dev/null 2>&1; then
        echo "[OK] [$test_num] $test_name"
        PASS=$((PASS + 1))
    else
        echo "[FAIL] [$test_num] $test_name"
        FAIL=$((FAIL + 1))
    fi
}

echo "Testing Docker image: $IMAGE_NAME"
echo ""

# Test 1: Image exists
run_test "1/8" "Image exists" \
    docker image inspect "$IMAGE_NAME"

# Test 2: ROS2 availability
run_test "2/8" "ROS2" \
    docker run --rm "$IMAGE_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 --version"

# Test 3: ONNX Runtime GPU
run_test "3/8" "ONNX Runtime GPU" \
    docker run --rm --runtime=nvidia "$IMAGE_NAME" \
    python3 -c "import onnxruntime as ort; assert ort.get_device()=='GPU'"

# Test 4: llama.cpp
run_test "4/8" "llama.cpp" \
    docker run --rm "$IMAGE_NAME" llama-cli --version

# Test 5: Piper TTS
run_test "5/8" "Piper TTS" \
    docker run --rm "$IMAGE_NAME" piper --version

# Test 6: Database schema
run_test "6/8" "Database schema" \
    docker run --rm "$IMAGE_NAME" bash -c "test -f /opt/rover/db/init_schema.sql"

# Test 7: YOLO node binary
run_test "7/8" "YOLO node" \
    docker run --rm "$IMAGE_NAME" bash -c "test -f /usr/local/bin/yolo_ros_node"

# Test 8: Model mount points (image is intentionally model-free)
run_test "8/8" "Model mount points" \
    docker run --rm "$IMAGE_NAME" bash -c "test -d /opt/rover/models/llm && test -d /opt/rover/models/vlm"

echo ""
echo "========================================="
echo "Results: $PASS passed, $FAIL failed (total $((PASS + FAIL)))"
echo "========================================="

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
