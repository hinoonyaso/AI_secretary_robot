#!/bin/bash
# JetRover Docker Image Build Script
# Usage: ./scripts/build_docker.sh [--no-cache] [--stage STAGE]

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

IMAGE_NAME="jetrover/brain"
TAG="latest"
DOCKERFILE="docker/Dockerfile"

# Parse arguments
BUILD_ARGS=""
TARGET_STAGE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-cache)
            BUILD_ARGS="$BUILD_ARGS --no-cache"
            shift
            ;;
        --stage)
            if [[ -z "${2:-}" ]]; then
                echo "Missing value for --stage"
                exit 1
            fi
            TARGET_STAGE="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

if [ -n "$TARGET_STAGE" ]; then
    BUILD_ARGS="$BUILD_ARGS --target $TARGET_STAGE"
fi

echo "========================================="
echo "Building JetRover Docker Image"
echo "========================================="
echo "Image: $IMAGE_NAME:$TAG"
echo "Dockerfile: $DOCKERFILE"
echo "Target stage: ${TARGET_STAGE:-full-runtime (final)}"
echo "Build args: $BUILD_ARGS"
echo ""

# Build
# shellcheck disable=SC2086
docker build \
    -t "$IMAGE_NAME:$TAG" \
    -f "$DOCKERFILE" \
    $BUILD_ARGS \
    .

echo ""
echo "========================================="
echo "Build Complete!"
echo "========================================="
docker images "$IMAGE_NAME:$TAG"

echo ""
echo "Run with: docker run --runtime=nvidia -it $IMAGE_NAME:$TAG"
echo "Or use docker-compose: docker compose up -d"
