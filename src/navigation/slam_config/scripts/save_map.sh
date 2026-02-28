#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SLAM_CONFIG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAVIGATION_DIR="$(cd "${SLAM_CONFIG_DIR}/.." && pwd)"

MAP_DIR="${SLAM_CONFIG_DIR}/maps"
NAV2_MAP_DIR="${NAVIGATION_DIR}/nav2_config/maps"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
SLAM_MAP_BASE="${MAP_DIR}/map_${TIMESTAMP}"
NAV2_MAP_BASE="${NAV2_MAP_DIR}/map_${TIMESTAMP}"

mkdir -p "${MAP_DIR}"
mkdir -p "${NAV2_MAP_DIR}"

echo "Saving SLAM posegraph: ${SLAM_MAP_BASE}.posegraph"

ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${SLAM_MAP_BASE}'}"

echo "Saving Nav2 occupancy map: ${NAV2_MAP_BASE}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -f "${NAV2_MAP_BASE}"

ln -sfn "${SLAM_MAP_BASE}.posegraph" "${MAP_DIR}/saved_map.posegraph"
ln -sfn "${SLAM_MAP_BASE}.data" "${MAP_DIR}/saved_map.data"
ln -sfn "${NAV2_MAP_BASE}.yaml" "${NAV2_MAP_DIR}/default_map.yaml"
ln -sfn "${NAV2_MAP_BASE}.pgm" "${NAV2_MAP_DIR}/default_map.pgm"

echo "Map saved and latest symlinks updated:"
echo "  - SLAM : ${MAP_DIR}/saved_map.posegraph"
echo "  - Nav2 : ${NAV2_MAP_DIR}/default_map.yaml"
