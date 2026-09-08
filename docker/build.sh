#!/usr/bin/env bash
# 이미지 빌드. 처음 한 번은 20~40분 걸린다(ros-jazzy-desktop + moveit).
set -euo pipefail
cd "$(dirname "$0")/.."
docker build -f docker/Dockerfile -t box_cell_sim:jazzy-harmonic .
echo
echo "완료. 실행 :"
echo "  xhost +local:docker"
echo "  ./docker/run.sh"
