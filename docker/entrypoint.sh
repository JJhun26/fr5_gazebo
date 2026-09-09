#!/usr/bin/env bash
# 컨테이너 진입점. 환경을 깔고 명령을 넘긴다.
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

# 메시와 라벨 텍스처를 gz가 찾을 수 있게.
export GZ_SIM_RESOURCE_PATH="/ws/install/box_cell_description/share:/ws/install/box_cell_sim/share:${GZ_SIM_RESOURCE_PATH}"

# DDS는 host 네트워크에서 그대로 쓴다. 브리지 네트워크를 넘으면 디스커버리가
# 깨지고 지연이 생긴다(기획서 4절).
export ROS_LOCALHOST_ONLY=0

# 라벨 텍스처를 cell.yaml보다 오래됐으면 다시 만든다.
#
# "없으면 만든다"로 두었더니, 라벨 치수를 고쳐도 옛 PNG가 그대로 쓰였다.
# QR 여백을 규격에 맞게 고쳐 놓고 이틀 전 라벨을 계속 렌더링하고 있었다.
# 이미지 빌드에서도 다시 만들지만, run.sh는 src를 마운트해 덮어쓰므로
# 여기서 한 번 더 본다.
LABEL_PNG=/ws/src/box_cell_sim/models/box/materials/textures/box_1.png
CELL_YAML=/ws/src/box_cell_description/config/cell.yaml
if [ -f /ws/tools/make_labels.py ]; then
  if [ ! -f "$LABEL_PNG" ] || [ "$CELL_YAML" -nt "$LABEL_PNG" ]; then
    echo "라벨 텍스처가 없거나 cell.yaml보다 오래됐다. 다시 만든다."
    python3 /ws/tools/make_labels.py || echo "경고 : 라벨 생성 실패. QR 판독이 실패한다."
  fi
elif [ ! -f "$LABEL_PNG" ]; then
  echo "경고 : 라벨 텍스처도 tools/make_labels.py도 없다. QR 판독이 실패한다."
fi

exec "$@"
