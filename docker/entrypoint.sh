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

# 라벨 텍스처가 없으면 만든다. 이미지 빌드 때 넣어 두지만, 소스를 마운트해
# 개발할 때는 비어 있을 수 있다.
if [ ! -f /ws/src/box_cell_sim/models/box/materials/textures/box_1.png ]; then
  if [ -f /ws/tools/make_labels.py ]; then
    echo "라벨 텍스처가 없다. 만든다."
    python3 /ws/tools/make_labels.py
  else
    echo "경고 : 라벨 텍스처도 tools/make_labels.py도 없다. QR 판독이 실패한다."
  fi
fi

exec "$@"
