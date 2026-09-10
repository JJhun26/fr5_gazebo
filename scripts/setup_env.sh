# 이 워크스페이스를 쓸 셸 환경. source로 부른다(실행하지 않는다).
#
#   source scripts/setup_env.sh
#
# docker/entrypoint.sh가 하던 일을 호스트에서 한다. 내용은 같다.
# 다른 점은 경로가 /ws에 박혀 있지 않고 저장소 위치를 따라간다는 것뿐이다.

# shellcheck shell=bash

# 저장소 뿌리. source로 불릴 때도 맞게 잡히도록 BASH_SOURCE를 쓴다.
BOX_CELL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export BOX_CELL_ROOT

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "!! /opt/ros/${ROS_DISTRO}/setup.bash가 없다. ./scripts/install_deps.sh 먼저."
fi

if [ -f "${BOX_CELL_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${BOX_CELL_ROOT}/install/setup.bash"
fi

# 메시와 라벨 텍스처를 gz가 찾을 수 있게. 설치 트리를 먼저 본다.
export GZ_SIM_RESOURCE_PATH="${BOX_CELL_ROOT}/install/box_cell_description/share:${BOX_CELL_ROOT}/install/box_cell_sim/share:${GZ_SIM_RESOURCE_PATH:-}"

# 산출물(적재 기록, MES 큐/DB, 트윈 JSON, 판독 실패 프레임)이 쌓이는 곳.
# 기본은 예전과 같은 /tmp/box_cell이다. 실물 셀에서는 /var/lib/box_cell처럼
# 재부팅을 넘기는 곳으로 옮긴다.
export BOX_CELL_DATA_DIR="${BOX_CELL_DATA_DIR:-/tmp/box_cell}"
mkdir -p "${BOX_CELL_DATA_DIR}"

# DDS. 컨테이너의 host 네트워크와 같은 자리다. 같은 PC 안에서만 돌릴 것이면
# ROS_LOCALHOST_ONLY=1이 시끄러운 이웃을 막아 준다. 실물 셀은 여러 대라 0이다.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# 라벨 텍스처가 cell.yaml보다 오래됐으면 다시 만든다.
#
# "없으면 만든다"로 두었더니, 라벨 치수를 고쳐도 옛 PNG가 그대로 쓰였다.
# QR 여백을 규격에 맞게 고쳐 놓고 이틀 전 라벨을 계속 렌더링하고 있었다.
box_cell_refresh_labels() {
  local png="${BOX_CELL_ROOT}/src/box_cell_sim/models/box/materials/textures/box_1.png"
  local yaml="${BOX_CELL_ROOT}/src/box_cell_description/config/cell.yaml"
  local mk="${BOX_CELL_ROOT}/tools/make_labels.py"
  [ -f "$mk" ] || { [ -f "$png" ] || echo "경고 : 라벨 텍스처도 make_labels.py도 없다. QR 판독이 실패한다."; return; }
  if [ ! -f "$png" ] || [ "$yaml" -nt "$png" ]; then
    echo "라벨 텍스처가 없거나 cell.yaml보다 오래됐다. 다시 만든다."
    python3 "$mk" || echo "경고 : 라벨 생성 실패. QR 판독이 실패한다."
  fi
}
