#!/usr/bin/env bash
# 데모 실행. docker/run.sh의 호스트판이다. 인자는 그대로 launch로 넘어간다.
#
#   ./scripts/run_demo.sh                                   전체 데모(GUI)
#   ./scripts/run_demo.sh headless:=true                    화면 없이
#   ./scripts/run_demo.sh hardware:=mock                    Gazebo 없이 MoveIt까지
#   ./scripts/run_demo.sh rviz:=true autostart:=false       사람이 시작을 잡는다
#   ./scripts/run_demo.sh hardware:=real robot_ip:=192.168.58.2
#
# GPU에 대해 : 호스트에서는 할 일이 없다. 컨테이너에서 필요했던 EGL ICD
# 주입과 GLX 벤더 못박기는 드라이버가 제자리에 있으면 생기지 않는 문제다.
# 확인은 ./scripts/doctor.sh가 한다.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# shellcheck disable=SC1091
source "$ROOT/scripts/setup_env.sh"

if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "!! 워크스페이스가 아직 안 빌드됐다. ./scripts/build.sh 먼저."
  exit 1
fi

box_cell_refresh_labels

exec ros2 launch box_cell_bringup demo.launch.py "$@"
