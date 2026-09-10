#!/usr/bin/env bash
# 환경 점검. 데모 전에 한 번 돌린다.
#
#   ./scripts/doctor.sh
#
# 컨테이너를 벗어나면서 "이미지에 들어 있으니 당연히 있다"가 전부 사라졌다.
# 여기 있는 항목은 전부 한 번씩 실제로 데모를 세웠던 것들이다. 무엇이
# 빠졌는지 로그 300줄에서 찾는 대신 여기서 30초에 본다.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

PASS=0; WARN=0; FAIL=0
ok()   { printf '  \033[32m[OK]\033[0m   %s\n' "$*"; PASS=$((PASS+1)); }
warn() { printf '  \033[33m[주의]\033[0m %s\n' "$*"; WARN=$((WARN+1)); }
bad()  { printf '  \033[31m[실패]\033[0m %s\n' "$*"; FAIL=$((FAIL+1)); }
head_() { printf '\n\033[1m%s\033[0m\n' "$*"; }

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

head_ "1. 운영체제"
CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"
VER="$(. /etc/os-release && echo "$VERSION_ID")"
if [ "$CODENAME" = "noble" ]; then ok "Ubuntu $VER ($CODENAME). Jazzy 공식 조합이다."
else warn "Ubuntu $VER ($CODENAME). Jazzy 바이너리는 24.04용이다. docker/ 쪽을 쓸 것."; fi

head_ "2. ROS 2 / Gazebo"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  ok "/opt/ros/${ROS_DISTRO} 있음"
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  bad "/opt/ros/${ROS_DISTRO}가 없다. ./scripts/install_deps.sh"
fi
for p in ros_gz_sim ros_gz_bridge ros_gz_image gz_ros2_control controller_manager \
         joint_trajectory_controller moveit_ros_move_group moveit_configs_utils \
         pilz_industrial_motion_planner xacro robot_state_publisher cv_bridge rviz2; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then ok "$p"; else bad "$p 없음 (apt: ros-${ROS_DISTRO}-${p//_/-})"; fi
done
if command -v gz >/dev/null; then
  GZV="$(gz sim --versions 2>/dev/null | head -1)"
  case "$GZV" in
    8.*) ok "Gazebo Harmonic (gz-sim $GZV)" ;;
    "")  warn "gz는 있는데 버전을 못 읽었다." ;;
    *)   warn "gz-sim $GZV. Jazzy의 짝은 Harmonic(gz-sim 8)이다." ;;
  esac
else
  bad "gz 명령이 없다 (apt: ros-${ROS_DISTRO}-ros-gz)"
fi

head_ "3. 파이썬"
for m in cv2 numpy yaml PIL qrcode pyzbar fastapi uvicorn; do
  if python3 -c "import $m" >/dev/null 2>&1; then ok "$m"
  elif [ "$m" = "fastapi" ] || [ "$m" = "uvicorn" ]; then
    warn "$m 없음. MES가 표준 라이브러리 서버로 떨어진다(대시보드는 뜬다)."
  else bad "$m 없음. ./scripts/install_deps.sh"; fi
done

head_ "4. 워크스페이스"
if [ -f install/setup.bash ]; then ok "빌드됨 (install/)"; else bad "아직 빌드 안 됨. ./scripts/build.sh"; fi
PNG="src/box_cell_sim/models/box/materials/textures/box_1.png"
YAML="src/box_cell_description/config/cell.yaml"
if [ ! -f "$PNG" ]; then bad "라벨 텍스처가 없다. python3 tools/make_labels.py"
elif [ "$YAML" -nt "$PNG" ]; then warn "라벨이 cell.yaml보다 오래됐다. run_demo.sh가 다시 만든다."
else ok "라벨 텍스처가 cell.yaml보다 새것"; fi
DATA="${BOX_CELL_DATA_DIR:-/tmp/box_cell}"
if mkdir -p "$DATA" 2>/dev/null && [ -w "$DATA" ]; then ok "산출물 디렉터리 $DATA"
else bad "$DATA에 쓸 수 없다. BOX_CELL_DATA_DIR로 옮길 것."; fi

head_ "5. 렌더링"
if command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
  ok "NVIDIA 드라이버 : $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
else
  warn "NVIDIA GPU가 안 보인다. 소프트웨어 렌더링이면 카메라 4대가 못 따라온다."
fi
if command -v glxinfo >/dev/null && [ -n "${DISPLAY:-}" ]; then
  R="$(glxinfo -B 2>/dev/null | grep -i 'OpenGL renderer' | cut -d: -f2- | xargs)"
  case "$R" in
    *llvmpipe*|*softpipe*) bad "GLX 렌더러가 $R. 소프트웨어다. 드라이버를 볼 것." ;;
    "") warn "glxinfo가 렌더러를 못 냈다." ;;
    *) ok "GLX 렌더러 : $R" ;;
  esac
elif [ -z "${DISPLAY:-}" ]; then
  warn "DISPLAY가 없다. headless:=true로 돌릴 것."
else
  warn "glxinfo가 없다 (apt: mesa-utils)."
fi
[ "${XDG_SESSION_TYPE:-}" = "wayland" ] && warn "Wayland 세션이다. GUI가 Xwayland를 거친다. 느리면 Xorg 세션으로 로그인할 것."

head_ "6. 커널 쪽"
SHM="$(df -B1 --output=size /dev/shm 2>/dev/null | tail -1)"
if [ -n "$SHM" ] && [ "$SHM" -ge 2000000000 ]; then ok "/dev/shm $((SHM/1024/1024)) MB"
else warn "/dev/shm이 작다($((${SHM:-0}/1024/1024)) MB). Gazebo가 죽을 수 있다."; fi
RMEM="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
if [ "$RMEM" -ge 16777216 ]; then ok "net.core.rmem_max $((RMEM/1024/1024)) MB"
else warn "net.core.rmem_max가 $((RMEM/1024/1024)) MB다. 원본 영상 토픽이 유실된다. install_deps.sh가 올린다."; fi

printf '\n\033[1m정리\033[0m  OK %d, 주의 %d, 실패 %d\n' "$PASS" "$WARN" "$FAIL"
[ "$FAIL" -eq 0 ] || echo "실패 항목을 먼저 해결할 것."
exit $(( FAIL > 0 ? 1 : 0 ))
