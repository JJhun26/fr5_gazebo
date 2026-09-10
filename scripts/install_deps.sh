#!/usr/bin/env bash
# =============================================================================
# Ubuntu 24.04 (noble)에 이 셀을 바로 깐다. Docker 없이.
#
#   ./scripts/install_deps.sh
#
# 왜 컨테이너를 벗어나는가.
#   컨테이너는 개발 PC가 Ubuntu 26.04라서 필요했다. Jazzy 바이너리가 26.04에
#   없기 때문이다. 대상이 24.04면 그 이유가 사라진다. Jazzy도 Gazebo
#   Harmonic도 공식 바이너리가 있고, 무엇보다 GPU가 그냥 된다.
#   컨테이너에서 실시간 계수를 0.004에서 1.00으로 끌어올리느라 했던 일
#   (NVIDIA EGL ICD 주입, __GLX_VENDOR_LIBRARY_NAME 못박기)은 호스트에서는
#   할 일이 아니다. 드라이버가 이미 제자리에 있다.
#
# 무엇을 까는가.
#   ROS 2 Jazzy(desktop) + Gazebo Harmonic(ros-gz) + MoveIt 2 + ros2_control,
#   그리고 판독/라벨/MES가 쓰는 파이썬 몇 개.
#   Dockerfile의 apt 목록과 같은 것을 깐다. 두 벌이 갈리지 않게 여기 한 곳만
#   고치고 Dockerfile 쪽은 따라오게 한다.
#
# 한 번만 돌리면 된다. 다시 돌려도 안전하다(전부 멱등).
# =============================================================================
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME}")"
SUDO=""
[ "$(id -u)" -eq 0 ] || SUDO="sudo"

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

# --- 0. 여기가 맞는 자리인가 ---------------------------------------------
if [ "$CODENAME" != "noble" ]; then
  echo "!! 이 스크립트는 Ubuntu 24.04(noble) 기준이다. 지금은 '$CODENAME'이다."
  echo "   Jazzy 공식 바이너리는 24.04용뿐이다. 다른 버전이면 docker/ 쪽을 쓸 것."
  read -r -p "   그래도 계속하겠는가? [y/N] " ans
  [ "${ans:-N}" = "y" ] || exit 1
fi

# --- 1. 저장소 ------------------------------------------------------------
say "apt 저장소 준비"
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  software-properties-common curl gnupg2 lsb-release ca-certificates
$SUDO add-apt-repository -y universe

if [ ! -f /etc/apt/sources.list.d/ros2.list ] && \
   [ ! -f /etc/apt/sources.list.d/ros2-latest.list ] && \
   ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  say "ROS 2 저장소 등록"
  # 요즘 권장하는 방식은 키와 목록을 담은 deb 한 장이다. 키 갱신이 apt로
  # 따라온다. 그 deb을 못 받으면 예전 방식(키링 + sources.list)으로 떨어진다.
  VER="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}' || true)"
  DEB="/tmp/ros2-apt-source.deb"
  if [ -n "$VER" ] && curl -fsSL -o "$DEB" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${CODENAME}_all.deb"; then
    $SUDO apt-get install -y "$DEB"
  else
    echo "   ros2-apt-source deb을 못 받았다. 키링을 직접 넣는다."
    $SUDO curl -fsSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
      | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi
  $SUDO apt-get update
fi

# --- 2. ROS 2 + Gazebo Harmonic + MoveIt ---------------------------------
# ros-jazzy-ros-gz가 Harmonic을 끌어온다. Jazzy와 Harmonic이 공식 조합이다.
say "ROS 2 ${ROS_DISTRO} + Gazebo Harmonic + MoveIt 설치 (20~40분)"
$SUDO apt-get install -y --no-install-recommends \
  build-essential git python3-pip python3-colcon-common-extensions \
  python3-rosdep python3-vcstool fonts-nanum mesa-utils libgl1-mesa-dri x11-apps \
  "ros-${ROS_DISTRO}-desktop" \
  "ros-${ROS_DISTRO}-ros-gz" \
  "ros-${ROS_DISTRO}-ros-gz-sim" \
  "ros-${ROS_DISTRO}-ros-gz-bridge" \
  "ros-${ROS_DISTRO}-ros-gz-image" \
  "ros-${ROS_DISTRO}-ros-gz-interfaces" \
  "ros-${ROS_DISTRO}-gz-ros2-control" \
  "ros-${ROS_DISTRO}-ros2-control" \
  "ros-${ROS_DISTRO}-ros2-controllers" \
  "ros-${ROS_DISTRO}-joint-trajectory-controller" \
  "ros-${ROS_DISTRO}-joint-state-broadcaster" \
  "ros-${ROS_DISTRO}-moveit" \
  "ros-${ROS_DISTRO}-moveit-planners-ompl" \
  "ros-${ROS_DISTRO}-pilz-industrial-motion-planner" \
  "ros-${ROS_DISTRO}-pick-ik" \
  "ros-${ROS_DISTRO}-moveit-configs-utils" \
  "ros-${ROS_DISTRO}-moveit-simple-controller-manager" \
  "ros-${ROS_DISTRO}-xacro" \
  "ros-${ROS_DISTRO}-robot-state-publisher" \
  "ros-${ROS_DISTRO}-cv-bridge" \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-tf2-ros" \
  "ros-${ROS_DISTRO}-rviz2"

# --- 3. 파이썬 -------------------------------------------------------------
# QR 판독(pyzbar), 라벨 생성(qrcode/pillow), MES 서버(fastapi/uvicorn)가 쓴다.
# apt에 있으면 apt로 넣는다. 배포판 파이썬을 pip로 헤집는 것은 마지막 수단이다.
# 없는 것만 --user로 넣어 ~/.local에 둔다. /usr는 건드리지 않는다.
say "파이썬 의존성"
$SUDO apt-get install -y --no-install-recommends libzbar0 || true

ensure_py() {   # ensure_py <import 이름> <apt 패키지|-> <pip 패키지>
  local mod="$1" aptpkg="$2" pippkg="$3"
  if python3 -c "import $mod" >/dev/null 2>&1; then
    echo "   $mod 있음"
    return
  fi
  if [ "$aptpkg" != "-" ] && $SUDO apt-get install -y --no-install-recommends "$aptpkg" >/dev/null 2>&1 \
     && python3 -c "import $mod" >/dev/null 2>&1; then
    echo "   $mod <- apt $aptpkg"
    return
  fi
  echo "   $mod <- pip --user $pippkg"
  pip3 install --no-cache-dir --user --break-system-packages "$pippkg"
}

ensure_py cv2     python3-opencv  opencv-python-headless
ensure_py numpy   python3-numpy   numpy
ensure_py yaml    python3-yaml    pyyaml
ensure_py PIL     python3-pil     pillow
ensure_py qrcode  python3-qrcode  qrcode
ensure_py fastapi python3-fastapi fastapi
ensure_py uvicorn python3-uvicorn "uvicorn[standard]"
ensure_py pyzbar  -               pyzbar
ensure_py barcode -               python-barcode

# --- 4. rosdep ------------------------------------------------------------
say "rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || $SUDO rosdep init
rosdep update --rosdistro "${ROS_DISTRO}" || \
  echo "!! rosdep update 실패. 네트워크를 볼 것. 빌드는 그래도 될 수 있다."

# --- 5. 커널 쪽 한 가지 ---------------------------------------------------
# 소켓 수신 버퍼가 기본 4 MB면 큰 영상이 통째로 유실된다. 지금 파이프라인은
# 압축 토픽을 받도록 되어 있어 이게 없어도 돌지만, 원본 영상을 한 번이라도
# 구독하면(rviz에서 image_raw를 켜는 것만으로도) 바로 티가 난다.
CUR="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
if [ "$CUR" -lt 16777216 ]; then
  say "소켓 수신 버퍼를 16 MB로 (지금 ${CUR})"
  echo 'net.core.rmem_max=16777216' | $SUDO tee /etc/sysctl.d/60-box-cell.conf >/dev/null
  $SUDO sysctl -p /etc/sysctl.d/60-box-cell.conf >/dev/null
fi

cat <<'EOS'

설치 끝.

다음 :
  ./scripts/build.sh        워크스페이스 빌드 (처음 5~10분)
  ./scripts/doctor.sh       환경 점검. 무엇이 빠졌는지 한눈에 본다
  ./scripts/run_demo.sh     전체 데모

GPU는 따로 할 일이 없다. 컨테이너에서 필요했던 EGL ICD 주입과
__GLX_VENDOR_LIBRARY_NAME은 호스트 드라이버가 이미 하는 일이다.
확인만 한다 :  glxinfo -B | grep "OpenGL renderer"
EOS
