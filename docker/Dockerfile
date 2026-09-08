# =============================================================================
# 물류 로봇 데모 시뮬레이션 : ROS 2 Jazzy + Gazebo Harmonic
#
# 왜 컨테이너인가.
#   이 개발 PC는 Ubuntu 26.04다. ROS 2 Jazzy의 공식 바이너리는 24.04용이고,
#   26.04에는 아직 없다. 소스 빌드로 얹을 수도 있지만, 데모 장비와 개발 PC의
#   버전이 갈리기 시작하면 "여기서는 되는데"가 반드시 생긴다.
#   기획서 4절도 엣지를 Docker로 배포한다고 못박고 있다. 같은 방식으로 간다.
#
# 왜 Gazebo Classic이 아닌가.
#   FAIRINO 공식 fairino_gazebo는 Gazebo Classic + ROS 2 Humble이다.
#   Classic은 2025년 1월로 지원이 끝났고 Jazzy용 패키지가 없다. 그래서
#   URDF와 메시만 가져오고 ros2_control 계층은 gz_ros2_control로 옮겼다.
#   기구학, 관성, 관절 한계는 공식 값 그대로다.
# =============================================================================
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    GZ_VERSION=harmonic

# --- 시스템 -----------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        mesa-utils \
        libgl1-mesa-dri \
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

# --- ROS 2 + Gazebo Harmonic + MoveIt ---------------------------------------
# ros-jazzy-ros-gz가 Harmonic을 끌어온다. Jazzy와 Harmonic이 공식 조합이다.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-image \
        ros-jazzy-ros-gz-interfaces \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-joint-trajectory-controller \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-moveit \
        ros-jazzy-moveit-planners-ompl \
        ros-jazzy-pilz-industrial-motion-planner \
        ros-jazzy-moveit-configs-utils \
        ros-jazzy-moveit-simple-controller-manager \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-tf2-ros \
        ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# --- 파이썬 쪽 --------------------------------------------------------------
# QR 판독과 라벨 생성, MES 서버가 쓴다.
# --break-system-packages : 컨테이너 안이고 이 파이썬은 이 프로젝트 전용이다.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-numpy \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --break-system-packages \
        qrcode pillow fastapi "uvicorn[standard]" pyzbar

# --- 렌더링 -----------------------------------------------------------------
# Gazebo의 Sensors 시스템이 ogre2로 카메라를 돌린다.
#
# NVIDIA EGL ICD를 여기서 직접 만든다. nvidia-container-toolkit은 드라이버
# 라이브러리(libEGL_nvidia.so)는 넣어 주지만 이 등록 파일은 넣어 주지 않는
# 경우가 있다. 그러면 libglvnd가 mesa만 발견해 소프트웨어 EGL로 떨어지고,
# GPU 사용률 0%에 실시간 계수 0.004가 나온다(실측). 파일 내용은 드라이버
# 버전과 무관하므로 이미지에 넣어 두는 편이 안전하다.
RUN mkdir -p /usr/share/glvnd/egl_vendor.d \
    && printf '%s\n' \
       '{' \
       '    "file_format_version" : "1.0.0",' \
       '    "ICD" : {' \
       '        "library_path" : "libEGL_nvidia.so.0"' \
       '    }' \
       '}' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# GLX 벤더를 NVIDIA로 못박는다.
#
# 호스트가 Wayland 세션이면 컨테이너의 GLX는 Xwayland를 거치는데, 그때
# libglvnd가 벤더를 자동으로 못 고르고 mesa(llvmpipe)로 떨어진다.
# libGLX_nvidia.so는 멀쩡히 들어와 있는데도 그렇다. 그 상태로 GUI를 띄우면
# 렌더링이 통째로 CPU로 가고 GPU 사용률이 0 %로 나온다(실측).
#
# EGL(헤드리스 센서 렌더링)은 위의 10_nvidia.json이, GLX(창 모드)는
# 이 환경 변수가 맡는다. 둘 다 있어야 두 경로 모두 GPU로 간다.
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    QT_X11_NO_MITSHM=1

# --- 워크스페이스 -----------------------------------------------------------
WORKDIR /ws
COPY src /ws/src

# rosdep은 안전망이다. 필요한 것은 위 apt 블록에서 이미 명시적으로 깔았고,
# fastapi/uvicorn/qrcode는 apt에 없어 pip로 넣었다. 그래서 여기서 무언가
# 해결하지 못해도 빌드를 세우지 않는다. 다만 무엇을 못 찾았는지는 남긴다.
RUN source /opt/ros/jazzy/setup.bash \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -r -y \
         --skip-keys "fastapi uvicorn qrcode pyzbar" \
       || echo "rosdep이 일부 키를 못 풀었다. 위 apt/pip 목록으로 충분한지 확인할 것."

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "box_cell_bringup", "demo.launch.py"]
