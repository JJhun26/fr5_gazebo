#!/usr/bin/env bash
# 시뮬레이션 실행. 인자는 그대로 컨테이너로 넘어간다.
#
#   ./docker/run.sh                                  전체 데모
#   ./docker/run.sh bash                             셸만
#   ./docker/run.sh ros2 launch box_cell_bringup demo.launch.py hardware:=mock
set -euo pipefail
cd "$(dirname "$0")/.."

GPU_ARGS=()
if docker info 2>/dev/null | grep -qi 'Runtimes:.*nvidia'; then
  GPU_ARGS=(--gpus all)
  echo "NVIDIA 런타임 감지. GPU로 돈다."
else
  echo "!! NVIDIA 런타임이 안 보인다. 소프트웨어 렌더링으로 돈다(아주 느리다)."
  echo "   nvidia-container-toolkit 설치 여부를 확인할 것. README 참고."
fi

# 헤드리스(DISPLAY가 없거나 headless:=true)에서는 Ogre2가 EGL로 떨어진다.
# 그때 컨테이너 안에 mesa와 nvidia EGL 벤더가 둘 다 있으면 mesa를 골라
# 소프트웨어로 렌더링한다. GPU 사용률이 0%인데 카메라가 초당 한 장도
# 안 나오는 상태가 그것이다. 벤더를 nvidia 하나로 못 박는다.

xhost +local:docker >/dev/null 2>&1 || true

exec docker run --rm -it \
  --shm-size=2g \
  "${GPU_ARGS[@]}" \
  --network host \
  --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)/src:/ws/src:rw" \
  -v "$(pwd)/tools:/ws/tools:ro" \
  -v box_cell_data:/tmp/box_cell \
  box_cell_sim:jazzy-harmonic "$@"
