#!/usr/bin/env bash
# 워크스페이스 빌드. Docker 없이 호스트에서.
#
#   ./scripts/build.sh              전부
#   ./scripts/build.sh box_cell_logic box_cell_motion   지정한 패키지만
#
# 처음 한 번은 5~10분, 그다음은 고친 패키지만 다시 된다.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# shellcheck disable=SC1091
source "$ROOT/scripts/setup_env.sh"

# 라벨 텍스처와 MES 시드. 빌드가 반드시 다시 만든다.
# 치수를 바꾼 사람은 바뀌었다고 믿는데 그림은 안 바뀌는 어긋남을 막는다.
if [ -f tools/make_labels.py ]; then
  echo "== 라벨 텍스처 생성"
  python3 tools/make_labels.py || echo "!! 라벨 생성 실패. qrcode/pillow를 볼 것."
fi

# rosdep은 안전망이다. 필요한 것은 install_deps.sh가 이미 명시적으로 깔았고,
# fastapi/uvicorn/qrcode/pyzbar는 apt에 없거나 pip로 들어갔다. 그래서 여기서
# 무언가 해결하지 못해도 빌드를 세우지 않는다. 다만 무엇을 못 찾았는지는 남긴다.
if command -v rosdep >/dev/null; then
  echo "== rosdep"
  rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "fastapi uvicorn qrcode pyzbar python-barcode" \
    || echo "!! rosdep이 일부 키를 못 풀었다. install_deps.sh 목록으로 충분한지 확인할 것."
fi

echo "== colcon build"
ARGS=()
[ $# -gt 0 ] && ARGS=(--packages-select "$@")
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "${ARGS[@]}"

cat <<EOS

빌드 끝. 새 셸에서는 이것부터 :
  source $ROOT/scripts/setup_env.sh

실행 :
  ./scripts/run_demo.sh
EOS
