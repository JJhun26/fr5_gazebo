#!/usr/bin/env bash
# xacro를 펼쳐 본다. 문법과 치수 계산을 빌드 전에 잡는다.
#
#   tools/xacro_expand.sh src/box_cell_description/urdf/box_cell_robot.urdf.xacro -o /tmp/robot.urdf
#   tools/xacro_expand.sh src/box_cell_description/urdf/box_cell_robot.urdf.xacro hardware:=real
#
# 호스트에 xacro가 깔려 있으면 그것을 쓴다(호스트 설치가 기본이 된 뒤로 이쪽이
# 보통이다). 없으면 예전처럼 컨테이너 안의 xacro로 떨어진다. 어느 쪽이든
# 실제 빌드가 쓰는 것과 같은 xacro라 결과도 같다.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# 인자에서 -o 출력 경로를 뽑아낸다. 나머지는 그대로 넘긴다.
OUT=""; ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -o) OUT="$2"; shift 2 ;;
    *)  ARGS+=("$1"); shift ;;
  esac
done
IN="${ARGS[0]}"
REL="${IN#"$ROOT"/}"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# --- 호스트 경로 ------------------------------------------------------------
# xacro가 $(find box_cell_description)을 풀려면 ament가 그 패키지를 알아야 한다.
# 빌드했으면 install/setup.bash가 알려 준다. 아직이면 소스 트리를 직접 가리킨다.
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  if [ -f "$ROOT/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$ROOT/install/setup.bash"
  else
    export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}"
    echo "!! 아직 빌드 전이다. \$(find ...)가 안 풀리면 ./scripts/build.sh 먼저." >&2
  fi
fi

if command -v xacro >/dev/null 2>&1; then
  xacro "$IN" -o "$TMP/expanded.xml" "${ARGS[@]:1}"
else
  # --- 컨테이너 경로 --------------------------------------------------------
  echo "호스트에 xacro가 없다. 컨테이너로 떨어진다." >&2
  if ! docker ps >/dev/null 2>&1 && command -v sg >/dev/null; then
    run() { sg docker -c "$*"; }   # docker 그룹이 현재 셸에 아직 반영되지 않은 경우
  else
    run() { eval "$@"; }
  fi
  run "docker run --rm -v '$ROOT/src:/ws/src:ro' -v '$TMP:/out' \
    box_cell_sim:jazzy-harmonic bash -lc \
    'source /ws/install/setup.bash 2>/dev/null; xacro /ws/$REL -o /out/expanded.xml ${ARGS[*]:1}'"
fi

if [ -n "$OUT" ]; then cp "$TMP/expanded.xml" "$OUT"; else cat "$TMP/expanded.xml"; fi
