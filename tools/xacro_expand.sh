#!/usr/bin/env bash
# xacro를 펼쳐 본다. 문법과 치수 계산을 빌드 전에 잡는다.
#
# 컨테이너 안의 진짜 xacro를 쓴다. 예전에는 xacro 소스를 스크래치패드에
# 받아 두고 썼는데, 그 경로가 세션마다 바뀌어 자꾸 끊겼다. 이미지에 있는
# 것을 쓰면 실제 빌드가 쓰는 것과 같은 xacro라 결과도 같다.
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

DOCKER=(docker)
if ! docker ps >/dev/null 2>&1 && command -v sg >/dev/null; then
  # docker 그룹이 현재 셸에 아직 반영되지 않은 경우
  run() { sg docker -c "$*"; }
else
  run() { eval "$@"; }
fi

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

run "docker run --rm -v '$ROOT/src:/ws/src:ro' -v '$TMP:/out' \
  box_cell_sim:jazzy-harmonic bash -lc \
  'xacro /ws/${REL#src/../} -o /out/expanded.xml ${ARGS[*]:1}'" \
  || run "docker run --rm -v '$ROOT/src:/ws/src:ro' -v '$TMP:/out' \
  box_cell_sim:jazzy-harmonic bash -lc \
  'source /ws/install/setup.bash 2>/dev/null; xacro /ws/$REL -o /out/expanded.xml ${ARGS[*]:1}'"

if [ -n "$OUT" ]; then cp "$TMP/expanded.xml" "$OUT"; else cat "$TMP/expanded.xml"; fi
