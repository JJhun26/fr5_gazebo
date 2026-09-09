#!/usr/bin/env python3
"""판독 위치를 집을 때 팔이 C1 카메라를 비켜 가는지 확인한다.

왜 필요한가.
    C1(D435f)은 갠트리에 매달려 컨베이어 판독 위치를 내려다본다. 로봇이 그
    자리의 박스를 집으려면 손목이 카메라 바로 아래를 지난다. 도달성 검사
    (verify_reach.py)는 이 충돌을 못 본다. IK는 카메라를 모르기 때문이다.

    실제로 이 충돌이 났다. 판독이 실제보다 100 mm 상류의 좌표를 내놓자
    (그때는 영상 지연 때문이었다) 로봇이 카메라 바로 밑을 파고들었고,
    MoveIt이 forearm_link / wrist1_link / wrist2_link와 camera_c1_conveyor의
    접촉을 잡아 계획을 버렸다. 사이클이 통째로 실패했다.

    그래서 "집어도 되는 x 하한"을 숫자로 못박아 두어야 한다. cell.yaml의
    read_station.pick_x_min이 그 숫자이고, 이 도구가 그 근거다.

무엇을 재는가.
    팔 링크를 관절 원점 사이의 캡슐(반지름 link_radius)로 보고, 카메라 충돌
    상자와의 최단 거리를 낸다. MoveIt의 FCL만큼 정밀하지는 않지만 여유가
    어디서 사라지는지 보기에는 충분하다. 파지 자세와 그 위 접근 자세를 모두
    본다. 손목이 더 높이 오는 접근 자세가 언제나 더 빠듯하다.

    tools/xacro_expand.sh src/box_cell_description/urdf/box_cell_robot.urdf.xacro -o robot.urdf
    python3 tools/verify_camera_clearance.py robot.urdf
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_reach import CELL_YAML, Chain, down_pose  # noqa: E402

# FR5 상완/전완의 굵기. 링크를 캡슐로 볼 때의 반지름이다.
LINK_RADIUS = 0.060
# 손목 위쪽 사슬만 본다. 어깨는 카메라 근처에 갈 일이 없다.
LINK_CHAIN = ["j2", "j3", "j4", "j5", "j6"]


def segment_box_distance(a: np.ndarray, b: np.ndarray, center: np.ndarray,
                         half: np.ndarray, samples: int = 41) -> float:
    """선분과 축정렬 상자의 최단 거리."""
    best = float("inf")
    for t in np.linspace(0.0, 1.0, samples):
        p = a + (b - a) * t
        d = np.maximum(np.abs(p - center) - half, 0.0)
        best = min(best, float(np.linalg.norm(d)))
    return best


def transit_path(chain, q_from: np.ndarray, q_to: np.ndarray, steps: int = 40):
    """Pilz PTP가 지나는 자세들. 관절 공간 직선이다."""
    for k in range(steps + 1):
        yield q_from + (q_to - q_from) * (k / steps)


def worst_clearance(chain, q: np.ndarray, center, half, tool_radius: float) -> float:
    """이 자세에서 카메라 상자와 팔/공구의 최단 여유."""
    o = chain.link_origins(q)
    d = min(
        segment_box_distance(np.array(o[LINK_CHAIN[i]]), np.array(o[LINK_CHAIN[i + 1]]),
                             center, half)
        for i in range(len(LINK_CHAIN) - 1)
    ) - LINK_RADIUS
    # 손목 끝에서 TCP까지가 그리퍼와 물고 있는 박스다. 더 굵게 본다.
    tcp = chain.fk(q)[:3, 3]
    dt = segment_box_distance(np.array(o["j6"]), tcp, center, half) - tool_radius
    return min(d, dt)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf", type=Path)
    ap.add_argument("--cell", type=Path, default=CELL_YAML)
    ap.add_argument("--from-mm", type=int, default=280)
    ap.add_argument("--to-mm", type=int, default=520)
    ap.add_argument("--step-mm", type=int, default=20)
    args = ap.parse_args()

    cell = yaml.safe_load(args.cell.read_text())
    chain = Chain(args.urdf)

    top = cell["frame"]["table_top_height"]
    rs, tun = cell["read_station"], cell["tuning"]
    read_top = top + rs["surface_z"] + cell["box"]["size"][2]
    y = rs["center"][1]
    home = np.array(tun["home_joints"], dtype=float)

    # cell_geometry가 Planning Scene에 넣는 카메라 상자와 같은 치수여야 한다.
    cam = cell["cameras"]["c1_conveyor"]
    center = np.array([cam["xyz"][0], cam["xyz"][1], top + cam["xyz"][2]])
    half = np.array([0.06, 0.14, 0.06]) / 2.0

    def above_table(q: np.ndarray) -> bool:
        o = chain.link_origins(q)
        return all(o[j][2] >= top for j in ("j3", "j4", "j5", "j6"))

    print(f"C1 충돌 상자  중심 ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}) "
          f"반치수 ({half[0]:.3f}, {half[1]:.3f}, {half[2]:.3f})")
    print(f"링크 반지름   {LINK_RADIUS*1000:.0f} mm")
    print(f"판독면 z      {read_top:.3f}   접근 +{tun['pick_approach']:.3f}\n")
    print(f"{'x mm':>6} {'파지 여유':>11} {'접근 여유':>11}  판정")

    safe_min: int | None = None
    for x_mm in range(args.from_mm, args.to_mm + 1, args.step_mm):
        x = x_mm / 1000.0
        gaps: list[float | None] = []
        for z in (read_top, read_top + tun["pick_approach"]):
            q, _, _ = chain.solve(down_pose(x, y, z), [home], restarts=60, accept=above_table)
            if q is None:
                gaps.append(None)
                continue
            o = chain.link_origins(q)
            d = min(
                segment_box_distance(
                    np.array(o[LINK_CHAIN[i]]), np.array(o[LINK_CHAIN[i + 1]]), center, half
                )
                for i in range(len(LINK_CHAIN) - 1)
            )
            gaps.append(d - LINK_RADIUS)

        usable = [g for g in gaps if g is not None]
        worst = min(usable) if usable else None
        ok = worst is not None and worst > 0.0 and len(usable) == 2
        if ok and safe_min is None:
            safe_min = x_mm
        if not ok:
            safe_min = None      # 연속 구간만 인정한다

        show = lambda g: "IK없음" if g is None else f"{g*1000:+.0f} mm"  # noqa: E731
        print(f"{x_mm:>6} {show(gaps[0]):>11} {show(gaps[1]):>11}  "
              f"{'여유' if ok else '충돌'}")

    # --- 이송 경로. Pilz PTP는 관절 공간 직선이라 툴이 뒤로 크게 휜다.
    # 목표 두 점이 멀쩡해도 그 사이가 카메라를 지나면 계획이 통째로 버려지고
    # OMPL 대체 경로로 넘어간다. 그 경로는 시뮬레이터가 못 따라간다.
    # (60 각 박스를 물고 있으므로 공구 반지름은 대각 반 + 여유로 본다.)
    box = cell["box"]["size"]
    tool_r = math.hypot(box[0], box[1]) / 2.0 + 0.02
    pal = cell["pallet"]
    hover_z = top + tun["pallet_hover_z"]
    print("\n== 이송 경로 (판독 이탈점 -> 팔레트 상공) ==")
    print(f"공구 반지름 {tool_r*1000:.0f} mm (박스 대각 반 + 20)")
    q_home = home
    worst_all = float("inf")
    for unit in pal["units"]:
        cx, cy = unit["center"]
        q_a, _, _ = chain.solve(
            down_pose(rs["center"][0], y, read_top + tun["retreat"]), [q_home],
            restarts=60, accept=above_table)
        q_b, _, _ = chain.solve(down_pose(cx, cy, hover_z), [q_home],
                                restarts=60, accept=above_table)
        if q_a is None or q_b is None:
            print(f"  P{unit['id']}  IK 없음")
            return 1
        gaps = [worst_clearance(chain, q, center, half, tool_r)
                for q in transit_path(chain, q_a, q_b)]
        w = min(gaps)
        k = gaps.index(w)
        worst_all = min(worst_all, w)
        print(f"  P{unit['id']}  최소 여유 {w*1000:+.0f} mm  (경로 {k}/{len(gaps)-1} 지점)"
              f"  {'여유' if w > 0 else '충돌'}")

    limit = rs.get("pick_x_min")
    print()
    if safe_min is None:
        print("여유 있는 구간이 없다. 카메라 위치를 다시 잡아야 한다.")
        return 1
    print(f"연속으로 여유가 있는 하한 : x = {safe_min} mm")
    if limit is None:
        print("cell.yaml에 read_station.pick_x_min이 없다.")
        return 1
    if round(limit * 1000) < safe_min:
        print(f"cell.yaml pick_x_min {limit*1000:.0f} mm는 하한보다 상류다. 올려야 한다.")
        return 1
    print(f"cell.yaml pick_x_min {limit*1000:.0f} mm  ok")
    if worst_all <= 0:
        print("이송 경로가 카메라를 지난다. Pilz PTP가 못 쓰이고 OMPL로 넘어간다.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
