#!/usr/bin/env python3
"""cell.yaml 하나로 기획서의 배치 검증 수치를 다시 계산한다.

기획서 "배치와 도달 검증"은 네 개의 거리(414, 554, 603, 369)와
"팔레트 여덟 모서리의 거리는 398에서 806으로 모두 도달 고리 안"이라는
한 문장을 근거로 배치를 확정했다. 이 스크립트는 cell.yaml에서 그 값을
다시 만들어 낸다. 값이 어긋나면 cell.yaml을 잘못 고친 것이다.

카메라는 화각과 거리에서 지상 분해능(px/mm)을 뽑아, 40 mm QR이 몇 픽셀로
찍히는지 함께 낸다. 기획서 7절 "카메라 기종과 조명 : 라벨 크기에서 해상도
역산"이 요구하는 계산이다.

    python3 tools/verify_layout.py
    python3 tools/verify_layout.py --json     # CI에서 쓰기 좋은 형태
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import yaml

CELL_YAML = Path(__file__).resolve().parent.parent / "src/box_cell_description/config/cell.yaml"

# RealSense 실측 제원. 데이터시트 공칭값이며 URDF의 센서 정의와 같은 원본을 쓴다.
#   fov  : (수평, 수직) degree
#   res  : (가로, 세로) px
#   range: (최소, 최대) m
CAMERA_SPECS: dict[str, dict[str, Any]] = {
    "d435f": {
        "color": {"fov": (69.0, 42.0), "res": (1920, 1080)},
        "depth": {"fov": (87.0, 58.0), "res": (848, 480), "range": (0.30, 3.00)},
        "baseline": 0.050,
    },
    "d455": {
        "color": {"fov": (90.0, 65.0), "res": (1280, 800)},
        "depth": {"fov": (87.0, 58.0), "res": (848, 480), "range": (0.60, 6.00)},
        "baseline": 0.095,
    },
    "d405": {
        # D405는 스테레오 모듈 하나가 컬러와 깊이를 함께 낸다.
        "color": {"fov": (87.0, 58.0), "res": (1280, 720)},
        "depth": {"fov": (87.0, 58.0), "res": (1280, 720), "range": (0.07, 0.50)},
        "baseline": 0.018,
    },
    "generic_rgb": {
        "color": {"fov": (69.0, 42.0), "res": (1280, 720)},
        "depth": None,
        "baseline": 0.0,
    },
}

PASS = "  ok "
FAIL = "  !! "


class Report:
    def __init__(self) -> None:
        self.failures: list[str] = []
        self.rows: list[dict[str, Any]] = []

    def check(self, name: str, value: float, expect: float, tol: float, unit: str = "mm") -> None:
        ok = abs(value - expect) <= tol
        if not ok:
            self.failures.append(f"{name}: {value:.1f} {unit} (기대 {expect} ± {tol})")
        self.rows.append({"name": name, "value": value, "expect": expect, "ok": ok, "unit": unit})
        mark = PASS if ok else FAIL
        print(f"{mark}{name:38s} {value:9.1f} {unit}   기대 {expect} ± {tol}")

    def note(self, name: str, value: float, unit: str = "") -> None:
        self.rows.append({"name": name, "value": value, "ok": None, "unit": unit})
        print(f"  ·  {name:38s} {value:9.2f} {unit}")


def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def slot_xy(pallet_center: tuple[float, float], index: int, pitch: float) -> tuple[float, float]:
    """기획서 5.3의 격자 공식. col = i % 2, row = (i // 2) % 2, layer = i // 4."""
    col = index % 2
    row = (index // 2) % 2
    return (
        pallet_center[0] + (col - 0.5) * pitch,
        pallet_center[1] + (row - 0.5) * pitch,
    )


def slot_z(pallet_thickness: float, index: int, box_height: float) -> float:
    """TCP가 잡는 높이 = 팔레트 두께 + 층 * 박스높이 + 박스높이(상면)."""
    layer = index // 4
    return pallet_thickness + layer * box_height + box_height


def ground_resolution(cam_spec: dict[str, Any], standoff: float) -> tuple[float, float, float]:
    """거리 standoff에서의 (가로 시야 m, 세로 시야 m, px/mm)."""
    fov_h, fov_v = cam_spec["fov"]
    res_h, _res_v = cam_spec["res"]
    width = 2.0 * standoff * math.tan(math.radians(fov_h) / 2.0)
    height = 2.0 * standoff * math.tan(math.radians(fov_v) / 2.0)
    px_per_mm = res_h / (width * 1000.0)
    return width, height, px_per_mm


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", type=Path, default=CELL_YAML)
    ap.add_argument("--json", action="store_true", help="결과를 JSON으로도 출력")
    args = ap.parse_args()

    cell = yaml.safe_load(args.cell.read_text())
    r = Report()

    robot = tuple(cell["robot"]["base_center"])
    inner = cell["robot"]["reach"]["inner"]
    outer = cell["robot"]["reach"]["outer"]

    print("\n== 로봇 중심에서의 거리 (기획서 '배치와 도달 검증') ==")
    r.check("라벨 판독 위치 (405, 725)", dist(robot, tuple(cell["read_station"]["center"])) * 1000, 414, 1.0)
    for unit in cell["pallet"]["units"]:
        r.check(f"팔레트 {unit['id']} {tuple(unit['center'])}", dist(robot, tuple(unit["center"])) * 1000, {1: 554, 2: 603}[unit["id"]], 1.0)
    r.check("예외 통 (95, 110)", dist(robot, tuple(cell["exception_bin"]["center"])) * 1000, 369, 1.0)

    print("\n== 팔레트 여덟 모서리 도달 (기대 398 ~ 806, 고리 안) ==")
    half = cell["pallet"]["size"] / 2.0
    corners: list[float] = []
    for unit in cell["pallet"]["units"]:
        cx, cy = unit["center"]
        for sx in (-half, half):
            for sy in (-half, half):
                corners.append(dist(robot, (cx + sx, cy + sy)) * 1000)
    r.check("팔레트 모서리 최소 거리", min(corners), 398, 2.0)
    r.check("팔레트 모서리 최대 거리", max(corners), 806, 2.0)
    inside = all(inner * 1000 <= d <= outer * 1000 for d in corners)
    print(f"{PASS if inside else FAIL}여덟 모서리가 도달 고리({inner*1000:.0f} ~ {outer*1000:.0f}) 안")
    if not inside:
        r.failures.append("팔레트 모서리가 도달 고리를 벗어난다")

    print("\n== 적재 격자 (기획서 5.3) ==")
    pitch = cell["pallet"]["slots"]["pitch"]
    box_h = cell["box"]["size"][2]
    n_slots = cell["pallet"]["slots"]["cols"] * cell["pallet"]["slots"]["rows"] * cell["pallet"]["slots"]["layers"]
    r.check("슬롯 수", n_slots, cell["box"]["count"], 0, "개")
    # 기획서는 블록 한 변을 2 x 피치로 센다. 박스 실제 외곽은 피치 + 박스 = 130.
    block = 2 * pitch
    zone = cell["pallet"]["paint_zone"]
    r.check("2x2 블록 한 변 (2 x 피치)", block * 1000, 140, 0.1)
    r.check("색칠 구역 안 사방 여유", (zone - block) / 2 * 1000, 30, 0.1)
    r.check("박스 외곽 한 변", (pitch + cell["box"]["size"][0]) * 1000, 130, 0.1)
    r.check("팔레트 가장자리까지 여유", (cell["pallet"]["size"] - block) / 2 * 1000, 80, 0.1)
    r.check("박스 사이 간극", (pitch - cell["box"]["size"][0]) * 1000, 10, 0.1)

    slot_dists: list[float] = []
    for unit in cell["pallet"]["units"]:
        for i in range(n_slots):
            sx, sy = slot_xy(tuple(unit["center"]), i, pitch)
            slot_dists.append(dist(robot, (sx, sy)) * 1000)
    print(f"  ·  슬롯 16개 거리 {min(slot_dists):.0f} ~ {max(slot_dists):.0f} mm")
    if not all(inner * 1000 <= d <= outer * 1000 for d in slot_dists):
        r.failures.append("슬롯이 도달 고리를 벗어난다")
        print(f"{FAIL}슬롯이 도달 고리를 벗어난다")
    else:
        print(f"{PASS}슬롯 16개 모두 도달 고리 안")

    top_z = slot_z(cell["pallet"]["thickness"], n_slots - 1, box_h)
    r.check("2층 박스 상면 높이", top_z * 1000, 110, 0.1)

    print("\n== 컨베이어와 판독 ==")
    conv = cell["conveyor"]
    r.check("컨베이어 중심 y", conv["belt"]["center_y"] * 1000, 725, 0.1)
    r.check("상판 뒤쪽 여유", (cell["table"]["size_y"] - conv["belt"]["center_y"] - conv["belt"]["width"] / 2) * 1000, 0, 0.1)
    r.check("판독 높이 (상판 위)", conv["belt"]["surface_z"] * 1000, 180, 0.1)
    box_top = conv["belt"]["surface_z"] + box_h
    r.note("컨베이어 위 박스 상면", box_top * 1000, "mm")

    print("\n== 카메라 ==")
    ok_cams = True
    for name, cam in cell["cameras"].items():
        spec = CAMERA_SPECS[cam["model"]]
        print(f"\n  [{name}]  {cam['model']}")
        if "standoff" in cam:
            standoff = cam["standoff"]
        elif "target_local" in cam:
            standoff = math.dist(cam["xyz"], cam["target_local"])
        else:
            standoff = math.dist(cam["xyz"], cam["target"])

        # rpy가 실제로 target을 겨누는지 확인한다.
        target = cam.get("target") or cam.get("target_local")
        aim = aim_error_deg(cam["xyz"], cam["rpy"], target)
        print(f"     겨냥 오차 {aim:.2f} deg,  거리 {standoff*1000:.0f} mm")
        if aim > 1.0:
            ok_cams = False
            r.failures.append(f"{name}: rpy가 target을 겨누지 않는다 ({aim:.2f} deg)")

        w, h, px_mm = ground_resolution(spec["color"], standoff)
        print(f"     컬러 시야 {w*1000:.0f} x {h*1000:.0f} mm,  {px_mm:.2f} px/mm")
        if spec["depth"]:
            lo, hi = spec["depth"]["range"]
            in_range = lo <= standoff <= hi
            print(f"     깊이 유효 {lo:.2f} ~ {hi:.2f} m -> {'안' if in_range else '밖'}")
            if not in_range:
                ok_cams = False
                r.failures.append(f"{name}: 작동 거리 {standoff:.2f} m가 깊이 유효 범위 밖")

        if name == "c1_conveyor":
            label = cell["box"]["label"]["size"]
            px = label * 1000 * px_mm
            # QR 본체는 라벨에서 여백과 문자열 폭을 뺀 만큼이다.
            lab = cell["box"]["label"]
            qr_mm = (label - 2 * lab.get("quiet", 0.0025) - lab.get("text_strip", 0.004)) * 1000
            qr_px = qr_mm * px_mm
            per_module = qr_px / 21.0        # QR 버전 1 = 21 모듈
            print(
                f"     라벨 {label*1000:.0f} mm = {px:.0f} px, "
                f"QR {qr_mm:.0f} mm = {qr_px:.0f} px, 모듈당 {per_module:.1f} px"
            )
            # 모듈당 5 px 아래로 내려가면 렌더러의 축소 필터가 모듈을 뭉갠다.
            # 실물 카메라도 렌즈 MTF 때문에 사정이 비슷하다.
            if per_module < 5.0:
                ok_cams = False
                r.failures.append(
                    f"{name}: QR 모듈당 {per_module:.1f} px. 5 px 아래면 판독이 무너진다."
                )
        if name == "c2_pallet":
            # 팔레트 두 장을 한 화면에 담는가 (기획서 7절 미확정 항목)
            ys = [u["center"][1] for u in cell["pallet"]["units"]]
            need = (max(ys) - min(ys)) + cell["pallet"]["size"]
            covers = w >= need
            print(f"     팔레트 2장 y 폭 {need*1000:.0f} mm vs 시야 가로 {w*1000:.0f} mm -> {'덮는다' if covers else '못 덮는다'}")
            if not covers:
                ok_cams = False
                r.failures.append("c2_pallet: 팔레트 2장을 한 대로 못 덮는다")

    print("\n== 간섭 ==")
    for post in cell["gantry"]["posts"]:
        d = dist(robot, tuple(post["xy"])) * 1000
        pallet_edge = max(u["center"][0] for u in cell["pallet"]["units"]) + half
        clear = (post["xy"][0] - cell["gantry"]["section"] / 2 - pallet_edge) * 1000
        print(f"  ·  기둥 {post['id']} 로봇 중심에서 {d:.0f} mm, 팔레트 모서리와 {clear:.0f} mm")
        if clear < 20:
            r.failures.append(f"기둥 {post['id']}이 팔레트와 {clear:.0f} mm 밖에 안 뜬다")
        if d < outer * 1000:
            print(f"     기둥이 도달 고리 안({outer*1000:.0f})이다. Planning Scene에 반드시 등록할 것.")

    print()
    if r.failures or not ok_cams:
        print("실패:")
        for f in r.failures:
            print("  - " + f)
    else:
        print("전 항목 통과. 기획서 배치 검증 수치와 일치한다.")

    if args.json:
        print(json.dumps({"rows": r.rows, "failures": r.failures}, ensure_ascii=False, indent=2))
    return 1 if r.failures else 0


def aim_error_deg(xyz: list[float], rpy: list[float], target: list[float]) -> float:
    """링크 +x(광축)가 target을 향하는지 각도 오차로 낸다."""
    roll, pitch, yaw = rpy
    # R = Rz(yaw) Ry(pitch) Rx(roll) 의 첫 열
    axis = (
        math.cos(pitch) * math.cos(yaw),
        math.cos(pitch) * math.sin(yaw),
        -math.sin(pitch),
    )
    to = [target[i] - xyz[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in to))
    if n == 0:
        return 0.0
    to = [v / n for v in to]
    dot = max(-1.0, min(1.0, sum(axis[i] * to[i] for i in range(3))))
    return math.degrees(math.acos(dot))


if __name__ == "__main__":
    sys.exit(main())
