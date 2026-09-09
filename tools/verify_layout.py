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

    print("\n== 적재 (규격 혼재) ==")
    #
    # 기획서 5.3은 격자 공식으로 자리를 정했다. 박스가 모두 60 각 40 높이라는
    # 전제 위에서만 성립한다. 실물 택배 상자는 규격이 제각각이라 그 전제를
    # 버렸고, 자리는 계산이 아니라 탐색으로 정한다(box_cell_common/pallet_pack).
    # 그래서 여기서도 **실제 적재에 쓰는 것과 같은 패커**를 돌려 확인한다.
    # 도구가 다른 계산을 쓰면 통과해도 의미가 없다.
    import sys as _sys
    _sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src/box_cell_common"))
    from box_cell_common.pallet_pack import Packer

    pcfg = cell["pallet"]["slots"]
    kinds = cell["box"]["kinds"]

    def new_pack() -> Packer:
        return Packer(
            size=float(cell["pallet"]["size"]),
            margin=float(pcfg["margin"]),
            gap=float(pcfg["gap"]),
            step=float(pcfg["step"]),
            max_layers=int(pcfg["layers"]),
        )

    for kind, spec in kinds.items():
        sx, sy, sz = spec["size"]
        pk = new_pack()
        while True:
            pl = pk.find(sx, sy, sz)
            if pl is None or len(pk.placed) >= 40:
                break
            pk.add(pl, "")
        per_layer = sum(1 for q in pk.placed if q.layer == 0)
        top = max((q.z_top for q in pk.placed), default=0.0)
        print(f"  ·  규격 {kind:2s} {sx*1000:3.0f}x{sy*1000:3.0f}x{sz*1000:2.0f} : "
              f"1층 {per_layer}개, 최대 {len(pk.placed)}개, 최고 {top*1000:.0f} mm")
        if per_layer == 0:
            r.failures.append(f"규격 {kind}는 팔레트에 한 개도 못 놓는다")

    # 실제 데모 순서대로 넣어 본다. 규격은 MES 시드가 정한다.
    seed_path = Path(__file__).resolve().parent.parent / "src/box_cell_mes/box_cell_mes/seed_items.json"
    if seed_path.exists():
        import json as _json
        seed = _json.loads(seed_path.read_text())
        stack = [i for i in seed if i.get("handling", "normal") not in ("hazmat", "oversize")]
        # 셀은 팔레트를 두 장 쓴다. 한 장에 안 들어가면 다른 장을 본다.
        # task_manager도 같은 순서로 움직인다(NextSlot 실패 -> 반대쪽 시도).
        packs = {u["id"]: new_pack() for u in cell["pallet"]["units"]}
        order = [u["id"] for u in cell["pallet"]["units"]]
        placed = 0
        for it in stack:
            spec = kinds.get(it.get("kind", ""))
            if not spec:
                continue
            for pid in order:
                pl = packs[pid].find(*spec["size"])
                if pl is not None:
                    packs[pid].add(pl, it["code"])
                    placed += 1
                    break
        print(f"  ·  이번 로트 {len(seed)}개 중 적재 대상 {len(stack)}개 -> {placed}개 적재")
        for pid in order:
            for q in packs[pid].placed:
                print(f"       P{pid} {q.code} 층{q.layer} ({q.x*1000:+6.1f},{q.y*1000:+6.1f}) "
                      f"밑면 {q.z_base*1000:5.1f} yaw {math.degrees(q.yaw):.0f}도")
        if placed < len(stack):
            r.failures.append(f"적재 대상 {len(stack)}개 중 {placed}개만 들어간다")

        dists = []
        for unit in cell["pallet"]["units"]:
            for q in packs[unit["id"]].placed:
                dists.append(dist(robot, (unit["center"][0] + q.x, unit["center"][1] + q.y)) * 1000)
        if dists:
            print(f"  ·  적재 자리 거리 {min(dists):.0f} ~ {max(dists):.0f} mm")
            if not all(inner * 1000 <= d <= outer * 1000 for d in dists):
                r.failures.append("적재 자리가 도달 고리를 벗어난다")
            else:
                print(f"{PASS}적재 자리 전부 도달 고리 안")

    # 송장이 제일 작은 상자 위에 들어가는가
    lab = cell["box"]["label"]
    small = min(kinds.values(), key=lambda k: k["size"][0] * k["size"][1])["size"]
    mx = (small[0] - lab["width"]) / 2 * 1000
    my = (small[1] - lab["height"]) / 2 * 1000
    print(f"  ·  송장 {lab['width']*1000:.0f}x{lab['height']*1000:.0f} mm, "
          f"제일 작은 상자 상면 여백 {mx:.1f} / {my:.1f} mm")
    if mx < 2 or my < 2:
        r.failures.append("송장이 제일 작은 상자 상면에 안 들어간다")

    # 판독 관련 수치는 기본 규격 기준으로 본다. 규격이 여러 가지라
    # "박스 높이"라는 하나의 값이 없다.
    box_h = kinds[cell["box"].get("default_kind", "M")]["size"][2]

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
            # 송장은 정사각이 아니다. QR 한 변은 세로가 정한다.
            #   세로 - 여백x2 - 바코드 띠 - 문자열 폭
            lab = cell["box"]["label"]
            lw, lh = lab["width"] * 1000, lab["height"] * 1000
            quiet = lab.get("quiet", 0.009) * 1000
            qr_mm = lh - 2 * quiet - lab.get("barcode_strip", 0.009) * 1000 \
                - lab.get("text_strip", 0.0035) * 1000
            qr_px = qr_mm * px_mm
            per_module = qr_px / 21.0        # QR 버전 1 = 21 모듈
            quiet_modules = quiet / (qr_mm / 21.0)
            print(
                f"     송장 {lw:.0f} x {lh:.0f} mm, "
                f"QR {qr_mm:.0f} mm = {qr_px:.0f} px, 모듈당 {per_module:.1f} px"
            )
            print(f"     QR 여백 {quiet:.1f} mm = {quiet_modules:.1f} 모듈 (규격 4 이상)")
            if quiet_modules < 4.0:
                ok_cams = False
                r.failures.append(
                    f"QR 여백이 {quiet_modules:.1f}모듈이다. 규격은 4모듈을 요구한다."
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
    cfr = cell["camera_frame"]
    gs = float(cfr["section"])
    table_x = float(cell["table"]["size_x"])
    for post in cfr["posts"]:
        d = dist(robot, tuple(post["xy"])) * 1000
        # 기둥은 상판 바깥 바닥에 선다. 상판 모서리와 부딪히면 안 된다.
        clear = (post["xy"][0] - gs / 2 - table_x) * 1000
        print(f"  ·  기둥 {post['id']} 로봇 중심에서 {d:.0f} mm, 상판 모서리와 {clear:.0f} mm")
        if clear < 20:
            r.failures.append(f"기둥 {post['id']}이 상판과 {clear:.0f} mm 밖에 안 뜬다")
        if d < outer * 1000:
            print(f"     기둥이 도달 고리 안({outer*1000:.0f})이다. Planning Scene에 반드시 등록할 것.")
    # 가로 팔은 기둥과 달리 도달 고리 안으로 깊이 들어온다. 높이가 관건이다.
    for arm in cfr["arms"]:
        print(f"  ·  가로 팔 {arm['id']} x {arm['x_min']*1000:.0f}~{arm['x_max']*1000:.0f}, "
              f"y {arm['y']*1000:.0f}, 상판 위 {cfr['arm_z']*1000:.0f} mm")
    print("     팔 높이는 tools/verify_camera_clearance.py가 로봇 동선과 대 본다.")

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
