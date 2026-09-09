#!/usr/bin/env python3
"""펼친 URDF로 FR5의 정기구학과 역기구학을 직접 풀어 도달성을 확인한다.

기획서 "배치와 도달 검증"은 평면 거리만 봤다. 거리가 고리 안이라는 것과
그 자세로 실제 갈 수 있다는 것은 다른 이야기다. 관절 한계, 손목 특이점,
석션 TCP 오프셋 220.5까지 넣어야 답이 나온다.

여기서 확인하는 것 :
  - 대기 자세의 TCP 위치
  - 판독 위치 파지 자세와 그 상공
  - 팔레트 슬롯 16개(2장 x 2층 x 4자리)의 적재 자세
  - 예외 통 투하 자세
모두 TCP를 수직 하향(석션 컵이 박스 상면을 내려다보는 자세)으로 고정하고
감쇠 최소자승 IK로 푼다. MoveIt의 KDL과 같은 계열이라 여기서 안 풀리면
MoveIt에서도 안 풀린다고 보면 된다.

    tools/xacro_expand.sh src/box_cell_description/urdf/box_cell_robot.urdf.xacro -o /tmp/robot.urdf
    <venv>/bin/python tools/verify_reach.py /tmp/robot.urdf
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

CELL_YAML = Path(__file__).resolve().parent.parent / "src/box_cell_description/config/cell.yaml"


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def transform(xyz: np.ndarray, rot: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = xyz
    return t


class Chain:
    """URDF에서 base부터 tip까지의 사슬 하나를 뽑아 FK/IK를 푼다."""

    def __init__(self, urdf: Path, tip: str = "tcp_link", base: str = "world") -> None:
        root = ET.parse(urdf).getroot()
        self.joints: dict[str, ET.Element] = {}
        self.parent_of: dict[str, tuple[str, ET.Element]] = {}
        for j in root.findall("joint"):
            child = j.find("child").get("link")
            self.parent_of[child] = (j.find("parent").get("link"), j)
            self.joints[j.get("name")] = j

        # tip에서 base까지 거슬러 올라간 뒤 뒤집는다
        path: list[ET.Element] = []
        link = tip
        while link != base:
            if link not in self.parent_of:
                raise SystemExit(f"{link}에서 {base}까지 이어지지 않는다")
            parent, joint = self.parent_of[link]
            path.append(joint)
            link = parent
        path.reverse()

        self.steps: list[dict] = []
        self.movable: list[str] = []
        self.limits: list[tuple[float, float]] = []
        for j in path:
            origin = j.find("origin")
            xyz = np.array([float(v) for v in (origin.get("xyz", "0 0 0")).split()]) if origin is not None else np.zeros(3)
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0")).split()] if origin is not None else [0, 0, 0]
            step = {
                "name": j.get("name"),
                "type": j.get("type"),
                "origin": transform(xyz, rpy_to_matrix(*rpy)),
                "axis": np.array([float(v) for v in (j.find("axis").get("xyz").split())]) if j.find("axis") is not None else None,
            }
            self.steps.append(step)
            if step["type"] in ("revolute", "continuous", "prismatic"):
                self.movable.append(step["name"])
                lim = j.find("limit")
                self.limits.append((float(lim.get("lower")), float(lim.get("upper"))))

    def link_origins(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """각 관절 뒤 링크 원점의 world 위치. 상판 침범 확인용."""
        out: dict[str, np.ndarray] = {}
        t = np.eye(4)
        k = 0
        for s in self.steps:
            t = t @ s["origin"]
            if s["type"] in ("revolute", "continuous"):
                axis = s["axis"] / np.linalg.norm(s["axis"])
                t = t @ transform(np.zeros(3), axis_angle(axis, q[k]))
                k += 1
            out[s["name"]] = t[:3, 3].copy()
        return out

    def fk(self, q: np.ndarray, upto: int | None = None) -> np.ndarray:
        t = np.eye(4)
        k = 0
        for i, s in enumerate(self.steps):
            t = t @ s["origin"]
            if s["type"] in ("revolute", "continuous"):
                axis = s["axis"] / np.linalg.norm(s["axis"])
                t = t @ transform(np.zeros(3), axis_angle(axis, q[k]))
                k += 1
            if upto is not None and i == upto:
                break
        return t

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """수치 야코비안. 6 x n."""
        eps = 1e-6
        base = self.fk(q)
        jac = np.zeros((6, len(q)))
        for i in range(len(q)):
            dq = q.copy()
            dq[i] += eps
            t = self.fk(dq)
            jac[:3, i] = (t[:3, 3] - base[:3, 3]) / eps
            dr = t[:3, :3] @ base[:3, :3].T
            jac[3:, i] = rot_to_vec(dr) / eps
        return jac

    def solve(
        self,
        target: np.ndarray,
        seeds: list[np.ndarray],
        restarts: int = 60,
        rng: np.random.Generator | None = None,
        accept=None,
    ) -> tuple[np.ndarray | None, float, float]:
        """여러 시드에서 풀어 본다.

        감쇠 최소자승은 국소 최소에 쉽게 갇힌다. 특히 손목 분기가 다른 목표로
        건너뛸 때 그렇다. MoveIt이 쓰는 KDL도 같은 이유로 무작위 재시작을 돈다.
        여기서도 같은 조건으로 맞춰야 'IK가 풀린다'는 말이 의미를 갖는다.
        """
        rng = rng or np.random.default_rng(0)
        best = (None, float("inf"), float("inf"))
        trials = list(seeds)
        for _ in range(restarts):
            trials.append(np.array([rng.uniform(lo, hi) for lo, hi in self.limits]))

        # 첫 시드(= 직전 자세)에서 관절 이동량이 가장 적은 해를 고른다.
        # 아무 해나 잡으면 팔꿈치가 뒤집히는 자세가 나와서, 도달은 하지만
        # 사이클 시간이 늘고 전시 화면에서 팔이 크게 휘젓는다.
        ref = seeds[0]
        found: list[tuple[float, np.ndarray, float, float]] = []
        for seed in trials:
            q, ep, er = self.ik(target, seed)
            if q is not None:
                if accept is not None and not accept(q):
                    continue          # 도달은 하지만 쓸 수 없는 자세다
                found.append((float(np.abs(q - ref).max()), q, ep, er))
                if len(found) >= 8:
                    break
            elif ep < best[1]:
                best = (None, ep, er)
        if found:
            found.sort(key=lambda x: x[0])
            _, q, ep, er = found[0]
            return q, ep, er
        return best

    def ik(
        self,
        target: np.ndarray,
        seed: np.ndarray,
        iters: int = 400,
        tol_pos: float = 5e-5,
        tol_rot: float = 1e-3,
    ) -> tuple[np.ndarray | None, float, float]:
        q = seed.copy()
        lam = 0.05
        for _ in range(iters):
            cur = self.fk(q)
            dp = target[:3, 3] - cur[:3, 3]
            dr = rot_to_vec(target[:3, :3] @ cur[:3, :3].T)
            err = np.concatenate([dp, dr])
            if np.linalg.norm(dp) < tol_pos and np.linalg.norm(dr) < tol_rot:
                return q, float(np.linalg.norm(dp)), float(np.linalg.norm(dr))
            jac = self.jacobian(q)
            # 감쇠 최소자승
            dq = jac.T @ np.linalg.solve(jac @ jac.T + lam**2 * np.eye(6), err)
            q = q + np.clip(dq, -0.2, 0.2)
            for i, (lo, hi) in enumerate(self.limits):
                q[i] = min(max(q[i], lo), hi)
        cur = self.fk(q)
        return None, float(np.linalg.norm(target[:3, 3] - cur[:3, 3])), float(
            np.linalg.norm(rot_to_vec(target[:3, :3] @ cur[:3, :3].T))
        )


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def rot_to_vec(r: np.ndarray) -> np.ndarray:
    angle = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    if abs(angle) < 1e-9:
        return np.zeros(3)
    return angle / (2 * math.sin(angle)) * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def down_pose(x: float, y: float, z: float, yaw: float = 0.0) -> np.ndarray:
    """TCP가 수직 하향인 자세. 석션 컵의 +z가 아래(-z world)를 본다."""
    rot = rpy_to_matrix(math.pi, 0.0, yaw)
    return transform(np.array([x, y, z]), rot)


def pallet_targets(cell: dict, pallet: dict, kind: str) -> list[tuple[str, float, float, float]]:
    """이 팔레트에 그 규격을 채웠을 때 나오는 적재 자리들. (이름, x, y, 상면 z).

    기획서 5.3은 격자 공식으로 자리를 정했다. 규격이 여러 가지가 되면서
    그 공식을 버렸고, 자리는 실제 적재에 쓰는 패커가 정한다. 도구가 다른
    계산을 쓰면 통과해도 의미가 없으므로 같은 패커를 돌린다.
    """
    import sys as _sys

    _sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src/box_cell_common"))
    from box_cell_common.pallet_pack import Packer

    cfg = cell["pallet"]["slots"]
    pk = Packer(
        size=float(cell["pallet"]["size"]),
        margin=float(cfg["margin"]),
        gap=float(cfg["gap"]),
        step=float(cfg["step"]),
        max_layers=int(cfg["layers"]),
    )
    sx, sy, sz = cell["box"]["kinds"][kind]["size"]
    px, py = pallet["center"]
    base = cell["frame"]["table_top_height"] + cell["pallet"]["thickness"]
    out = []
    while len(out) < 8:
        pl = pk.find(sx, sy, sz)
        if pl is None:
            break
        pk.add(pl, "")
        out.append(
            (
                f"P{pallet['id']} {kind} #{len(out)} (층{pl.layer})",
                px + pl.x,
                py + pl.y,
                base + pl.z_base + pl.sz,
            )
        )
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf", type=Path)
    ap.add_argument("--cell", type=Path, default=CELL_YAML)
    args = ap.parse_args()

    cell = yaml.safe_load(args.cell.read_text())
    chain = Chain(args.urdf)

    # MoveIt의 joint_limits.yaml이 URDF보다 좁게 묶은 관절이 있으면 그것도 지킨다.
    #
    # 안 보면 이 도구가 "도달한다"고 한 해를 MoveIt이 못 쓰는 일이 생긴다.
    # 실제로 그랬다. 예외 통 투하 자세를 j2=-180.7, j4=+4.7, j5=-90으로 풀어
    # 통과 판정했는데, MoveIt은 그 범위를 쓰지 않는다. 적재 하강이 풀리는
    # 자세 계열로 로봇을 묶어 두었기 때문이다(joint_limits.yaml 주석 참고).
    ml = Path(__file__).resolve().parent.parent / "src/box_cell_moveit_config/config/joint_limits.yaml"
    narrowed: dict[str, tuple[float, float]] = {}
    if ml.exists():
        jl = (yaml.safe_load(ml.read_text()) or {}).get("joint_limits", {}) or {}
        for name, spec in jl.items():
            if spec.get("has_position_limits"):
                narrowed[name] = (float(spec["min_position"]), float(spec["max_position"]))
    if narrowed:
        for i, name in enumerate(chain.movable):
            if name in narrowed:
                lo, hi = narrowed[name]
                a, b = chain.limits[i]
                chain.limits[i] = (max(a, lo), min(b, hi))
        print("MoveIt이 좁힌 관절 : " + ", ".join(
            f"{n} [{math.degrees(v[0]):+.0f}, {math.degrees(v[1]):+.0f}]"
            for n, v in narrowed.items()))
    print(f"사슬 : world -> tcp_link, 가동 관절 {chain.movable}")

    home = np.array(cell["tuning"]["home_joints"], dtype=float)
    t_home = chain.fk(home)
    print(f"\n대기 자세 TCP  world ({t_home[0,3]:.3f}, {t_home[1,3]:.3f}, {t_home[2,3]:.3f})")
    top = cell["frame"]["table_top_height"]
    print(f"               상판 기준 z {t_home[2,3]-top:+.3f} m")

    # TCP 오프셋 확인 : wrist3_link에서 tcp_link까지
    t_w = chain.fk(home, upto=None)
    wrist_chain = Chain(args.urdf, tip="wrist3_link")
    t_wr = wrist_chain.fk(home)
    off = np.linalg.norm(t_w[:3, 3] - t_wr[:3, 3])
    ok_off = abs(off - cell["tool"]["tcp_offset"]) < 1e-6
    print(f"TCP 오프셋      {off*1000:.1f} mm  (cell.yaml {cell['tool']['tcp_offset']*1000:.1f}) {'ok' if ok_off else '불일치'}")

    # 로봇 베이스 위치. j1 시드를 목표 방향으로 잡는 데 쓴다.
    t_home_base = np.array(
        [cell["robot"]["base_center"][0], cell["robot"]["base_center"][1], top]
    )

    def above_table(q: np.ndarray) -> bool:
        """팔꿈치와 손목 관절이 상판 위에 있어야 쓸 수 있는 자세다.

        IK는 상판을 모른다. 팔꿈치를 아래로 접은 해도 수치상으로는 맞는데,
        실물에서는 상판을 때린다. 여기서 걸러 두면 '전부 도달'이라는 말이
        '전부 쓸 수 있다'는 뜻이 된다.

        관절 원점만 보는 거친 검사다. 링크 몸통까지 보는 정밀 검사는
        Planning Scene에 상판을 넣은 MoveIt이 한다(기획서 5.2).
        """
        origins = chain.link_origins(q)
        return all(origins[j][2] >= top for j in ("j3", "j4", "j5", "j6"))

    targets: list[tuple[str, np.ndarray]] = []

    tun = cell["tuning"]
    rs = cell["read_station"]
    # 규격이 여러 가지라 "박스 높이"라는 하나의 값이 없다. 판독 자리는
    # 제일 낮은 규격과 제일 높은 규격을 둘 다 본다. 컵이 가야 하는 높이가
    # 그만큼 벌어지기 때문이다.
    kinds = cell["box"]["kinds"]
    h_min = min(k["size"][2] for k in kinds.values())
    h_max = max(k["size"][2] for k in kinds.values())
    box_h = kinds[cell["box"].get("default_kind", "M")]["size"][2]
    read_top = top + rs["surface_z"] + box_h
    targets.append(("판독 파지", down_pose(rs["center"][0], rs["center"][1], read_top)))
    targets.append(("판독 접근", down_pose(rs["center"][0], rs["center"][1], read_top + tun["pick_approach"])))
    targets.append(("판독 상공", down_pose(rs["center"][0], rs["center"][1], top + tun["read_hover_z"])))
    # 라벨이 45도 돌아온 최악의 경우
    targets.append(("판독 파지 (제일 낮은 규격)",
                    down_pose(rs["center"][0], rs["center"][1], top + rs["surface_z"] + h_min)))
    targets.append(("판독 파지 (제일 높은 규격)",
                    down_pose(rs["center"][0], rs["center"][1], top + rs["surface_z"] + h_max)))
    targets.append(("판독 파지 yaw+45", down_pose(rs["center"][0], rs["center"][1], read_top, math.radians(45))))
    targets.append(("판독 파지 yaw-45", down_pose(rs["center"][0], rs["center"][1], read_top, math.radians(-45))))
    # 디팔레타이징한 박스를 벨트에 올려 놓는 자리
    conv = cell["conveyor"]
    targets.append(("벨트 투입", down_pose(conv["infeed_x"], conv["belt"]["center_y"], read_top)))
    targets.append(("벨트 투입 상공", down_pose(conv["infeed_x"], conv["belt"]["center_y"], top + tun["read_hover_z"])))

    # 적재 자리. 규격마다 자리가 달라지므로 제일 작은 것과 제일 큰 것을
    # 둘 다 본다. 작은 규격은 자리가 많아 팔레트 구석까지 가고, 큰 규격은
    # 높이 올라간다. 양 끝이 되면 사이는 된다.
    small = min(kinds, key=lambda k: kinds[k]["size"][2])
    large = max(kinds, key=lambda k: kinds[k]["size"][2])
    for unit in cell["pallet"]["units"]:
        for kind in dict.fromkeys((small, large)):
            for name, x, y, z in pallet_targets(cell, unit, kind):
                targets.append((name, down_pose(x, y, z)))
        cx, cy = unit["center"]
        targets.append((f"P{unit['id']} 상공", down_pose(cx, cy, top + tun["pallet_hover_z"])))

    eb = cell["exception_bin"]
    targets.append(("예외 통 투하", down_pose(eb["center"][0], eb["center"][1], top + eb["height"] + eb["drop_height"])))

    print(f"\n{'목표':22s} {'해':>4s}  {'오차mm':>7s} {'이동deg':>7s}  {'관절 j1..j6 (deg)'}")
    failures: list[str] = []
    seed = home.copy()
    rng = np.random.default_rng(20260907)
    for name, target in targets:
        # 시드 : 직전 해(연속 동작과 같은 조건), 대기 자세, 그리고 j1을 목표
        # 방향으로 미리 돌려 둔 것. 그래도 안 되면 무작위 재시작.
        aim = math.atan2(target[1, 3] - t_home_base[1], target[0, 3] - t_home_base[0])
        aimed = home.copy()
        aimed[0] = float(np.clip(-aim - math.radians(14.5), *chain.limits[0]))
        q, ep, er = chain.solve(
            target, [seed, home.copy(), aimed], rng=rng, accept=above_table
        )
        if q is None:
            failures.append(f"{name}: 위치오차 {ep*1000:.1f} mm, 자세오차 {math.degrees(er):.1f} deg")
            print(f"{name:22s} {'실패':>4s}  {ep*1000:10.2f}")
            continue
        travel = math.degrees(float(np.abs(q - seed).max()))
        seed = q  # 다음 목표는 직전 해에서 출발한다. 실제 연속 동작과 같은 조건.
        margin = min(min(q[i] - lo, hi - q[i]) for i, (lo, hi) in enumerate(chain.limits))
        flag = "" if margin > math.radians(5) else f"  <= 한계까지 {math.degrees(margin):.1f} deg"
        print(
            f"{name:22s} {'ok':>4s}  {ep*1000:7.3f} {travel:7.1f}  "
            + " ".join(f"{math.degrees(v):7.1f}" for v in q)
            + flag
        )
        if margin <= 0:
            failures.append(f"{name}: 관절 한계 초과")

    print()
    if failures:
        print("실패:")
        for f in failures:
            print("  - " + f)
        return 1
    print(f"목표 {len(targets)}개 전부 도달. 관절 한계 안.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
