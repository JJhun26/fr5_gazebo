#!/usr/bin/env python3
"""적재 자리마다 수직 하강 직선이 끝까지 풀리는지 MoveIt에게 직접 묻는다.

왜 필요한가.
    verify_reach.py는 "그 점에 갈 수 있는가"만 본다. 그런데 이 셀에서 사이클을
    가장 많이 죽인 것은 도달성이 아니라 **자세**였다. 적재 상공에 TCP가
    목표와 0.4 mm 차이로 정확히 서 있는데도, 거기서 바로 아래로 내리는
    compute_cartesian_path가 43%에서 끊겼다. 같은 지점을 다른 관절 자세로
    풀어 시작하면 100% 풀린다. FR5는 같은 TCP 점에 여러 자세로 갈 수 있고
    그 자세들이 등가가 아니다(예외 통 지점에서 j1이 148도 다른 두 자세를
    실제로 잡았다).

    그래서 joint_limits.yaml에서 j2, j4, j5를 묶어 로봇을 한 자세 계열에
    가둬 두었다. 이 도구는 그 울타리가 **모든 적재 자리에 대해** 여전히
    충분한지 확인한다. 울타리를 조이면 어느 자리에서는 하강 도중 관절이
    한계에 걸려 멈춘다. 실제로 j4를 [-110, -70]으로 조였을 때 하강이 53%에서
    멈췄고, 멈춘 지점의 j4가 정확히 -70.0이었다.

무엇을 하는가.
    돌고 있는 move_group에 붙어, 자리마다
      1. 상공 자세를 /compute_ik로 푼다 (여러 무작위 시드로)
      2. 그 자세를 시작점으로 [상공, 접근, 적재] 직선을 요청한다
      3. fraction이 1.0인 시드가 하나라도 있는지 본다
    실물 파지 조건과 무관한, 순전히 계획 가능성 검사다.

    ros2 launch box_cell_bringup demo.launch.py headless:=true   # 먼저 띄우고
    python3 tools/verify_place_descent.py

    빈 팔레트만 보면 안 된다.
    자리 20개가 빈 팔레트에서 전부 100%로 풀렸는데도 실제 데모에서는
    세 번째 박스부터 하강이 8%에서 끊겼다. 차이는 하나다. 그때는 팔레트에
    이미 박스가 놓여 있고, 그것들이 계획 씬의 충돌체이며, 내려가는 로봇은
    박스를 하나 물고 있다. 빈 씬 검사는 그 상황을 아예 안 본다.

    그래서 두 번째 단계로 실제 로트를 패커에 그대로 태워 순서대로 놓아
    본다. N번째를 검사하기 전에 1..N-1을 충돌체로 씬에 넣고, 물고 있는
    박스도 TCP에 붙인다. 실패하는 단계와 그때의 이웃이 바로 나온다.
"""

from __future__ import annotations

import math
import random
import sys

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PositionIKRequest,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from shape_msgs.msg import SolidPrimitive
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
SEEDS = 10          # 자리마다 시도할 무작위 시드 수
SEED_RANGE = (-3.05, 3.05)


def down(x: float, y: float, z: float) -> Pose:
    """TCP 수직 하향 자세."""
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.w = 1.0, 0.0
    return p


def main() -> int:
    cell = CellGeometry()
    d = cell.data
    top = d["frame"]["table_top_height"]
    pal = d["pallet"]
    slots = pal["slots"]
    approach = float(slots["approach_height"])
    # 규격이 여러 가지라 하나의 "박스 높이"가 없다. 아래에서 규격별로 본다.
    hover_z = top + float(d["tuning"]["pallet_hover_z"])

    rclpy.init()
    node = Node("verify_place_descent")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    for cli, name in ((ik, "/compute_ik"), (cart, "/compute_cartesian_path")):
        if not cli.wait_for_service(timeout_sec=20.0):
            print(f"{name}가 없다. move_group을 먼저 띄울 것.")
            return 1

    def call(cli, req, timeout=20.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result()

    def state(values: list[float]) -> RobotState:
        st = RobotState()
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(v) for v in values]
        st.joint_state = js
        st.is_diff = False
        return st

    def solve(pose: Pose, seed: list[float]) -> list[float] | None:
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = "fr5_arm"
        req.ik_request.ik_link_name = "tcp_link"
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose = pose
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 1
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state = state(seed)
        res = call(ik, req)
        if res is None or res.error_code.val != 1:
            return None
        names = list(res.solution.joint_state.name)
        pos = list(res.solution.joint_state.position)
        try:
            return [pos[names.index(j)] for j in JOINTS]
        except ValueError:
            return None

    def fraction(start: list[float], x: float, y: float, z: float) -> float:
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "fr5_arm"
        req.link_name = "tcp_link"
        req.waypoints = [down(x, y, hover_z), down(x, y, z + approach), down(x, y, z)]
        req.max_step = 0.002
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.start_state = state(start)
        res = call(cart, req)
        return float(res.fraction) if res else 0.0

    targets: list[tuple[str, float, float, float]] = []
    # 규격이 여러 가지라 고정 격자가 없다. 실제 적재에 쓰는 패커를 돌려
    # 대표 자리를 얻는다. 제일 작은 규격과 제일 큰 규격을 둘 다 본다.
    kinds = d["box"]["kinds"]
    small = min(kinds, key=lambda k: kinds[k]["size"][2])
    large = max(kinds, key=lambda k: kinds[k]["size"][2])
    for unit in pal["units"]:
        for kind in dict.fromkeys((small, large)):
            for i, (x, y, z) in enumerate(
                cell.sample_placements(int(unit["id"]), kind=kind, count=6)
            ):
                targets.append((f"P{unit['id']} {kind} #{i}", x, y, z))
    rs = d["read_station"]
    targets.append(("판독 자리", rs["center"][0], rs["center"][1],
                    cell.read_top_z_for(kinds[large]["size"][2])))
    eb = d["exception_bin"]
    targets.append(("예외 통", eb["center"][0], eb["center"][1],
                    top + float(eb["height"]) + float(eb["drop_height"])))

    random.seed(0)
    print(f"상공 z={hover_z:.3f}, 접근 +{approach:.3f}, 시드 {SEEDS}개/자리\n")
    bad = 0
    for name, x, y, z in targets:
        best = 0.0
        best_q: list[float] | None = None
        for _ in range(SEEDS):
            seed = [random.uniform(*SEED_RANGE) for _ in JOINTS]
            q = solve(down(x, y, hover_z), seed)
            if q is None:
                continue
            f = fraction(q, x, y, z)
            if f > best:
                best, best_q = f, q
            if best > 0.999:
                break
        ok = best > 0.999
        bad += not ok
        posture = ("  " + " ".join(f"{math.degrees(v):+6.1f}" for v in best_q)) if best_q else ""
        print(f"{name:20s} {'ok' if ok else '실패'}  최고 {best*100:6.2f}%{posture}")

    print()
    if bad:
        print(f"{bad}개 자리에서 하강이 끝까지 안 풀린다. "
              "joint_limits.yaml의 j2/j4/j5 범위를 넓혀야 한다.")
        return 1
    print(f"자리 {len(targets)}개 전부 하강이 100% 풀린다.\n")

    return sequence_check(node, cell, call, solve, fraction, hover_z)


def _lot(cell) -> list[tuple[str, str, tuple[float, float, float]]]:
    """MES 시드를 그대로 읽어 실제로 들어오는 로트를 만든다.

    데모가 쌓는 것과 같은 순서, 같은 규격이어야 의미가 있다. 적재 대상이
    아닌 것(hazmat, oversize)은 예외 통으로 빠지므로 여기서도 뺀다.
    """
    import json
    from pathlib import Path

    seed = None
    for base in (Path(__file__).resolve().parent.parent / "src", Path("/ws/src")):
        cand = base / "box_cell_mes/box_cell_mes/seed_items.json"
        if cand.exists():
            seed = cand
            break
    if seed is None:
        return []
    out = []
    for r in json.loads(seed.read_text()):
        if r.get("handling") in ("hazmat", "oversize"):
            continue
        size = cell.box_size_of(r.get("kind", ""))
        if size:
            out.append((r["code"], r.get("kind", ""), size))
    return out


def _obj(name: str, size, xyz, yaw: float, op: int) -> CollisionObject:
    o = CollisionObject()
    o.header.frame_id = "world"
    o.id = name
    o.operation = op
    if op == CollisionObject.ADD:
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(xyz[0]), float(xyz[1]), float(xyz[2])
        )
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        o.primitives = [sp]
        o.primitive_poses = [pose]
    return o


def sequence_check(node, cell, call, solve, fraction, hover_z) -> int:
    """실제 적재 순서를 그대로 재현하며 매 단계의 하강을 확인한다.

    N번째를 검사하기 전에 1..N-1을 계획 씬에 넣는다. 실물에서는
    scene_publisher가 /pallet/state를 받아 하는 일이고, 여기서는 그것을
    손으로 재현한다. 놓인 박스를 씬에 안 넣으면 로봇은 이웃을 통과해
    내려가도 된다고 답한다. 그 답은 데모에서 그대로 실패한다.
    """
    from box_cell_common.pallet_pack import Packer

    lot = _lot(cell)
    if not lot:
        print("MES 시드를 못 찾아 순서 검사를 건너뛴다.")
        return 0

    apply_cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not apply_cli.wait_for_service(timeout_sec=20.0):
        print("/apply_planning_scene이 없다. 순서 검사를 건너뛴다.")
        return 0

    def push(objs: list[CollisionObject]) -> None:
        req = ApplyPlanningScene.Request()
        sc = PlanningScene()
        sc.is_diff = True
        sc.world.collision_objects = objs
        req.scene = sc
        call(apply_cli, req)

    cfg = cell.pallet_slot_cfg()
    pd = cell.data["pallet"]
    deck = cell.table_top + float(pd["thickness"])
    # pallet_manager와 같은 설정으로 만든다. 다르면 자리가 달라져
    # 검사 자체가 다른 셀을 보게 된다.
    pack_cfg = {
        "size": float(pd["size"]),
        "margin": float(cfg["margin"]),
        "gap": float(cfg["gap"]),
        "step": float(cfg["step"]),
        "max_layers": int(cfg["layers"]),
    }
    packs = {pid: Packer(**pack_cfg) for pid in cell.pallet_ids}

    print("실제 로트를 순서대로 놓아 본다 (놓인 박스를 씬에 넣고 검사)\n")
    placed_names: list[str] = []
    bad = 0
    n = 0
    for code, kind, size in lot:
        pl = None
        pid = None
        for cand in cell.pallet_ids:
            pl = packs[cand].find(size[0], size[1], size[2])
            if pl is not None:
                pid = cand
                break
        if pl is None:
            print(f"{code:10s} {kind:2s} 자리 없음 (팔레트가 다 찼다)")
            continue
        n += 1
        px, py = cell.pallet_center(pid)
        x, y = px + pl.x, py + pl.y
        z = deck + pl.z_base + pl.sz          # 박스 상면 = TCP 목표

        # 물고 있는 박스. 이것이 이웃에 닿는지가 핵심이다.
        # motion_server._attach_box와 같은 방식으로 붙인다. 다르게 붙이면
        # 이 검사가 통과해도 실물은 실패한다. 컵 프레임 기준이고, 상면은
        # 컵 접촉면에 붙이고 줄인 만큼은 밑면에서만 뺀다.
        clear = 0.006
        cup_len = float(cell.data["tool"]["cup_length"])
        held = AttachedCollisionObject()
        held.link_name = "es45_cup"
        held.object = _obj("held", (pl.sx, pl.sy, max(0.005, pl.sz - clear)),
                           (0, 0, 0), 0.0, CollisionObject.ADD)
        held.object.header.frame_id = "es45_cup"
        held.object.primitive_poses[0].position.z = cup_len + pl.sz / 2.0 - clear / 2.0
        held.touch_links = ["es45_cup", "es45_body", "es45_adapter", "wrist3_link"]
        req = ApplyPlanningScene.Request()
        sc = PlanningScene()
        sc.is_diff = True
        sc.robot_state.is_diff = True
        sc.robot_state.attached_collision_objects = [held]
        req.scene = sc
        call(apply_cli, req)

        best = 0.0
        for _ in range(SEEDS):
            seed = [random.uniform(*SEED_RANGE) for _ in JOINTS]
            q = solve(down(x, y, hover_z), seed)
            if q is None:
                continue
            best = max(best, fraction(q, x, y, z))
            if best > 0.999:
                break

        # 물던 것을 뗀다.
        #
        # 떼기만 하면 안 된다. MoveIt은 부착물을 떼면 그것을 그 자리의
        # 월드 충돌체로 되돌려 놓는다. 그대로 두면 마지막 TCP 자리에
        # "held"라는 유령 상자가 남아 다음 검사를 통째로 막는다. 실제로
        # 그것 때문에 빈 팔레트 검사가 9자리에서 0%가 나왔다.
        held.object.operation = CollisionObject.REMOVE
        req = ApplyPlanningScene.Request()
        sc = PlanningScene()
        sc.is_diff = True
        sc.robot_state.is_diff = True
        sc.robot_state.attached_collision_objects = [held]
        req.scene = sc
        call(apply_cli, req)
        push([_obj("held", None, None, 0.0, CollisionObject.REMOVE)])

        # 패커에도 확정해 넣는다. find만 하고 add를 안 하면 다음 박스가
        # 같은 자리를 다시 고른다. 실물에서는 commit이 하는 일이다.
        packs[pid].add(pl, code)

        name = f"seqbox_{n}"
        push([_obj(name, (pl.sx, pl.sy, pl.sz),
                   (x, y, z - pl.sz / 2.0), pl.yaw, CollisionObject.ADD)])
        placed_names.append(name)

        ok = best > 0.999
        bad += not ok
        print(f"{n:2d}. {code:10s} {kind:2s} P{pid} 층{pl.layer} "
              f"({x:.3f}, {y:.3f}, {z:.3f}) yaw {math.degrees(pl.yaw):3.0f}도  "
              f"{'ok' if ok else '실패'} {best*100:6.2f}%")

    push([_obj(nm, None, None, 0.0, CollisionObject.REMOVE) for nm in placed_names])

    print()
    if bad:
        print(f"{bad}단계에서 하강이 끝까지 안 풀린다. "
              "빈 팔레트로는 안 보이는 실패다. 자리 간격(pallet.slots.gap)이나 "
              "충돌 여유(padding)를 봐야 한다.")
        return 1
    print(f"{n}단계 전부 하강이 100% 풀린다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
