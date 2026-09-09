#!/usr/bin/env python3
"""픽앤플레이스 한 동작을 액션 하나로 감싼다.

기획서 5.1 : "motion_server가 MoveIt2를 감싸는 이유는 상위 로직이 계획
세부를 몰라도 되게 하기 위함이다. 접근, 파지, 이탈, 이송, 적재, 복귀를
한 액션으로 묶는다."

한 사이클의 아홉 구간 중 이 액션이 맡는 것.
  1  대기 -> 판독 위치 상공        고정 웨이포인트 PTP
  2  박스 상면 수직 하강           Cartesian 직선
  3  흡착                         밸브 제어만
  4  이탈                         Cartesian 직선
  5  판독 상공 -> 팔레트 상공       MoveIt2 (Pilz LIN, 실패 시 OMPL)
  6  적재 자리 하강                Cartesian + 충돌 검사
  7  해제                         밸브 제어만
  8  이탈                         Cartesian
  9  상공 -> 대기 자세             고정 웨이포인트 PTP

파지 중 박스는 AttachedCollisionObject로 그리퍼 링크에 붙인다(기획서 5.2).
붙이지 않으면 MoveIt이 손에 든 박스를 모르고 이웃을 스치는 궤적을 경고 없이
낸다. 2층을 쌓을 때 1층을 피해 들어가는 궤적이 자동으로 나오는 것도 이 덕이다.
touch_links에 그리퍼 링크를 넣어 흡착 접촉 자체는 충돌로 보지 않게 한다.
"""

from __future__ import annotations

import math
import time

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.action import PickPlace
from box_cell_msgs.srv import Vacuum
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from box_cell_motion.moveit_client import MoveItClient, down_pose


def above(pose: Pose, dz: float) -> Pose:
    out = Pose()
    out.position.x = pose.position.x
    out.position.y = pose.position.y
    out.position.z = pose.position.z + dz
    out.orientation = pose.orientation
    return out


class MotionServer(Node):
    def __init__(self) -> None:
        super().__init__("motion_server")

        self.declare_parameter("group", "fr5_arm")
        self.declare_parameter("tcp_link", "tcp_link")
        self.declare_parameter("attach_link", "es45_cup")
        self.declare_parameter("joints", ["j1", "j2", "j3", "j4", "j5", "j6"])
        self.declare_parameter("vacuum_dwell", 0.6)     # 컵이 붙기를 기다리는 시간
        self.declare_parameter("release_dwell", 0.4)
        # 손에 든 박스 모델을 수직으로만 줄이는 양(위아래 합).
        # 방금 들어 올린 밑면은 직전까지 무언가에 닿아 있던 면이라, 실제
        # 크기 그대로 두면 이탈 궤적이 시작점부터 충돌로 잡힌다. 이웃을
        # 피하는 데 쓰이는 x, y는 건드리지 않으므로 2층 적재는 그대로다.
        self.declare_parameter("held_box_z_clearance", 0.006)

        self.cell = CellGeometry()
        self.cb = ReentrantCallbackGroup()
        self.tuning = self.cell.tuning
        self._attached = ""     # 지금 그리퍼에 붙어 있는 충돌체 이름
        # 직선 구간이 실패했을 때 팔이 실제로 어디에 있었는지 남기려고 둔다.
        # compute_cartesian_path는 첫 웨이포인트가 아니라 현재 자세에서
        # 시작하므로, 실패 원인을 보려면 이 값이 있어야 한다.
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._js: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self.moveit = MoveItClient(
            self,
            group=str(self.get_parameter("group").value),
            tcp_link=str(self.get_parameter("tcp_link").value),
            joints=list(self.get_parameter("joints").value),
            callback_group=self.cb,
        )
        self.vacuum = self.create_client(Vacuum, "/gripper/vacuum", callback_group=self.cb)
        self.scene = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=self.cb
        )

        # 판독 3회째의 근접 재시도용. 기획서 5.4가 "판독 실패 시 C4 손목
        # 카메라로 근접 재시도"라고 한 그 동작이다.
        #
        # 전에는 task_manager가 팔을 두고 C4 셔터만 눌렀다. 그때 팔은 팔레트
        # 위 대기 자세였으므로 컨베이어가 화각에 없었고, 3회째는 언제나
        # 실패였다. 사실상 재시도가 두 번뿐이었던 셈이다.
        # 동작 중 표시. 늦게 뜨는 구독자도 마지막 값을 받아야 한다.
        self.busy_pub = self.create_publisher(
            Bool, "/motion/busy",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._set_busy(False)

        self.create_service(Trigger, "/motion/c4_read_pose", self._on_c4_pose,
                            callback_group=self.cb)
        self.create_service(Trigger, "/motion/home", self._on_home,
                            callback_group=self.cb)

        self.server = ActionServer(
            self,
            PickPlace,
            "/pick_place",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self.cb,
        )
        self.get_logger().info("motion_server 시작. /pick_place 대기.")

    # ------------------------------------------------------------------ 보조
    def _set_vacuum(self, on: bool) -> None:
        req = Vacuum.Request()
        req.on = on
        res = self.vacuum.call(req)
        # ES45는 잡았는지 되물을 수 없다. 응답을 성공 판정에 쓰지 않는다.
        detail = res.detail if res else "무응답"
        self.get_logger().info(f"진공 {'ON' if on else 'OFF'} : {detail}")
        time.sleep(
            float(self.get_parameter("vacuum_dwell" if on else "release_dwell").value)
        )

    def _on_c4_pose(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """C4가 판독 자리를 내려다보는 자리로 손목을 옮긴다.

        C4는 wrist3_link에 붙어 TCP 전방 150 지점으로 모아 겨눈다. 그래서
        TCP를 박스 상면 위 150에 세우면 광축이 라벨을 정면으로 문다.
        """
        x, y = self.cell.read_station_xy()
        z = self.cell.read_top_z + float(self.cell.data["cameras"]["c4_wrist"].get(
            "converge", 0.150))
        transit = float(self.tuning["transit_scale"])
        r = self.moveit.to_pose_via_joints(down_pose(x, y, z), vel=transit, acc=transit)
        res.success = bool(r.ok)
        res.message = r.detail if not r.ok else f"C4 판독 자리 (z={z:.3f})"
        self.get_logger().info(f"C4 근접 자세 : {res.message}")
        return res

    def _on_home(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        transit = float(self.tuning["transit_scale"])
        r = self.moveit.to_joints(self.cell.home_joints, vel=transit, acc=transit)
        res.success = bool(r.ok)
        res.message = r.detail if not r.ok else "대기 자세"
        return res

    def _on_js(self, msg: JointState) -> None:
        self._js = msg

    def _joints_now(self) -> str:
        if self._js is None:
            return "관절 없음"
        d = dict(zip(self._js.name, self._js.position))
        return " ".join(f"{j}={math.degrees(d[j]):+.1f}" for j in list(self.get_parameter("joints").value) if j in d)

    def _attach_box(self, name: str, pose: Pose, size=None) -> None:
        """든 박스를 그리퍼에 붙인다.

        치수는 상위 로직이 준다(MES에서 온 값). 규격이 여러 가지가 되면서
        여기서 cell.yaml의 값 하나를 쓸 수 없게 됐다. 안 주면 기본 규격이다.
        """
        if not name:
            return
        sx, sy, sz = tuple(size) if size and len(size) == 3 else self.cell.default_box_size
        link = str(self.get_parameter("attach_link").value)

        obj = CollisionObject()
        obj.header.frame_id = link
        obj.id = name
        obj.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        clear = float(self.get_parameter("held_box_z_clearance").value)
        prim.dimensions = [sx, sy, max(0.005, sz - clear)]
        local = Pose()
        # es45_cup 링크 원점에서 컵 접촉면(TCP)까지가 컵 길이만큼이고,
        # 박스 상면이 그 접촉면에 붙으므로 박스 중심은 거기서 반 높이 더 아래다.
        #
        # tcp_offset(0.2205)을 여기 쓰면 안 된다. 그 값은 wrist3_link 기준이다.
        # 컵 프레임에 그대로 넣으면 유령 박스가 상판 속에 생기고, 부착 직후의
        # 모든 계획이 충돌로 막힌다.
        # 상면은 컵 접촉면에 그대로 두고, 줄인 만큼은 밑면에서만 뺀다.
        cup_len = float(self.cell.data["tool"]["cup_length"])
        local.position.z = cup_len + sz / 2.0 - clear / 2.0
        local.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [local]

        att = AttachedCollisionObject()
        att.link_name = link
        att.object = obj
        # 흡착 접촉 자체는 충돌이 아니다. 넣지 않으면 파지 직후 계획이 전부 막힌다.
        att.touch_links = ["es45_cup", "es45_body", "es45_adapter", "wrist3_link"]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [att]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.scene.call(req)
        self._attached = name
        self.get_logger().info(f"{name}를 {link}에 부착")

    def _detach_box(self, name: str) -> None:
        # 붙인 적이 없으면 떼지 않는다. 붙지도 않은 것을 떼려 들면 MoveIt이
        # "Attached body not found"를 로그에 쏟아 내고, 진짜 오류가 묻힌다.
        if not name or self._attached != name:
            return
        self._attached = ""
        link = str(self.get_parameter("attach_link").value)
        att = AttachedCollisionObject()
        att.link_name = link
        att.object.id = name
        att.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [att]
        # 손에서 뗀 박스는 월드에서도 지운다. 놓은 자리의 충돌체는
        # scene_publisher가 pallet_manager의 기록을 보고 다시 만든다.
        world_remove = CollisionObject()
        world_remove.id = name
        world_remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [world_remove]

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.scene.call(req)
        self.get_logger().info(f"{name} 부착 해제")

    # ------------------------------------------------------------------ 실행
    def _execute(self, goal_handle):
        goal = goal_handle.request
        started = time.time()
        fb = PickPlace.Feedback()
        self._set_busy(True)
        try:
            return self._run(goal_handle, goal, started, fb)
        finally:
            self._set_busy(False)

    def _reachable_place(self, pose: Pose) -> Pose:
        """놓을 자세를 로봇이 갈 수 있는 쪽으로 고른다.

        상자를 90도 돌려 놓는 자리가 있다. 그 자세는 손목 마지막 축(j6)을
        90도 더 돌려야 나오는데, 팔레트 1(로봇에 가까운 쪽)에서는 그 값이
        j6의 물리 한계를 넘는다. 실측으로 팔레트 1의 yaw 90도 자리는 IK가
        아예 안 풀렸고(하강 0%), 팔레트 2의 같은 자리는 100% 풀렸다.

        그런데 직육면체를 180도 돌려 놓는 것은 같은 자리에 같은 모양으로
        놓는 것이다. 발자국이 그대로다. 그러니 원래 자세가 안 되면 180도
        돌린 자세로 놓으면 되고, 적재 결과는 조금도 달라지지 않는다.
        실물에서도 작업자가 하는 일이다.
        """
        if self.moveit.joints_for(pose) is not None:
            return pose
        alt = Pose()
        alt.position = pose.position
        # z축 180도 회전. (x, y) -> (-y, x). 하향 자세의 z, w는 0으로 둔다.
        alt.orientation.x = -pose.orientation.y
        alt.orientation.y = pose.orientation.x
        alt.orientation.z = pose.orientation.z
        alt.orientation.w = pose.orientation.w
        if self.moveit.joints_for(alt) is None:
            self.get_logger().warn("적재 자세를 두 방향 다 못 푼다. 원래 자세로 간다.")
            return pose
        self.get_logger().info("적재 자세를 180도 돌린다. 손목이 원래 방향으로는 안 돈다.")
        return alt

    def _set_busy(self, busy: bool) -> None:
        """동작 중임을 알린다. scene_publisher가 이걸 보고 씬 갱신을 미룬다.

        MoveIt은 계획한 뒤 실행 직전/도중에 씬이 바뀌면 궤적을 버린다
        (MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE). 이 셀에서는 그
        일이 정상적으로 일어난다. 적재를 기록하거나 반출로 한 칸을 빼는
        순간이 로봇이 아직 움직이는 순간과 겹치기 때문이다. 순환 시나리오
        에서는 매 사이클 그러므로 사이클이 통째로 안 돌았다.

        계획기를 바꾸거나 다시 푸는 것으로는 못 고친다. 다시 풀어도 그
        사이에 또 바뀌기 때문이다. 씬은 로봇이 멈춰 있을 때만 바꾼다.
        실물 셀도 그렇게 한다.
        """
        m = Bool()
        m.data = busy
        self.busy_pub.publish(m)

    def _run(self, goal_handle, goal, started, fb):

        def step(phase: str, progress: float) -> None:
            fb.phase = phase
            fb.progress = progress
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f"[{phase}] {progress*100:.0f}%")

        def fail(msg: str):
            """실패하고 빠져나가기 전에 손을 반드시 비운다.

            부착을 남긴 채 끝내면 다음 사이클은 유령 박스를 든 상태에서
            계획을 세운다. 그 박스는 어디에나 부딪히므로 이후 모든 계획이
            INVALID_MOTION_PLAN으로 죽는다. 한 번의 실패가 셀 전체를
            멈추게 하는 경로가 여기였다.

            손을 비우는 것만으로는 모자란다. 팔도 대기 자세로 되돌려야 한다.
            실패한 자리에 팔을 세워 두면 다음 사이클이 그 자세에서 시작하는데,
            그 자세는 검증한 자세 계열이 아닐 수 있다. FR5는 같은 TCP 지점에
            여러 관절 자세로 갈 수 있고(실측: 같은 점에 j1이 148도 다른
            자세로 도착했다), 검증하지 않은 자세에서는 바로 아래로 내리는
            직선이 안 풀린다. 실제로 한 번 실패한 뒤로 모든 사이클이
            "직선 경로가 43%만 풀렸다"로 죽었다.

            실물 셀도 같다. 오류가 나면 먼저 안전 자세로 돌아온 다음 다시
            시작한다. 멈춘 자리에서 이어서 하지 않는다.
            """
            self.get_logger().error(msg)
            try:
                self._detach_box(goal.pick_object)
                self._set_vacuum(False)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"실패 정리 중 예외 : {exc}")
            try:
                r = self.moveit.to_joints(
                    self.cell.home_joints,
                    vel=float(self.tuning["transit_scale"]),
                    acc=float(self.tuning["transit_scale"]),
                )
                if not r.ok:
                    self.get_logger().error(
                        f"복귀 실패 : {r.detail}. 다음 사이클이 검증되지 않은 "
                        "자세에서 시작한다."
                    )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"복귀 중 예외 : {exc}")
            goal_handle.abort()
            out = PickPlace.Result()
            out.success = False
            out.msg = msg
            out.duration_sec = time.time() - started
            return out

        def where() -> str:
            try:
                t = self._tf.lookup_transform(
                    "world", "tcp_link", rclpy.time.Time(), timeout=Duration(seconds=0.2)
                ).transform.translation
                return f"({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
            except Exception:  # noqa: BLE001
                return "TF 없음"

        approach = goal.approach_height or self.cell.approach_height
        retreat = float(self.tuning["retreat"])
        cart_speed = float(self.tuning["cartesian_speed"])
        transit = float(self.tuning["transit_scale"])

        # 2 : 접근점으로. 목표 상면 위 approach.
        step("APPROACH", 0.05)
        # 자세 목표가 아니라 관절 목표로 간다. 같은 지점이라도 어떤 팔꿈치
        # 자세로 도착하느냐에 따라 바로 아래로 내려가는 직선이 풀리기도 하고
        # 안 풀리기도 한다(moveit_client.joints_for 주석 참고).
        # 자세 계열은 joint_limits.yaml의 j2/j4 범위가 강제한다. 여기서
        # 시드로 몰아 주려고 했더니 오히려 하강이 0%로 풀리는 해가 나왔다.
        res = self.moveit.to_pose_via_joints(
            above(goal.pick_pose, approach), vel=transit, acc=transit
        )
        if not res.ok:
            return fail(f"접근점 계획 실패 : {res.detail}")

        # 2 : 수직 하강. 여기만 직선이다.
        step("DESCEND", 0.2)
        res = self.moveit.straight([above(goal.pick_pose, approach), goal.pick_pose], speed=cart_speed)
        if not res.ok:
            return fail(f"하강 실패 : {res.detail}")

        # 3 : 흡착
        step("GRASP", 0.3)
        self._set_vacuum(True)
        self._attach_box(goal.pick_object, goal.pick_pose, list(goal.box_size))

        # 4 : 이탈
        step("RETREAT", 0.4)
        res = self.moveit.straight([goal.pick_pose, above(goal.pick_pose, retreat)], speed=cart_speed)
        if not res.ok:
            return fail(f"이탈 실패 : {res.detail}")

        # 5 : 이송. 팔레트 상공까지.
        step("TRANSIT", 0.55)
        place = self._reachable_place(goal.place_pose)
        hover = above(place, approach)
        hover.position.z = max(hover.position.z, self.cell.pallet_hover_z)
        res = self.moveit.to_pose_via_joints(hover, vel=transit, acc=transit)
        if not res.ok:
            return fail(f"이송 실패 : {res.detail}")

        # 6 : 적재 자리 하강. 2층 진입은 수직 하강만 허용한다.
        step("PLACE", 0.75)
        res = self.moveit.straight(
            [hover, above(place, approach), place], speed=cart_speed
        )
        if not res.ok:
            wp = above(place, approach)
            self.get_logger().error(
                f"적재 하강 실패. TCP {where()}, 관절 [{self._joints_now()}], "
                f"상공 ({hover.position.x:.4f}, {hover.position.y:.4f}, {hover.position.z:.4f}), "
                f"접근 ({wp.position.x:.4f}, {wp.position.y:.4f}, {wp.position.z:.4f}), "
                f"적재 ({place.position.x:.4f}, "
                f"{place.position.y:.4f}, {place.position.z:.4f})"
            )
            return fail(f"적재 하강 실패 : {res.detail}")

        # 7 : 해제
        step("RELEASE", 0.85)
        self._detach_box(goal.pick_object)
        self._set_vacuum(False)

        # 8 : 이탈
        res = self.moveit.straight(
            [place, above(place, retreat)], speed=cart_speed
        )
        if not res.ok:
            return fail(f"적재 이탈 실패 : {res.detail}")

        # 9 : 대기 자세 복귀
        if goal.return_home:
            step("HOME", 0.95)
            res = self.moveit.to_joints(self.cell.home_joints, vel=transit, acc=transit)
            if not res.ok:
                return fail(f"대기 자세 복귀 실패 : {res.detail}")

        step("DONE", 1.0)
        goal_handle.succeed()
        out = PickPlace.Result()
        out.success = True
        out.msg = "ok"
        out.duration_sec = time.time() - started
        self.get_logger().info(f"픽앤플레이스 완료 {out.duration_sec:.1f} s")
        return out


def main() -> None:
    rclpy.init()
    node = MotionServer()
    if not node.moveit.wait(timeout=60.0):
        node.get_logger().error("move_group을 못 찾았다. MoveIt이 떠 있는지 확인할 것.")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
