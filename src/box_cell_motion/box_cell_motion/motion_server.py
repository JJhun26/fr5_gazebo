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
from shape_msgs.msg import SolidPrimitive

from box_cell_motion.moveit_client import MoveItClient


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

    def _attach_box(self, name: str, pose: Pose) -> None:
        """든 박스를 그리퍼에 붙인다."""
        if not name:
            return
        sx, sy, sz = self.cell.box_size
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
        local.position.z = cup_len + self.cell.box_height / 2.0 - clear / 2.0
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
            """
            self.get_logger().error(msg)
            try:
                self._detach_box(goal.pick_object)
                self._set_vacuum(False)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"실패 정리 중 예외 : {exc}")
            goal_handle.abort()
            out = PickPlace.Result()
            out.success = False
            out.msg = msg
            out.duration_sec = time.time() - started
            return out

        approach = goal.approach_height or self.cell.approach_height
        retreat = float(self.tuning["retreat"])
        cart_speed = float(self.tuning["cartesian_speed"])
        transit = float(self.tuning["transit_scale"])

        # 2 : 접근점으로. 목표 상면 위 approach.
        step("APPROACH", 0.05)
        res = self.moveit.to_pose(
            above(goal.pick_pose, approach), vel=transit, acc=transit, prefer_lin=False
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
        self._attach_box(goal.pick_object, goal.pick_pose)

        # 4 : 이탈
        step("RETREAT", 0.4)
        res = self.moveit.straight([goal.pick_pose, above(goal.pick_pose, retreat)], speed=cart_speed)
        if not res.ok:
            return fail(f"이탈 실패 : {res.detail}")

        # 5 : 이송. 팔레트 상공까지.
        step("TRANSIT", 0.55)
        hover = above(goal.place_pose, approach)
        hover.position.z = max(hover.position.z, self.cell.pallet_hover_z)
        res = self.moveit.to_pose(hover, vel=transit, acc=transit, prefer_lin=False)
        if not res.ok:
            return fail(f"이송 실패 : {res.detail}")

        # 6 : 적재 자리 하강. 2층 진입은 수직 하강만 허용한다.
        step("PLACE", 0.75)
        res = self.moveit.straight(
            [hover, above(goal.place_pose, approach), goal.place_pose], speed=cart_speed
        )
        if not res.ok:
            return fail(f"적재 하강 실패 : {res.detail}")

        # 7 : 해제
        step("RELEASE", 0.85)
        self._detach_box(goal.pick_object)
        self._set_vacuum(False)

        # 8 : 이탈
        res = self.moveit.straight(
            [goal.place_pose, above(goal.place_pose, retreat)], speed=cart_speed
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
