"""MoveIt2를 액션과 서비스로 직접 부른다.

C++ MoveGroupInterface 대신 /move_action, /compute_cartesian_path,
/execute_trajectory를 그대로 쓴다. 장황하지만 버전에 덜 흔들리고, 어떤
플래너가 무슨 요청으로 불렸는지 로그에 그대로 남는다. 데모 전날 궤적이
달라졌을 때 원인을 찾을 수 있어야 한다.

기획서 5.1의 계획 방식을 그대로 옮긴다.
  고정 웨이포인트    Pilz PTP
  직선 이송          Pilz LIN
  수직 하강/이탈     compute_cartesian_path
  Pilz가 실패하면    OMPL(RRTConnect)로 한 번 더
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


# MoveItErrorCodes의 흔한 값들. 숫자만 남기면 데모 전날 로그를 못 읽는다.
MOVEIT_ERROR = {
    1: "SUCCESS",
    -1: "FAILURE",
    -2: "PLANNING_FAILED",
    -3: "INVALID_MOTION_PLAN",
    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -5: "CONTROL_FAILED",
    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -7: "TIMED_OUT",
    -10: "START_STATE_IN_COLLISION",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -19: "NO_IK_SOLUTION",
    -31: "INVALID_GOAL_CONSTRAINTS",
}


@dataclass
class PlanResult:
    ok: bool
    detail: str
    fraction: float = 1.0


def pose_from(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


def down_pose(x: float, y: float, z: float, yaw: float = 0.0) -> Pose:
    """TCP 수직 하향. 석션 컵의 +z가 아래를 본다.

    쿼터니언은 rpy (pi, 0, yaw)를 직접 계산한 것이다. tf_transformations
    의존을 하나 줄인다.
    """
    half = yaw / 2.0
    # Rz(yaw) * Rx(pi)
    return pose_from(x, y, z, math.cos(half), math.sin(half), 0.0, 0.0)


class MoveItClient:
    def __init__(
        self,
        node: Node,
        group: str,
        tcp_link: str,
        joints: list[str],
        callback_group=None,
    ) -> None:
        self.node = node
        self.group = group
        self.tcp_link = tcp_link
        self.joints = joints
        self.base_frame = "world"

        # 이 클래스의 호출은 전부 동기다. 액션 결과를 기다리는 동안에도
        # 실행기가 다른 콜백을 돌려야 하므로, 부르는 쪽이 반드시
        # MultiThreadedExecutor + ReentrantCallbackGroup을 써야 한다.
        self.move = ActionClient(node, MoveGroup, "/move_action", callback_group=callback_group)
        self.execute = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory", callback_group=callback_group
        )
        self.cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=callback_group
        )

    def wait(self, timeout: float = 30.0) -> bool:
        ok = self.move.wait_for_server(timeout_sec=timeout)
        ok = self.execute.wait_for_server(timeout_sec=timeout) and ok
        ok = self.cartesian.wait_for_service(timeout_sec=timeout) and ok
        return ok

    # ------------------------------------------------------------------ 계획
    def _request(
        self,
        pipeline: str,
        planner: str,
        vel: float,
        acc: float,
        attempts: int = 4,
    ) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = self.group
        req.pipeline_id = pipeline
        req.planner_id = planner
        req.num_planning_attempts = attempts
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        return req

    def _options(self) -> PlanningOptions:
        opt = PlanningOptions()
        opt.plan_only = False
        opt.replan = True
        opt.replan_attempts = 2
        return opt

    def _send_move(self, req: MotionPlanRequest) -> PlanResult:
        """계획하고 바로 실행한다.

        rclpy의 동기 send_goal()은 goal handle이 아니라 GetResult 응답을
        돌려준다(.status와 .result). 실패 이유는 status가 아니라
        result.error_code에 담기므로 그쪽을 읽는다. MoveIt은 계획에 실패하면
        goal을 ABORTED로 끝내면서 error_code에 사유를 넣어 준다.
        """
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = self._options()
        try:
            response = self.move.send_goal(goal)
        except Exception as exc:  # noqa: BLE001 - 서버 거부/통신 오류
            return PlanResult(False, f"move_action 호출 실패 : {exc}")
        if response is None:
            return PlanResult(False, "move_group 무응답")
        code = response.result.error_code.val
        if code == 1:
            return PlanResult(True, "ok")
        return PlanResult(False, f"{MOVEIT_ERROR.get(code, 'MoveItErrorCode')} ({code})")

    def to_joints(
        self, values: list[float], vel: float = 0.35, acc: float = 0.35
    ) -> PlanResult:
        """고정 웨이포인트. 기획서 1, 9 구간. Pilz PTP."""
        con = Constraints()
        for name, v in zip(self.joints, values, strict=True):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(v)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            con.joint_constraints.append(jc)

        for pipeline, planner in (("pilz_industrial_motion_planner", "PTP"), ("ompl", "RRTConnectkConfigDefault")):
            req = self._request(pipeline, planner, vel, acc)
            req.goal_constraints = [con]
            res = self._send_move(req)
            if res.ok:
                return PlanResult(True, f"{pipeline}/{planner}")
            self.node.get_logger().warn(f"{pipeline}/{planner} 실패 : {res.detail}")
        return PlanResult(False, "관절 목표 계획 실패")

    def to_pose(
        self,
        pose: Pose,
        vel: float = 0.35,
        acc: float = 0.35,
        pos_tol: float = 0.002,
        ang_tol: float = 0.02,
        prefer_lin: bool = True,
    ) -> PlanResult:
        """자세 목표.

        prefer_lin=False가 기본에 가깝게 쓰인다. 작업 공간을 가로지르는
        이송 구간(1, 5, 9)은 기획서 5.1이 '고정 웨이포인트 PTP'로 정하고
        있고, 실제로도 그래야 한다. 그 거리를 직선으로 끌면 특이점 근처에서
        관절 가속도가 튀어 Pilz가 계획을 포기한다
        ("Joint acceleration limit of j1 violated").

        직선이 필요한 곳은 박스 상면으로의 짧은 수직 하강과 이탈뿐이고,
        그쪽은 straight()가 compute_cartesian_path로 따로 맡는다.

        어느 쪽이든 마지막에는 OMPL이 받는다. 컨베이어 프레임이나 카메라
        지주에 걸려 Pilz가 못 풀 때를 위한 것이다.
        """
        target = PoseStamped()
        target.header.frame_id = self.base_frame
        target.pose = pose

        con = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tcp_link
        pc.weight = 1.0
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [pos_tol]
        pc.constraint_region.primitives = [region]
        pc.constraint_region.primitive_poses = [pose]
        con.position_constraints = [pc]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.tcp_link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = ang_tol
        oc.absolute_y_axis_tolerance = ang_tol
        oc.absolute_z_axis_tolerance = ang_tol
        oc.weight = 1.0
        con.orientation_constraints = [oc]

        order = [("pilz_industrial_motion_planner", "LIN" if prefer_lin else "PTP")]
        order.append(("pilz_industrial_motion_planner", "PTP"))
        order.append(("ompl", "RRTConnectkConfigDefault"))
        for pipeline, planner in order:
            req = self._request(pipeline, planner, vel, acc)
            req.goal_constraints = [con]
            res = self._send_move(req)
            if res.ok:
                return PlanResult(True, f"{pipeline}/{planner}")
            self.node.get_logger().warn(f"{pipeline}/{planner} 실패 : {res.detail}")
        return PlanResult(False, "자세 목표 계획 실패")

    # -------------------------------------------------------------- Cartesian
    def straight(
        self,
        waypoints: list[Pose],
        speed: float = 0.10,
        step: float = 0.002,
        min_fraction: float = 0.99,
    ) -> PlanResult:
        """직선 구간. 기획서 2, 4, 6, 8 구간.

        min_fraction을 0.99로 두는 이유는 분명하다. 절반만 내려간 궤적을
        실행하면 컵이 박스에 닿지 않은 채 진공을 켜게 된다. ES45는 잡았는지
        되물을 수 없으므로 그대로 빈손으로 팔레트까지 간다.
        """
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.group_name = self.group
        req.link_name = self.tcp_link
        req.waypoints = waypoints
        req.max_step = step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.start_state = RobotState()
        req.start_state.is_diff = True

        res = self.cartesian.call(req)
        if res is None:
            return PlanResult(False, "compute_cartesian_path 무응답", 0.0)
        if res.fraction < min_fraction:
            return PlanResult(
                False,
                f"직선 경로가 {res.fraction*100:.0f}%만 풀렸다 (충돌 또는 도달 한계)",
                res.fraction,
            )

        # 시간 재조정 : 속도 배율 대신 실제 직선 속도(m/s)로 잡는다.
        traj = self._retime(res.solution, waypoints, speed)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        try:
            response = self.execute.send_goal(goal)
        except Exception as exc:  # noqa: BLE001
            return PlanResult(False, f"execute_trajectory 호출 실패 : {exc}", res.fraction)
        if response is None:
            return PlanResult(False, "execute_trajectory 무응답", res.fraction)
        code = response.result.error_code.val
        if code == 1:
            return PlanResult(True, f"직선 {res.fraction*100:.0f}%", res.fraction)
        return PlanResult(
            False, f"실행 실패 {MOVEIT_ERROR.get(code, 'MoveItErrorCode')} ({code})", res.fraction
        )

    def _retime(self, traj, waypoints: list[Pose], speed: float):
        """직선 길이와 목표 속도로 궤적 시간을 다시 깐다.

        compute_cartesian_path는 시간을 아주 촘촘히 넣어 돌려준다. 그대로
        실행하면 하강이 순식간에 끝나 흡착 접촉이 튄다. 실물 석션은 컵이
        박스 상면에 눌리는 시간이 있어야 붙는다.
        """
        points = traj.joint_trajectory.points
        if len(points) < 2:
            return traj
        length = 0.0
        for a, b in zip(waypoints, waypoints[1:], strict=False):
            length += math.dist(
                (a.position.x, a.position.y, a.position.z),
                (b.position.x, b.position.y, b.position.z),
            )
        if length <= 1e-6:
            return traj
        total = max(0.4, length / max(speed, 1e-3))
        n = len(points) - 1
        for i, pt in enumerate(points):
            t = total * i / n
            pt.time_from_start = Duration(seconds=t).to_msg()
            # 속도/가속도는 컨트롤러가 다시 보간한다. 남겨 두면 시간과 안 맞는다.
            pt.velocities = []
            pt.accelerations = []
        return traj
