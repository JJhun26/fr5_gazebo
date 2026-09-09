#!/usr/bin/env python3
"""MoveIt Planning Scene 관리.

기획서 5.2 : "MoveIt2가 장애물을 아는 방법은 Planning Scene 하나뿐이다.
등록하지 않은 물체는 없는 것으로 취급된다."

고정 충돌체 : 상판(받침 개구를 피해 네 조각), 컨베이어, 기어박스, 팔레트 2장,
예외 통, 카메라 갠트리(기둥 2, 보, 팔 2), 카메라 본체 2대. 기동 시 한 번 등록.
기둥 두 개가 로봇 도달 고리 안에 있어서 이건 선택이 아니다.

동적 충돌체 : 팔레트에 쌓인 박스들. 놓을 때 ADD, 반출할 때 REMOVE.
여기서 중요한 것은 출처다. 시뮬레이터의 정답지(/sim/boxes)가 아니라
pallet_manager의 적재 기록(/pallet/state)을 보고 만든다. 실물에는 정답지가
없고 기록만 있기 때문이다. 이렇게 해야 2층을 쌓을 때 1층 박스를 피해
들어가는 궤적이 시뮬레이터와 실물에서 같은 이유로 나온다.
"""

from __future__ import annotations

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.msg import PalletState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def box_object(name: str, frame: str, size, pose_xyz, operation: int) -> CollisionObject:
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = name
    obj.operation = operation
    if operation == CollisionObject.ADD:
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(v) for v in size]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in pose_xyz)
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
    return obj


class ScenePublisher(Node):
    def __init__(self) -> None:
        super().__init__("scene_publisher")

        self.declare_parameter("frame", "world")
        # 적재 박스에 줄 여유. x, y에만 준다.
        # 수직으로 부풀리면 위아래로 맞닿은 두 박스가 서로 겹친 것으로 잡히고,
        # 2층을 집는 순간 시작 자세가 충돌이 되어 이탈 직선이 0%로 죽는다.
        # 쌓인 박스 충돌체를 얼마나 부풀릴지. 슬롯 간극(15)을 다 먹지 않게
        # 작게 잡는다. 4로 두었을 때 이웃이 둘인 자리에 못 내려놓았다.
        self.declare_parameter("box_padding", 0.002)
        self.declare_parameter("retry_sec", 1.0)

        self.cell = CellGeometry()
        self.frame = str(self.get_parameter("frame").value)
        self.cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.known_boxes: set[str] = set()
        self._statics_done = False

        # 로봇이 움직이는 동안에는 씬을 건드리지 않는다.
        #
        # MoveIt은 계획해 둔 궤적을 실행하는 동안 씬이 바뀌면 그 궤적을
        # 버린다(-4). 적재 기록과 반출은 하필 로봇이 아직 움직이는 순간에
        # 들어오므로, 그대로 반영하면 사이클이 자기 자신을 계속 중단시킨다.
        # 순환 시나리오에서 매 사이클 그렇게 죽었다.
        #
        # 그래서 바뀐 내용을 들고만 있다가 로봇이 멈추면 한 번에 반영한다.
        # 늦어도 상관없다. 다음 계획이 시작되기 전이면 충분하고, 그 사이에
        # 로봇은 어차피 멈춰 있다.
        self._busy = False
        self._deferred: PalletState | None = None
        self.create_subscription(PalletState, "/pallet/state", self._on_pallet, 10)
        self.create_subscription(
            Bool, "/motion/busy", self._on_busy,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_timer(float(self.get_parameter("retry_sec").value), self._ensure_statics)

    def _on_busy(self, msg: Bool) -> None:
        was, self._busy = self._busy, bool(msg.data)
        if was and not self._busy and self._deferred is not None:
            pending, self._deferred = self._deferred, None
            self._apply_pallet(pending)

    # ------------------------------------------------------------------ 공통
    def _apply(self, objects: list[CollisionObject]) -> bool:
        if not objects:
            return True
        if not self.cli.service_is_ready():
            return False
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.cli.call_async(req)
        return True

    def _ensure_statics(self) -> None:
        if self._statics_done:
            return
        if not self.cli.service_is_ready():
            self.get_logger().info("move_group의 apply_planning_scene을 기다린다", once=True)
            return
        objs = [
            box_object(o["name"], self.frame, o["size"], o["pose"], CollisionObject.ADD)
            for o in self.cell.static_obstacles()
        ]
        if self._apply(objs):
            self._statics_done = True
            names = ", ".join(o.id for o in objs)
            self.get_logger().info(f"고정 충돌체 {len(objs)}개 등록 : {names}")

    # ------------------------------------------------------------- 적재 박스
    def _on_pallet(self, msg: PalletState) -> None:
        """적재 기록이 바뀌면 그 팔레트의 박스 충돌체를 맞춘다.

        전에는 슬롯 번호로 자리를 다시 계산했다. 규격이 여러 가지가 되면서
        고정 격자가 사라졌고, 이제 pallet_manager가 실제 배치를 그대로
        실어 보낸다. 여기서 다시 계산하면 어긋난다.
        """
        if not self._statics_done:
            return
        if self._busy:
            # 마지막 것만 들고 있으면 된다. 중간 상태는 어차피 지나간다.
            self._deferred = msg
            return
        self._apply_pallet(msg)

    def _apply_pallet(self, msg: PalletState) -> None:
        pad = float(self.get_parameter("box_padding").value)
        want: dict[str, tuple] = {}
        for i, (code, pose, size) in enumerate(zip(msg.codes, msg.poses, msg.sizes)):
            name = f"stacked_p{msg.pallet_id}_s{i}"
            want[name] = (
                # z는 부풀리지 않는다. 위아래로 맞닿은 두 박스가 서로 겹친
                # 것으로 잡히면 2층을 집는 순간 시작 자세가 충돌이 된다.
                (size.x + pad, size.y + pad, size.z),
                (pose.position.x, pose.position.y, pose.position.z),
            )

        objs: list[CollisionObject] = []
        mine = {n for n in self.known_boxes if n.startswith(f"stacked_p{msg.pallet_id}_")}
        for name, (size, pose) in want.items():
            if name not in mine:
                objs.append(box_object(name, self.frame, size, pose, CollisionObject.ADD))
                self.known_boxes.add(name)
        for name in sorted(mine - set(want)):
            objs.append(box_object(name, self.frame, (0, 0, 0), (0, 0, 0),
                                   CollisionObject.REMOVE))
            self.known_boxes.discard(name)
        if objs and self._apply(objs):
            for o in objs:
                verb = "추가" if o.operation == CollisionObject.ADD else "제거"
                self.get_logger().info(f"충돌체 {verb} : {o.id}")


def main() -> None:
    rclpy.init()
    node = ScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
