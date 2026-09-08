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
        self.declare_parameter("box_padding", 0.004)
        self.declare_parameter("retry_sec", 1.0)

        self.cell = CellGeometry()
        self.frame = str(self.get_parameter("frame").value)
        self.cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.known_boxes: set[str] = set()
        self._statics_done = False

        self.create_subscription(PalletState, "/pallet/state", self._on_pallet, 10)
        self.create_timer(float(self.get_parameter("retry_sec").value), self._ensure_statics)

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
        """적재 기록이 바뀌면 그 팔레트의 박스 충돌체를 맞춘다."""
        if not self._statics_done:
            return
        sx, sy, sz = self.cell.box_size
        pad = float(self.get_parameter("box_padding").value)
        objs: list[CollisionObject] = []

        for index, occupied in enumerate(msg.occupied):
            name = f"stacked_p{msg.pallet_id}_s{index}"
            if occupied and name not in self.known_boxes:
                slot = self.cell.slot(msg.pallet_id, index)
                objs.append(
                    box_object(
                        name,
                        self.frame,
                        (sx + pad, sy + pad, sz),   # z는 부풀리지 않는다
                        (slot.x, slot.y, slot.center_z),
                        CollisionObject.ADD,
                    )
                )
                self.known_boxes.add(name)
            elif not occupied and name in self.known_boxes:
                objs.append(box_object(name, self.frame, None, None, CollisionObject.REMOVE))
                self.known_boxes.discard(name)

        if objs and self._apply(objs):
            added = [o.id for o in objs if o.operation == CollisionObject.ADD]
            removed = [o.id for o in objs if o.operation == CollisionObject.REMOVE]
            if added:
                self.get_logger().info(f"충돌체 추가 : {', '.join(added)}")
            if removed:
                self.get_logger().info(f"충돌체 제거 : {', '.join(removed)}")


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
