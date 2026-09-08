#!/usr/bin/env python3
"""적재 기록과 다음 자리 계산. 기획서 5.3.

    /pallet/next_slot (NextSlot)     다음 적재 자리
    /pallet/release   (ReleaseSlot)  반출. 디팔레타이징은 기록의 역순이다.
    /pallet/state     (PalletState)  두 팔레트의 점유 상태

자리 계산 자체는 cell.yaml의 격자 공식 하나다. 이 노드가 실제로 들고 있는
것은 '지금 어디에 무엇이 있는가'라는 기록이다. 카메라가 아니라 기록이
근거인 것이 기획서의 설계다("적재 좌표는 카메라가 아니라 계산에서 나온다").

반출을 기록의 역순으로 하는 이유는 2층 때문이다. 1층 박스를 먼저 꺼내면
그 위에 얹힌 2층이 무너진다. 마지막에 쌓은 것부터 꺼내면 항상 맨 위가 나온다.

기록은 파일로도 남긴다. 노드가 죽었다 살아나도 팔레트 위의 박스는 그대로
있기 때문이다. 실물에서 이 상태를 잃으면 사람이 팔레트를 비우고 다시
시작하는 수밖에 없다.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.msg import PalletState
from box_cell_msgs.srv import NextSlot, ReleaseSlot
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def down_quat(pose: Pose) -> None:
    """TCP 수직 하향. rpy (pi, 0, 0)."""
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0


class PalletManager(Node):
    def __init__(self) -> None:
        super().__init__("pallet_manager")

        self.declare_parameter("state_file", "/tmp/box_cell/pallet_state.json")
        # 기동 시 가득 찬 팔레트. 0이면 둘 다 빈 채로 시작한다.
        # -1이면 cell.yaml의 시나리오를 따른다 : infeed는 둘 다 비고,
        # circulate는 팔레트 1이 가득 찬 상태에서 출발한다.
        self.declare_parameter("initial_full", -1)
        self.declare_parameter("publish_rate", 2.0)
        # reset=true면 남아 있는 기록을 버리고 초기 배치로 시작한다.
        # 데모는 매번 같은 상태에서 출발해야 하므로 기본이 true다.
        # 노드만 죽었다 살아난 경우처럼 팔레트 위 실물이 그대로일 때는
        # false로 띄워 기록을 이어받는다.
        self.declare_parameter("reset", True)

        self.cell = CellGeometry()
        self.n = self.cell.slots_per_pallet
        self.state_file = Path(str(self.get_parameter("state_file").value))

        # occupied[pallet_id] = [코드 또는 None] * 8
        # 쌓은 순서도 함께 둔다. 반출이 역순이어야 하므로 순서가 곧 안전이다.
        self.slots: dict[int, list[str | None]] = {}
        self.order: dict[int, list[int]] = {}
        self._restore()

        latched = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PalletState, "/pallet/state", latched)
        self.create_service(NextSlot, "/pallet/next_slot", self._on_next)
        self.create_service(ReleaseSlot, "/pallet/release", self._on_release)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)

        self.get_logger().info(
            "pallet_manager 준비. "
            + ", ".join(f"P{p}={self._count(p)}/{self.n}" for p in self.cell.pallet_ids)
        )

    # ------------------------------------------------------------------ 기록
    def _restore(self) -> None:
        if bool(self.get_parameter("reset").value) and self.state_file.exists():
            self.state_file.unlink()
            self.get_logger().info(f"reset=true. 이전 적재 기록 {self.state_file}을 버린다.")
        if self.state_file.exists():
            try:
                data = json.loads(self.state_file.read_text())
                self.slots = {int(k): v for k, v in data["slots"].items()}
                self.order = {int(k): v for k, v in data["order"].items()}
                self.get_logger().info(f"적재 기록을 {self.state_file}에서 복구했다")
                return
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"기록 복구 실패, 새로 시작한다 : {exc}")

        full = int(self.get_parameter("initial_full").value)
        if full < 0:
            full = 0 if self.cell.scenario_mode == "infeed" else 1
        for pid in self.cell.pallet_ids:
            if pid == full:
                # box_spawner가 슬롯 0부터 순서대로 채운다. 기록도 같아야 한다.
                self.slots[pid] = [f"AXO-{i+1:04d}" for i in range(self.n)]
                self.order[pid] = list(range(self.n))
            else:
                self.slots[pid] = [None] * self.n
                self.order[pid] = []
        self._persist()

    def _persist(self) -> None:
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        self.state_file.write_text(
            json.dumps({"slots": self.slots, "order": self.order}, ensure_ascii=False, indent=2)
        )

    def _count(self, pid: int) -> int:
        return sum(1 for v in self.slots[pid] if v is not None)

    # ---------------------------------------------------------------- 서비스
    def _on_next(self, req: NextSlot.Request, res: NextSlot.Response) -> NextSlot.Response:
        pid = int(req.pallet_id)
        if pid not in self.slots:
            res.success = False
            self.get_logger().error(f"모르는 팔레트 {pid}")
            return res

        # 구석부터 하나씩. 인덱스 0이 (col 0, row 0), 즉 팔레트의 한 모서리다.
        # 0 -> 1 -> 2 -> 3 으로 1층을 채우고 그다음 2층으로 올라간다.
        # 낮은 층부터 채우는 것은 선택이 아니다. 2층을 먼저 쌓을 수는 없다.
        free = [i for i, v in enumerate(self.slots[pid]) if v is None]
        if not free:
            res.success = False
            self.get_logger().info(f"팔레트 {pid} 가득")
            return res
        index = min(free)

        slot = self.cell.slot(pid, index)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = slot.x, slot.y, slot.top_z
        down_quat(pose)

        res.success = True
        res.place_pose = pose
        res.layer = slot.layer
        res.index = index
        corner = "구석" if (slot.col, slot.row) == (0, 0) else ""
        self.get_logger().info(
            f"다음 자리 P{pid} #{index} (층{slot.layer}, col{slot.col} row{slot.row}{' ' + corner if corner else ''}) "
            f"world ({slot.x:.3f}, {slot.y:.3f}, {slot.top_z:.3f})"
        )
        return res

    def _on_release(self, req: ReleaseSlot.Request, res: ReleaseSlot.Response) -> ReleaseSlot.Response:
        pid = int(req.pallet_id)
        if pid not in self.slots:
            res.success = False
            return res

        if req.index >= 0:
            index = int(req.index)
            if self.slots[pid][index] is None:
                res.success = False
                self.get_logger().warn(f"P{pid} #{index}는 비어 있다")
                return res
        else:
            if not self.order[pid]:
                res.success = False
                self.get_logger().info(f"팔레트 {pid}가 비었다")
                return res
            # 기록의 역순. 마지막에 쌓은 것이 맨 위다.
            index = self.order[pid][-1]

        code = self.slots[pid][index] or ""
        slot = self.cell.slot(pid, index)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = slot.x, slot.y, slot.top_z
        down_quat(pose)

        self.slots[pid][index] = None
        if index in self.order[pid]:
            self.order[pid].remove(index)
        self._persist()

        res.success = True
        res.pick_pose = pose
        res.index = index
        res.code = code
        self.get_logger().info(f"반출 P{pid} #{index} ({code}), 남은 {self._count(pid)}개")
        self._publish()
        return res

    def commit(self, pid: int, index: int, code: str) -> None:
        """적재 완료를 기록한다. task_manager가 RECORD 단계에서 부른다."""
        self.slots[pid][index] = code
        if index not in self.order[pid]:
            self.order[pid].append(index)
        self._persist()
        self._publish()

    # ------------------------------------------------------------------ 발행
    def _publish(self) -> None:
        for pid in self.cell.pallet_ids:
            msg = PalletState()
            msg.pallet_id = pid
            msg.occupied = [v is not None for v in self.slots[pid]]
            msg.codes = [v or "" for v in self.slots[pid]]
            msg.count = self._count(pid)
            msg.full = msg.count == self.n
            self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PalletManager()

    # commit은 서비스가 아니라 토픽으로 받는다. task_manager가 적재 성공
    # 직후 한 줄 쏘면 되고, 응답을 기다릴 이유가 없다.
    from std_msgs.msg import String

    def on_commit(msg: String) -> None:
        try:
            pid, index, code = msg.data.split(",", 2)
            node.commit(int(pid), int(index), code)
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f"commit 형식 오류 '{msg.data}' : {exc}")

    node.create_subscription(String, "/pallet/commit", on_commit, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
