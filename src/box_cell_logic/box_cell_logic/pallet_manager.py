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
import math
from pathlib import Path

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_common.pallet_pack import Packer, Placement
from box_cell_msgs.msg import PalletState
from box_cell_msgs.srv import NextSlot, ReleaseSlot
from geometry_msgs.msg import Pose, Vector3
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def down_quat(pose: Pose, yaw: float = 0.0) -> None:
    """TCP 수직 하향에 yaw를 더한다. rpy (pi, 0, yaw).

    박스를 90도 돌려 놓아야 팔레트에 들어가는 경우가 있다. 규격이 여러
    가지가 되면서 생긴 일이다. 손목이 그만큼 더 돈다.
    """
    half = yaw / 2.0
    pose.orientation.x = math.cos(half)
    pose.orientation.y = math.sin(half)
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
        self.state_file = Path(str(self.get_parameter("state_file").value))

        # 팔레트마다 패커 하나. 규격이 여러 가지라 고정 격자가 없다.
        # 놓은 순서가 그대로 리스트 순서이고, 반출은 그 역순이다.
        # 순서가 곧 안전이다. 2층을 먼저 꺼낼 수는 없다.
        cfg = self.cell.data["pallet"]["slots"]
        self.pack_cfg = {
            "size": float(self.cell.data["pallet"]["size"]),
            "margin": float(cfg["margin"]),
            "gap": float(cfg["gap"]),
            "step": float(cfg["step"]),
            "max_layers": int(cfg["layers"]),
        }
        self.packs: dict[int, Packer] = {}
        # next_slot이 찾아 두고 commit이 확정한다. 그 사이에 잡아 두는 자리.
        self._pending: dict[int, Placement] = {}
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
            + ", ".join(f"P{p}={self._count(p)}개" for p in self.cell.pallet_ids)
        )

    # ------------------------------------------------------------------ 기록
    def _restore(self) -> None:
        if bool(self.get_parameter("reset").value) and self.state_file.exists():
            self.state_file.unlink()
            self.get_logger().info(f"reset=true. 이전 적재 기록 {self.state_file}을 버린다.")
        if self.state_file.exists():
            try:
                data = json.loads(self.state_file.read_text())
                self.packs = {
                    int(k): Packer.from_json(self.pack_cfg, v) for k, v in data.items()
                }
                self.get_logger().info(f"적재 기록을 {self.state_file}에서 복구했다")
                return
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"기록 복구 실패, 새로 시작한다 : {exc}")

        full = int(self.get_parameter("initial_full").value)
        if full < 0:
            full = 0 if self.cell.scenario_mode == "infeed" else 1
        for pid in self.cell.pallet_ids:
            self.packs[pid] = Packer(**self.pack_cfg)
            if pid == full:
                # 순환 시나리오의 출발 상태. 데모가 시작되기 전에 누군가
                # 쌓아 둔 것이므로 전부 기본 규격으로 본다. box_feeder도
                # 같은 자리에 같은 크기로 실물을 만든다.
                sx, sy, sz = self.cell.default_box_size
                for i in range(self.cell.box_count):
                    pl = self.packs[pid].find(sx, sy, sz)
                    if pl is None:
                        break
                    self.packs[pid].add(pl, f"AXO-{i+1:04d}")
        self._persist()

    def _persist(self) -> None:
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        self.state_file.write_text(
            json.dumps({str(k): v.to_json() for k, v in self.packs.items()},
                       ensure_ascii=False, indent=2)
        )

    def _count(self, pid: int) -> int:
        return len(self.packs[pid].placed)

    def _world_pose(self, pid: int, pl: Placement) -> Pose:
        """국소 배치를 world TCP 목표로. z는 박스 **상면**이다.

        흡착 TCP가 박스 상면을 잡으므로 여기서 나오는 z가 곧 TCP 높이다.
        """
        px, py = self.cell.pallet_center(pid)
        pose = Pose()
        pose.position.x = px + pl.x
        pose.position.y = py + pl.y
        pose.position.z = (
            self.cell.table_top + float(self.cell.data["pallet"]["thickness"])
            + pl.z_base + pl.sz
        )
        down_quat(pose, pl.yaw)
        return pose

    # ---------------------------------------------------------------- 서비스
    def _on_next(self, req: NextSlot.Request, res: NextSlot.Response) -> NextSlot.Response:
        pid = int(req.pallet_id)
        if pid not in self.packs:
            res.success = False
            res.reason = f"모르는 팔레트 {pid}"
            self.get_logger().error(res.reason)
            return res

        size = list(req.box_size)
        if len(size) != 3 or min(size) <= 0.0:
            size = list(self.cell.default_box_size)
            self.get_logger().warn("치수를 못 받았다. 기본 규격으로 자리를 찾는다.")

        pl = self.packs[pid].find(size[0], size[1], size[2])
        if pl is None:
            res.success = False
            res.reason = "자리 없음"
            self.get_logger().info(
                f"팔레트 {pid}에 {size[0]*1000:.0f}x{size[1]*1000:.0f}"
                f"x{size[2]*1000:.0f} 박스를 놓을 자리가 없다"
            )
            return res

        res.success = True
        res.place_pose = self._world_pose(pid, pl)
        res.layer = pl.layer
        res.index = len(self.packs[pid].placed)   # 이 박스가 몇 번째로 놓이는가
        res.yaw = pl.yaw
        corner = abs(pl.x + self.packs[pid].half - pl.sx / 2) < 1e-6 and \
                 abs(pl.y + self.packs[pid].half - pl.sy / 2) < 1e-6
        self.get_logger().info(
            f"다음 자리 P{pid} #{res.index} (층{pl.layer}"
            f"{', 구석' if corner else ''}) "
            f"world ({res.place_pose.position.x:.3f}, {res.place_pose.position.y:.3f}, "
            f"{res.place_pose.position.z:.3f}), yaw {math.degrees(pl.yaw):.0f}도"
        )
        # 아직 기록하지 않는다. 실제로 놓였다는 확인은 commit이 한다.
        self._pending[pid] = pl
        return res

    def _on_release(self, req: ReleaseSlot.Request, res: ReleaseSlot.Response) -> ReleaseSlot.Response:
        """맨 위 박스를 하나 내린다.

        index를 지정해도 맨 위가 아니면 거절한다. 규격이 여러 가지가 되면서
        "슬롯 3번"이라는 말이 사라졌고, 남은 것은 쌓은 순서뿐이다. 중간을
        빼면 위가 무너진다. 실물에서도 그렇게 꺼내지 않는다.
        """
        pid = int(req.pallet_id)
        if pid not in self.packs:
            res.success = False
            return res
        pack = self.packs[pid]
        if not pack.placed:
            res.success = False
            self.get_logger().info(f"팔레트 {pid}가 비었다")
            return res

        top = len(pack.placed) - 1
        if req.index >= 0 and int(req.index) != top:
            res.success = False
            self.get_logger().warn(
                f"P{pid} #{req.index}는 맨 위가 아니다(맨 위 #{top}). 반출은 역순이어야 한다."
            )
            return res

        pl = pack.pop_last()
        self._persist()

        res.success = True
        res.pick_pose = self._world_pose(pid, pl)
        res.index = top
        res.code = pl.code
        self.get_logger().info(f"반출 P{pid} #{top} ({pl.code}), 남은 {self._count(pid)}개")
        self._publish()
        return res

    def commit(self, pid: int, index: int, code: str) -> None:
        """적재 완료를 기록한다. task_manager가 RECORD 단계에서 부른다.

        자리는 next_slot이 이미 찾아 두었다. 여기서 다시 찾으면 그 사이
        다른 요청이 끼어들었을 때 어긋난다.
        """
        pl = self._pending.pop(pid, None)
        if pl is None:
            self.get_logger().warn(f"P{pid} 적재 기록 요청이 왔는데 잡아 둔 자리가 없다")
            return
        self.packs[pid].add(pl, code)
        self._persist()
        self._publish()

    # ------------------------------------------------------------------ 발행
    def _publish(self) -> None:
        for pid in self.cell.pallet_ids:
            pack = self.packs[pid]
            msg = PalletState()
            msg.pallet_id = pid
            msg.count = len(pack.placed)
            # 더 놓을 자리가 있는지는 기본 규격 하나를 넣어 보고 판단한다.
            sx, sy, sz = self.cell.default_box_size
            msg.full = pack.find(sx, sy, sz) is None
            for pl in pack.placed:
                msg.codes.append(pl.code)
                pose = self._world_pose(pid, pl)
                # 발행하는 것은 박스 **중심**이다. place_pose는 상면이었다.
                pose.position.z -= pl.sz / 2.0
                msg.poses.append(pose)
                v = Vector3()
                v.x, v.y, v.z = pl.sx, pl.sy, pl.sz
                msg.sizes.append(v)
                msg.layers.append(pl.layer)
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
