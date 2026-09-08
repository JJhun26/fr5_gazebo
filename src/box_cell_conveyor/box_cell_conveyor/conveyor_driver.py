#!/usr/bin/env python3
"""컨베이어. 벨트를 돌리고, 정지 센서에 박스가 닿으면 세운다.

기획서 한 사이클의 1, 2단계다.
  1. 박스가 컨베이어로 진입
  2. 정지 센서 도달. 벨트 정지

Gazebo에는 벨트 표면 속도를 흉내 내는 시스템이 없다. 대신 벨트 위에 있는
박스에 힘을 실어 벨트 속도를 따라가게 한다. 텔레포트로 옮기지 않는 이유는
분명하다. 텔레포트한 물체는 속도도 접촉도 거짓이라, 석션이 잡는 순간과
가이드 레일에 닿는 순간이 실물과 달라진다. 그러면 시뮬레이터로 검증한 뜻이
없다.

힘은 두 항이다.
  마찰 보상 : 벨트가 정지해 있으므로 박스는 미끄러진다. 등속으로 가려면
              운동마찰(mu * m * g)만큼을 계속 밀어 줘야 한다.
  속도 제어 : 목표 속도와의 차이에 비례하는 항. 가감속을 만든다.

지속 렌치(/wrench/persistent)를 쓰지 않는다. gz-sim의 ApplyLinkWrench는
그것을 받을 때 목록에 push_back 하고 매 스텝 전부 더해서 적용한다. 대체가
아니라 누적이다. 제어 주기마다 쏘면 몇 초 만에 수백 N이 되어 박스가 날아간다
(실제로 7 m 밖으로 날아갔다). 그래서 단발 렌치(/wrench)를 쓴다. 큐에서 꺼내
한 스텝만 적용하고 버리므로 누적이 없다.

대신 단발은 제어 주기마다 한 스텝에만 걸린다. 물리가 1 kHz인데 제어가
200 Hz라면 다섯 스텝 중 하나에만 힘이 실린다. 그래서 평균으로 원하는 힘을
내려면 그 비(=5)만큼 키워 쏜다. physics_step은 cell.yaml에 있고 월드 SDF도
같은 값을 읽는다.

정지 센서는 실물의 광전 센서다. 박스 중심이 stop_sensor_x를 지나면 벨트를
멈춘다. 실물 벨트도 관성으로 조금 더 가므로 여기서도 즉시 멈추지 않는다.
"""

from __future__ import annotations

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.msg import BoxPoseArray
from box_cell_msgs.srv import BeltCommand
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench
from std_msgs.msg import Bool, String

GRAVITY = 9.80665


class ConveyorDriver(Node):
    def __init__(self) -> None:
        super().__init__("conveyor_driver")

        self.declare_parameter("world", "box_cell")
        self.declare_parameter("friction", 0.6)
        self.declare_parameter("speed_gain", 12.0)      # 속도 오차 대 가속도
        self.declare_parameter("max_force", 8.0)        # N. 실물 벨트의 이송력 상한
        self.declare_parameter("stop_speed", 0.01)      # m/s. 이하면 멈춘 것으로 본다
        self.declare_parameter("control_rate", 200.0)
        # 폭주 방지. 어떤 이유로든 벨트 속도의 이 배를 넘으면 밀기를 멈춘다.
        # 힘 계산이 잘못돼도 박스가 셀 밖으로 날아가지는 않게 한다.
        self.declare_parameter("runaway_factor", 3.0)

        self.cell = CellGeometry()
        world = self.get_parameter("world").value
        self.mass = float(self.cell.data["box"]["mass"])

        self.mode = BeltCommand.Request.STOP
        self.boxes: BoxPoseArray | None = None
        self.station_box = ""            # 정지 센서에 서 있는 박스
        self._pushing = ""               # 지금 힘을 주고 있는 박스

        self.wrench_pub = self.create_publisher(EntityWrench, f"/world/{world}/wrench", 10)
        self.clear_pub = self.create_publisher(Entity, f"/world/{world}/wrench/clear", 10)

        # 단발 렌치가 걸리는 스텝은 제어 주기당 하나뿐이다. 평균 힘을 맞추려면
        # 물리 스텝 대비 제어 주기의 비만큼 키워야 한다.
        rate = float(self.get_parameter("control_rate").value)
        self.duty = 1.0 / (self.cell.physics_step * rate)

        self.sensor_pub = self.create_publisher(Bool, "/conveyor/stop_sensor", 10)
        self.station_pub = self.create_publisher(String, "/conveyor/box_at_station", 10)
        self.running_pub = self.create_publisher(Bool, "/conveyor/running", 10)

        self.create_subscription(BoxPoseArray, "/sim/boxes", self._on_boxes, 10)
        self.create_service(BeltCommand, "/belt/command", self._on_command)
        self.create_timer(1.0 / rate, self._control)

        self.get_logger().info(
            f"컨베이어 준비. 벨트 속도 {self.cell.belt_speed} m/s, "
            f"정지 센서 x={self.cell.stop_sensor_x}, "
            f"제어 {rate:.0f} Hz, 물리 {1/self.cell.physics_step:.0f} Hz, 힘 배율 {self.duty:.1f}"
        )

    # ------------------------------------------------------------------ 입력
    def _on_boxes(self, msg: BoxPoseArray) -> None:
        self.boxes = msg

    def _on_command(
        self, req: BeltCommand.Request, res: BeltCommand.Response
    ) -> BeltCommand.Response:
        self.mode = req.command
        if req.command == BeltCommand.Request.STOP:
            self._release()
            res.detail = "벨트 정지"
        elif req.command == BeltCommand.Request.RUN:
            res.detail = "벨트 연속 운전"
        elif req.command == BeltCommand.Request.FEED_ONE:
            # 새 박스를 받으므로 이전에 서 있던 놈은 잊는다.
            self.station_box = ""
            res.detail = "박스 한 개 이송"
        else:
            res.accepted = False
            res.detail = f"모르는 명령 {req.command}"
            return res
        res.accepted = True
        self.get_logger().info(res.detail)
        return res

    # ------------------------------------------------------------------ 제어
    def _release(self) -> None:
        """밀기를 그만둔다.

        단발 렌치는 저절로 사라지므로 따로 걷을 것이 없다. 그래도 clear를
        한 번 보내는 이유는, 다른 도구나 이전 실행이 남긴 지속 렌치가
        있을 수 있어서다. 보내서 손해 볼 것이 없다.
        """
        if not self._pushing:
            return
        ent = Entity()
        ent.name = self._pushing
        ent.type = Entity.MODEL
        self.clear_pub.publish(ent)
        self._pushing = ""

    def _push(self, name: str, force_x: float) -> None:
        msg = EntityWrench()
        msg.entity.name = name
        msg.entity.type = Entity.MODEL
        w = Wrench()
        w.force.x = force_x
        msg.wrench = w
        self.wrench_pub.publish(msg)
        self._pushing = name

    def _candidate(self):
        """지금 밀어야 할 박스. 벨트 위에 있고 정지 센서를 아직 안 지난 놈."""
        if self.boxes is None:
            return None
        best = None
        for b in self.boxes.boxes:
            if not b.on_belt or b.held:
                continue
            if b.pose.position.x >= self.cell.stop_sensor_x:
                continue
            if best is None or b.pose.position.x < best.pose.position.x:
                best = b
        return best

    def _at_station(self):
        """정지 센서 위치에 멈춰 선 박스."""
        if self.boxes is None:
            return None
        for b in self.boxes.boxes:
            if not b.on_belt or b.held:
                continue
            if abs(b.pose.position.x - self.cell.stop_sensor_x) < 0.03 and abs(
                b.twist.linear.x
            ) < float(self.get_parameter("stop_speed").value):
                return b
        return None

    def _control(self) -> None:
        running = self.mode in (BeltCommand.Request.RUN, BeltCommand.Request.FEED_ONE)
        self.running_pub.publish(Bool(data=running))

        stopped = self._at_station()
        if stopped is not None:
            if self.station_box != stopped.name:
                self.station_box = stopped.name
                self.get_logger().info(
                    f"정지 센서 도달 : {stopped.name} at x={stopped.pose.position.x:.3f}"
                )
            self.station_pub.publish(String(data=stopped.name))
            self.sensor_pub.publish(Bool(data=True))
            if self.mode == BeltCommand.Request.FEED_ONE:
                # 한 개만 보내는 모드였으면 여기서 벨트를 세운다.
                self.mode = BeltCommand.Request.STOP
                self._release()
                return
        else:
            self.sensor_pub.publish(Bool(data=False))
            self.station_pub.publish(String(data=""))
            if self.station_box:
                self.station_box = ""

        if not running:
            self._release()
            return

        box = self._candidate()
        if box is None:
            self._release()
            return

        if self._pushing and self._pushing != box.name:
            self._release()

        mu = float(self.get_parameter("friction").value)
        gain = float(self.get_parameter("speed_gain").value)
        vmax = self.cell.belt_speed
        v = box.twist.linear.x

        # 폭주 방지. 실물 벨트는 박스를 벨트 속도 이상으로 밀지 않는다.
        if abs(v) > vmax * float(self.get_parameter("runaway_factor").value):
            self.get_logger().warn(
                f"{box.name} 속도 {v:.2f} m/s. 벨트 속도를 크게 넘었다. 밀기를 멈춘다.",
                throttle_duration_sec=2.0,
            )
            self._release()
            return

        # 마찰 보상 + 속도 오차 비례. 목표에 닿으면 마찰만 상쇄해 등속이 된다.
        friction_ff = mu * self.mass * GRAVITY
        force = friction_ff + self.mass * gain * (vmax - v)
        limit = float(self.get_parameter("max_force").value)
        force = max(-limit, min(limit, force))
        # 단발 렌치는 제어 주기당 한 스텝만 걸린다. 그만큼 키워 쏜다.
        self._push(box.name, float(force * self.duty))


def main() -> None:
    rclpy.init()
    node = ConveyorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
