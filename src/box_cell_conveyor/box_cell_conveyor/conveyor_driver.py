#!/usr/bin/env python3
"""컨베이어. 롤러를 굴리고, 정지 센서에 박스가 닿으면 세운다.

기획서 한 사이클의 1, 2단계다.
  1. 박스가 컨베이어로 진입
  2. 정지 센서 도달. 벨트 정지

이송 방식에 대하여. 처음에는 평평한 벨트판 위의 박스에 힘을 실어 밀었다
(ApplyLinkWrench). 그 길은 막혔다. gz에서 정지한 물체는 건 힘을 받지 않는다.
벨트에 서 있는 박스는 50 N에도 꿈쩍 않는데, 같은 20 N을 공중에서 떨어지는
박스에 걸면 1.4 m를 날아간다(실측). set_pose와 remove가 true만 돌려주고
아무 일도 하지 않던 것도 같은 뿌리로 보인다.

지금은 구동 롤러가 실제로 돈다. 접촉하는 몸체가 움직이므로 박스는 마찰로
실려 간다. 힘도 자세도 명령하지 않는다. 로봇 관절과 똑같이 속도 명령 하나로
돌고, 그 경로는 이미 잘 돌고 있다.

이 노드가 내는 것은 각속도 하나뿐이다.

    각속도 = 벨트 속도 / 롤러 반지름

롤러 전부가 같은 토픽을 듣는다. 앞 박스가 서 있으면 뒤 박스가 밀려와 쌓이는
축적 동작은 저절로 나온다. 실물 롤러 컨베이어와 같다.

정지 센서는 실물의 광전 센서다. 박스 중심이 stop_sensor_x를 지나면 롤러를
멈춘다. 실물도 관성으로 조금 더 가므로 여기서도 즉시 멈추지 않는다.

mode:=real이면 밑이 통째로 바뀐다. 롤러 각속도 대신 인버터 기동/정지 신호
하나를 내고(/io/conveyor_run), 위치 계산 대신 광전 센서 입력 하나를 받는다
(/io/photo_eye). 오히려 단순해진다. 위쪽이 보는 것(/belt/command 서비스와
/conveyor/* 상태 토픽)은 두 모드가 완전히 같다. 그래서 task_manager는 어느
쪽이 밑에 있는지 모른다.

두 토픽을 실제 I/O로 옮기는 것은 IO 게이트웨이의 일이고, 그 노드는 배선과
함께 온다. 그 전에도 토픽은 나가므로 상위 타이밍은 그대로 확인할 수 있다.
"""

from __future__ import annotations

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.msg import BoxPoseArray
from box_cell_msgs.srv import BeltCommand
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


class ConveyorDriver(Node):
    def __init__(self) -> None:
        super().__init__("conveyor_driver")

        self.declare_parameter("stop_speed", 0.01)      # m/s. 이하면 멈춘 것으로 본다
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("station_window", 0.035)  # 정지 센서 판정 폭
        # 기동 직후에는 롤러 쪽 구독이 아직 안 붙었을 수 있다.
        # 값이 같아도 이 횟수만큼은 다시 보낸다.
        self.declare_parameter("resend", 5)
        # sim | real. real이면 롤러 대신 인버터, /sim/boxes 대신 광전 센서.
        self.declare_parameter("mode", "sim")
        self.declare_parameter("run_topic", "/io/conveyor_run")
        self.declare_parameter("photo_eye_topic", "/io/photo_eye")
        # 실물에는 "어느 박스인지"가 없다. 광전 센서는 있다/없다만 안다.
        # 상위(task_manager)는 이름의 내용이 아니라 같은 이름이 계속 보이는지만
        # 보므로(집어 갔으면 자리가 빈다는 인터록), 고정 이름 하나면 된다.
        self.declare_parameter("real_station_name", "box_at_station")

        self.cell = CellGeometry()
        # 통짜 벨트다. TrackController에 표면 속도(m/s)를 그대로 준다.
        # 롤러 각속도로 환산할 필요가 없다.
        self.surface_speed = self.cell.belt_speed

        # 기동 상태를 RUN으로 둔다.
        #
        # 실물 축적 컨베이어도 계속 돈다. 정지 센서에 박스가 닿을 때만 선다.
        # 시뮬레이터에서는 이유가 하나 더 있다. gz는 처음부터 가만히 놓인
        # 물체를 재우는 것으로 보이는데, 박스는 스폰 후 0.2초면 안착한다.
        # 벨트가 10초 뒤에야 돌기 시작하면 그때는 이미 잠들어 롤러가 돌아도
        # 실리지 않는다. 처음부터 돌려 두면 박스가 움직이는 롤러 위에 내려앉아
        # 그대로 실려 간다.
        self.mode = BeltCommand.Request.RUN
        self.boxes: BoxPoseArray | None = None
        self.station_box = ""            # 정지 센서에 서 있는 박스
        self._commanded: float | None = None
        self._repeat = 0

        self.hw_mode = str(self.get_parameter("mode").value)
        self.photo_eye = False

        self.speed_pub = self.create_publisher(Float64, "/conveyor/belt_speed", 10)
        self.sensor_pub = self.create_publisher(Bool, "/conveyor/stop_sensor", 10)
        self.station_pub = self.create_publisher(String, "/conveyor/box_at_station", 10)
        self.running_pub = self.create_publisher(Bool, "/conveyor/running", 10)

        if self.hw_mode == "real":
            self.run_pub = self.create_publisher(
                Bool, str(self.get_parameter("run_topic").value), 10
            )
            self.create_subscription(
                Bool, str(self.get_parameter("photo_eye_topic").value), self._on_photo_eye, 10
            )
        else:
            self.run_pub = None
            self.create_subscription(BoxPoseArray, "/sim/boxes", self._on_boxes, 10)
        self.create_service(BeltCommand, "/belt/command", self._on_command)
        rate = float(self.get_parameter("control_rate").value)
        self.create_timer(1.0 / rate, self._control)

        if self.hw_mode == "real":
            self.get_logger().info(
                f"컨베이어 준비 (real). 기동/정지 {self.get_parameter('run_topic').value}, "
                f"광전 센서 {self.get_parameter('photo_eye_topic').value}"
            )
        else:
            self.get_logger().info(
                f"컨베이어 준비. 벨트 {self.cell.belt_speed} m/s "
                f"= 벨트 표면 {self.surface_speed:.2f} m/s, "
                f"정지 센서 x={self.cell.stop_sensor_x}"
            )

    # ------------------------------------------------------------------ 입력
    def _on_boxes(self, msg: BoxPoseArray) -> None:
        self.boxes = msg

    def _on_photo_eye(self, msg: Bool) -> None:
        """실물 정지 센서. 차광되면 True."""
        self.photo_eye = bool(msg.data)

    def _on_command(
        self, req: BeltCommand.Request, res: BeltCommand.Response
    ) -> BeltCommand.Response:
        self.mode = req.command
        if req.command == BeltCommand.Request.STOP:
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
    def _set_speed(self, scale: float) -> None:
        """벨트를 돌린다. scale은 0(정지) 또는 1(정상 속도)이다.

        TrackController는 마지막 명령을 max_command_age 동안 유지하므로
        매 주기 쏠 필요가 없다. 그 값을 크게 잡아 두었다(conveyor.urdf.xacro).
        """
        if self._commanded is not None and abs(scale - self._commanded) < 1e-6:
            if self._repeat <= 0:
                return
            self._repeat -= 1
        else:
            self._repeat = int(self.get_parameter("resend").value)
        if self.hw_mode == "real" and self.run_pub is not None:
            # 인버터는 속도를 모른다. 기동이냐 정지냐 둘뿐이다.
            # 벨트 속도는 인버터 파라미터로 잡아 두고 여기서는 건드리지 않는다.
            self.run_pub.publish(Bool(data=scale > 0.5))
        else:
            self.speed_pub.publish(Float64(data=float(scale * self.surface_speed)))
        self._commanded = scale

    def _at_station(self):
        """정지 센서 위치에 멈춰 선 박스."""
        if self.boxes is None:
            return None
        window = float(self.get_parameter("station_window").value)
        stop_speed = float(self.get_parameter("stop_speed").value)
        for b in self.boxes.boxes:
            if not b.on_belt or b.held:
                continue
            if (
                abs(b.pose.position.x - self.cell.stop_sensor_x) < window
                and abs(b.twist.linear.x) < stop_speed
            ):
                return b
        return None

    def _station(self) -> tuple[bool, str]:
        """정지 자리에 무엇이 서 있는가. (있는가, 이름)

        sim은 정답지에서 위치와 속도를 본다. real은 광전 센서 하나뿐이다.
        위쪽에는 두 모드가 같은 모양으로 나간다.
        """
        if self.hw_mode == "real":
            if self.photo_eye:
                return True, str(self.get_parameter("real_station_name").value)
            return False, ""
        b = self._at_station()
        return (b is not None, b.name if b is not None else "")

    def _passed_sensor(self) -> bool:
        """정지 센서를 지난 박스가 있는가. 지났으면 더 보내면 안 된다."""
        if self.hw_mode == "real":
            # 실물은 센서 하나뿐이라 "지났다"를 따로 알 수 없다. 차광 중이면
            # 그 자리에 있는 것이고, 그때는 어차피 보내지 않는다.
            return self.photo_eye
        if self.boxes is None:
            return False
        return any(
            b.on_belt and not b.held and b.pose.position.x >= self.cell.stop_sensor_x
            for b in self.boxes.boxes
        )

    def _control(self) -> None:
        running = self.mode in (BeltCommand.Request.RUN, BeltCommand.Request.FEED_ONE)

        occupied, name = self._station()
        if occupied:
            if self.station_box != name:
                self.station_box = name
                self.get_logger().info(f"정지 센서 도달 : {name}")
            self.station_pub.publish(String(data=name))
            self.sensor_pub.publish(Bool(data=True))
            if self.mode == BeltCommand.Request.FEED_ONE:
                self.mode = BeltCommand.Request.STOP
                running = False
        else:
            self.sensor_pub.publish(Bool(data=False))
            self.station_pub.publish(String(data=""))
            if self.station_box:
                self.station_box = ""

        # 센서를 이미 지난 박스가 있으면 더 보내지 않는다. 실물 광전 센서와 같다.
        if running and self._passed_sensor():
            running = False

        self.running_pub.publish(Bool(data=running))
        self._set_speed(1.0 if running else 0.0)


def main() -> None:
    rclpy.init()
    node = ConveyorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._set_speed(0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
