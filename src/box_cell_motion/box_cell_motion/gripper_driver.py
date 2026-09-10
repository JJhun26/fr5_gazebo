#!/usr/bin/env python3
"""석션 밸브. ES45-02-24V.

실물 ES45는 24 V I/O 장치다. 통신도 센서도 없다. 컨트롤러는 이 물건이
붙어 있는지조차 모르고, 잡았는지 되물을 방법도 없다. 그래서 이 노드가 내는
/gripper/vacuum 응답은 "명령을 냈다"는 뜻이지 "잡았다"는 뜻이 아니다.
상위 로직은 진공을 켜고 정해진 시간을 기다리는 것 말고 할 수 있는 게 없다.
시뮬레이션도 그 성질을 그대로 둔다. 여기서 성공 여부를 돌려주기 시작하면
실물에 옮겼을 때 로직이 통째로 무너진다.

sim 모드에서는 Gazebo의 DetachableJoint를 켜고 끈다. 어느 박스를 붙일지는
TCP에서 가장 가까운 놈을 골라 정한다. 이건 시뮬레이터만의 사정이다.
실물에서는 진공이 물리적으로 닿은 것을 잡을 뿐 고르지 않는다. 그래서
"가장 가까운 놈"을 고를 때도 실물의 흡착 조건과 같은 판정을 쓴다.
  - 컵 접촉면에서 박스 상면까지 grasp_range 안
  - 컵 축과 박스 상면 법선 사이 각도가 grasp_tilt 안
조건을 못 맞추면 아무것도 붙지 않는다. 실물에서 헛집는 것과 같다.

real 모드에서는 같은 서비스가 FAIRINO 컨트롤러의 디지털 출력을 때린다.
바꾸는 곳은 _apply_real 하나뿐이다.
"""

from __future__ import annotations

import math

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.msg import BoxPoseArray
from box_cell_msgs.srv import Vacuum
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener


def quat_to_z_axis(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """쿼터니언이 나타내는 회전의 +z 축을 world에서 본 벡터."""
    return (
        2 * (x * z + w * y),
        2 * (y * z - w * x),
        1 - 2 * (x * x + y * y),
    )


class GripperDriver(Node):
    def __init__(self) -> None:
        super().__init__("gripper_driver")

        self.declare_parameter("mode", "sim")           # sim | real
        self.declare_parameter("tcp_frame", "tcp_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("grasp_range", 0.020)    # 컵에서 박스 상면까지 m
        self.declare_parameter("grasp_radius", 0.030)   # 컵 축에서 박스 중심까지 m
        self.declare_parameter("grasp_tilt", 0.35)      # rad. 약 20도
        self.declare_parameter("do_suck", 0)            # real 모드 디지털 출력 번호
        self.declare_parameter("do_release", 1)
        # real 모드에서 밸브를 어디로 때릴 것인가.
        #   topic  std_msgs/Bool을 do_topic으로 낸다. 그 토픽을 실제 I/O로
        #          옮기는 것은 IO 게이트웨이 노드의 일이다(아직 없다).
        #   none   아무것도 하지 않고 실패를 돌려준다. 배선 전 안전값이다.
        self.declare_parameter("real_backend", "topic")
        self.declare_parameter("do_topic", "/io/tool_do")

        self.mode = str(self.get_parameter("mode").value)
        self.cell = CellGeometry()
        self.boxes: BoxPoseArray | None = None
        self.commanded = False
        self.attached = ""      # sim에서 실제로 붙은 박스. 실물에는 없는 정보다.

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # DetachableJoint 하나가 박스 하나를 맡는다. 토픽 이름은 URDF와 짝이다.
        self.attach_pub = {
            f"box_{i}": self.create_publisher(Empty, f"/suction/box_{i}/attach", 1)
            for i in range(1, self.cell.box_count + 1)
        }
        self.detach_pub = {
            f"box_{i}": self.create_publisher(Empty, f"/suction/box_{i}/detach", 1)
            for i in range(1, self.cell.box_count + 1)
        }

        self.state_pub = self.create_publisher(Bool, "/gripper/vacuum_state", 10)
        self.held_pub = self.create_publisher(String, "/sim/gripper/attached", 10)

        # real 모드의 출구. 여기에 Bool 하나가 나가고, 그것을 24 V 출력으로
        # 옮기는 것은 IO 게이트웨이의 일이다. 그 노드가 아직 없어도 이 토픽은
        # 나가므로, 배선 전에 상위 로직의 타이밍을 그대로 확인할 수 있다.
        self.do_pub = (
            self.create_publisher(Bool, str(self.get_parameter("do_topic").value), 10)
            if self.mode == "real"
            else None
        )

        # 새로 나타난 박스는 반드시 한 번 떼어 준다. 아래 _release_new 주석 참고.
        self._detached: set[str] = set()

        self.create_subscription(BoxPoseArray, "/sim/boxes", self._on_boxes, 10)
        self.create_service(Vacuum, "/gripper/vacuum", self._on_vacuum)
        self.create_timer(0.1, self._heartbeat)

        self.get_logger().info(f"gripper_driver 시작. mode={self.mode}")

    def _on_boxes(self, msg: BoxPoseArray) -> None:
        self.boxes = msg
        if self.mode == "sim":
            self._release_new(msg)

    def _release_new(self, msg: BoxPoseArray) -> None:
        """새로 나타난 박스를 한 번 떼어 놓는다.

        설치된 gz-sim의 DetachableJoint는 **로드 시점에 초기 부착을 한다.**
        문서에는 attach 요청이 오기 전에는 조인트를 만들지 않는다고 되어
        있지만, 실제로는 박스가 생기는 순간 wrist3_link에 용접된다.

        그 상태의 박스는 중력도 받지 않고, 50 N을 걸어도 안 움직이고, 도는
        롤러 위에서도 안 실린다. 로봇에 붙어 있으니 당연하다. 이 한 줄이
        없어서 컨베이어가 통째로 죽어 있었다. detach를 한 번 보내면 그
        자리에서 떨어져 정상으로 돌아온다.

        실물에는 대응물이 없는, 순전히 시뮬레이터 사정이다.
        """
        for b in msg.boxes:
            if b.name in self._detached or b.name == self.attached:
                continue
            pub = self.detach_pub.get(b.name)
            if pub is None:
                continue
            pub.publish(Empty())
            self._detached.add(b.name)
            self.get_logger().info(f"{b.name} 초기 부착 해제 (gz DetachableJoint 기본 동작 보정)")

    # ------------------------------------------------------------------ 서비스
    def _on_vacuum(self, req: Vacuum.Request, res: Vacuum.Response) -> Vacuum.Response:
        self.commanded = bool(req.on)
        if self.mode == "real":
            res.accepted, res.detail = self._apply_real(self.commanded)
            return res

        if self.commanded:
            target = self._pick_target()
            if target is None:
                # 실물에서 헛집은 것과 같다. 밸브는 열렸고, 아무것도 안 붙었다.
                res.accepted = True
                res.detail = "진공 ON. 흡착 조건을 만족하는 박스가 없다"
                self.get_logger().warn(res.detail)
                return res
            self.attach_pub[target].publish(Empty())
            self.attached = target
            self._detached.discard(target)
            res.accepted = True
            res.detail = f"진공 ON. {target} 흡착"
        else:
            if self.attached:
                self.detach_pub[self.attached].publish(Empty())
                res.detail = f"진공 OFF. {self.attached} 해제"
                self._detached.add(self.attached)
                self.attached = ""
            else:
                res.detail = "진공 OFF"
            res.accepted = True
        self.get_logger().info(res.detail)
        return res

    def _apply_real(self, on: bool) -> tuple[bool, str]:
        """ES45 밸브의 24 V 디지털 출력.

        이 노드는 여기서 토픽 하나를 낼 뿐이다. 그 토픽을 실제 출력으로
        옮기는 것(FAIRINO 컨트롤러의 툴 DO든, 별도 Modbus 모듈이든)은
        IO 게이트웨이 노드의 일이고, 그 노드는 배선과 함께 온다.

        왜 여기서 컨트롤러 서비스를 직접 부르지 않는가. 부르는 순간 이
        노드가 특정 컨트롤러 모델의 서비스 이름과 타입에 묶인다. 셀에서
        밸브를 어디에 물릴지는 아직 정해지지 않았다(기획서 E 항목).
        토픽 한 겹을 두면 그 결정이 이 파일 밖으로 나간다.

        되돌리는 값은 여전히 "명령을 냈다"는 뜻이다. ES45에는 피드백이
        없다. 그 성질은 sim/real 어느 쪽에서도 바꾸지 않는다.
        """
        do = self.get_parameter("do_suck" if on else "do_release").value
        backend = str(self.get_parameter("real_backend").value)
        if backend != "topic" or self.do_pub is None:
            self.get_logger().warn(
                f"real_backend={backend}. DO {do}를 때리지 않았다. "
                "배선이 붙으면 real_backend:=topic으로 두고 IO 게이트웨이를 띄운다."
            )
            return False, f"real 모드 출력 없음 (real_backend={backend})"

        self.do_pub.publish(Bool(data=bool(on)))
        detail = f"진공 {'ON' if on else 'OFF'} 명령 (DO {do})"
        self.get_logger().info(detail)
        return True, detail

    # ------------------------------------------------------------------ 흡착 판정
    def _pick_target(self) -> str | None:
        """실물 흡착 조건과 같은 기준으로 대상을 고른다."""
        if self.boxes is None:
            self.get_logger().warn("/sim/boxes가 아직 없다")
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("world_frame").value),
                str(self.get_parameter("tcp_frame").value),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:  # noqa: BLE001 - TF 예외 종류가 여러 개다
            self.get_logger().warn(f"TCP TF를 못 읽었다: {exc}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        cup = (t.x, t.y, t.z)
        # 컵의 흡착 방향은 tcp_link의 +z다.
        axis = quat_to_z_axis(q.x, q.y, q.z, q.w)

        rng = float(self.get_parameter("grasp_range").value)
        rad = float(self.get_parameter("grasp_radius").value)
        tilt = float(self.get_parameter("grasp_tilt").value)
        # 규격이 여러 가지라 반 높이가 박스마다 다르다. /sim/boxes가 실제
        # 치수를 함께 준다(정답지다. 실물에는 없는 정보이고, 이 노드는
        # 시뮬레이터 쪽 노드라 써도 된다).

        best: tuple[float, str] | None = None
        # 조건을 못 맞춘 후보 중 가장 가까웠던 것. 실패 원인을 남기는 데 쓴다.
        near: tuple[str, float, float, float] | None = None
        for b in self.boxes.boxes:
            if b.held:
                continue
            p = b.pose.position
            o = b.pose.orientation
            top_normal = quat_to_z_axis(o.x, o.y, o.z, o.w)
            half_h = (b.size.z / 2.0) if b.size.z > 1e-6 else (self.cell.box_height / 2.0)
            top = (
                p.x + top_normal[0] * half_h,
                p.y + top_normal[1] * half_h,
                p.z + top_normal[2] * half_h,
            )
            d = [top[i] - cup[i] for i in range(3)]
            along = sum(d[i] * axis[i] for i in range(3))       # 컵 축 방향 거리
            lateral = math.sqrt(max(0.0, sum(v * v for v in d) - along * along))
            # 컵 축과 상면 법선이 마주 보아야 한다 (내적이 -1에 가까울수록 정면)
            cosang = -sum(axis[i] * top_normal[i] for i in range(3))
            angle = math.acos(max(-1.0, min(1.0, cosang)))

            if near is None or abs(along) + lateral < abs(near[1]) + near[2]:
                near = (b.name, along, lateral, angle)
            if not (-0.005 <= along <= rng):
                continue
            if lateral > rad or angle > tilt:
                continue
            score = abs(along) + lateral
            if best is None or score < best[0]:
                best = (score, b.name)

        if best is None:
            # 왜 아무것도 안 붙었는지 남긴다. 실물에서도 헛집는 일은 있지만,
            # 시뮬레이터에서 원인을 모르면 고칠 수가 없다. 가장 가까웠던
            # 후보의 세 수치를 보면 무엇이 모자랐는지 바로 보인다.
            if near is None:
                self.get_logger().warn("흡착 대상 후보가 아예 없다 (/sim/boxes가 비었나)")
            else:
                name, along, lateral, angle = near
                why = []
                if not (-0.005 <= along <= rng):
                    why.append(f"축방향 {along*1000:+.1f} mm (허용 -5 ~ {rng*1000:.0f})")
                if lateral > rad:
                    why.append(f"옆으로 {lateral*1000:.1f} mm (허용 {rad*1000:.0f})")
                if angle > tilt:
                    why.append(f"기울기 {math.degrees(angle):.1f} 도 (허용 {math.degrees(tilt):.0f})")
                self.get_logger().warn(
                    f"흡착 조건 미달. 가장 가까운 {name} : " + ", ".join(why)
                )
            return None
        return best[1]

    def _heartbeat(self) -> None:
        self.state_pub.publish(Bool(data=self.commanded))
        self.held_pub.publish(String(data=self.attached))


def main() -> None:
    rclpy.init()
    node = GripperDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
