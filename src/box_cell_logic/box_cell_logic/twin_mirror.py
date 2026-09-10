#!/usr/bin/env python3
"""트윈 미러. 실물 셀을 시뮬레이터에 비춘다.

`twin_bridge`와 방향이 반대다. 그쪽은 셀 상태를 밖으로 내고, 이쪽은 밖(실물)
에서 들어온 상태를 시뮬레이터 안으로 넣는다. 둘이 있어야 트윈이 한 바퀴 돈다.

    실물 셀  --(/real/joint_states)-->  twin_mirror  -->  Gazebo 로봇이 따라 움직임
    실물 셀  --(/cell/state 등)------->  twin_bridge  -->  관제 화면/파일/HTTP

무엇을 하는가.
    실물 FR5의 관절 상태를 받아, 시뮬레이터 쪽 joint_trajectory_controller에
    아주 짧은 궤적(현재 위치 -> lookahead 뒤 실물 위치)을 계속 넣는다.
    시뮬레이터의 로봇은 실물이 가는 대로 따라간다.

왜 위치를 직접 쓰지 않는가.
    gz_ros2_control에는 관절을 텔레포트시키는 창구가 없다. 있어도 쓰면 안 된다.
    텔레포트하면 물리가 끊겨서(속도가 거짓이 되고 접촉이 튄다) 그 화면은
    실물과 닮았을 뿐 트윈이 아니다. 컨트롤러를 통해 밀어 넣으면 시뮬레이터
    로봇은 실물과 같은 방식으로, 같은 컨트롤러를 거쳐 움직인다.

방향이 한쪽인 것은 설계다.
    이 노드는 실물 쪽으로 아무것도 내지 않는다. 트윈이 실물을 조종하기
    시작하면 그것은 트윈이 아니라 검증되지 않은 제2의 상위 제어기다.
    운전 지시는 실물 셀의 task_manager 한 곳으로만 들어간다.

같이 쓸 때 주의.
    시뮬레이터 쪽 task_manager가 같이 돌면 둘이 같은 컨트롤러를 두고 다툰다.
    미러로 쓸 때는 시뮬레이터를 autostart:=false로 띄우고 상태 기계를
    IDLE에 세워 둔다. docs/digital_twin.md에 순서를 적어 두었다.

실물 쪽 토픽을 어떻게 여기로 가져오는가.
    같은 DDS 도메인이면 그냥 보인다(ROS_DOMAIN_ID를 맞춘다). 다른 망이면
    실물 쪽에서 domain_bridge나 ros2 daemon 없이 zenoh/DDS 라우터로 넘긴다.
    이 노드는 토픽 이름만 본다. 어느 경로로 왔는지는 모른다.
"""

from __future__ import annotations

import rclpy
from box_cell_msgs.msg import PalletState
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]


class TwinMirror(Node):
    def __init__(self) -> None:
        super().__init__("twin_mirror")

        # 실물 쪽 관절 상태. 실물 셀의 joint_state_broadcaster가 내는 것을
        # 그대로 받거나, 도메인이 다르면 /real 네임스페이스로 넘겨 받는다.
        self.declare_parameter("source_topic", "/real/joint_states")
        # 시뮬레이터 쪽 컨트롤러. 이름은 세 모드가 같다.
        self.declare_parameter("target_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("joints", JOINTS)
        # 몇 Hz로 밀어 넣을 것인가. 실물 컨트롤러가 100 Hz라 20이면 충분하다.
        self.declare_parameter("rate", 20.0)
        # 목표 시각을 지금보다 얼마 뒤로 둘 것인가. 0이면 컨트롤러가 매번
        # 궤적을 버린다. 너무 크면 시뮬 로봇이 실물보다 그만큼 늦게 따라간다.
        self.declare_parameter("lookahead", 0.15)
        # 이 시간 동안 실물 소식이 없으면 밀어 넣기를 멈춘다. 실물이 죽었는데
        # 시뮬레이터가 마지막 자세로 계속 명령받고 있으면 화면이 거짓말을 한다.
        self.declare_parameter("timeout", 1.0)
        # 적재 기록까지 비출 것인가. 켜면 시뮬레이터 쪽 pallet_manager를
        # 띄우지 않아야 한다. 둘이 같은 토픽에 쓴다.
        self.declare_parameter("mirror_pallet", False)
        self.declare_parameter("source_pallet_topic", "/real/pallet/state")

        self.joints = [str(j) for j in self.get_parameter("joints").value]
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.timeout = float(self.get_parameter("timeout").value)

        self._latest: dict[str, float] = {}
        self._stamp: float | None = None
        self._warned_missing = False

        self.create_subscription(
            JointState, str(self.get_parameter("source_topic").value), self._on_joints, 20
        )
        self.pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter("target_topic").value), 10
        )

        if bool(self.get_parameter("mirror_pallet").value):
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pallet_pub = self.create_publisher(PalletState, "/pallet/state", latched)
            self.create_subscription(
                PalletState,
                str(self.get_parameter("source_pallet_topic").value),
                self._on_pallet,
                latched,
            )

        rate = float(self.get_parameter("rate").value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"twin_mirror 준비. {self.get_parameter('source_topic').value} -> "
            f"{self.get_parameter('target_topic').value}, "
            f"lookahead {self.lookahead:.2f}s. 실물 쪽으로는 아무것도 내지 않는다."
        )

    # ---------------------------------------------------------------- 입력
    def _on_joints(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._latest[name] = float(pos)
        self._stamp = self.get_clock().now().nanoseconds * 1e-9

    def _on_pallet(self, msg: PalletState) -> None:
        self.pallet_pub.publish(msg)

    # ---------------------------------------------------------------- 출력
    def _tick(self) -> None:
        if self._stamp is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._stamp > self.timeout:
            # 실물 소식이 끊겼다. 마지막 자세를 계속 명령하지 않는다.
            return

        missing = [j for j in self.joints if j not in self._latest]
        if missing:
            if not self._warned_missing:
                self.get_logger().warn(
                    f"실물 관절 상태에 {missing}가 없다. 이름 규약이 다른지 볼 것."
                )
                self._warned_missing = True
            return
        self._warned_missing = False

        traj = JointTrajectory()
        traj.joint_names = list(self.joints)
        point = JointTrajectoryPoint()
        point.positions = [self._latest[j] for j in self.joints]
        # 속도는 넣지 않는다. 실물의 속도를 그대로 넣으면 시뮬 컨트롤러가
        # 그 속도로 끝점을 지나가려 해서 목표를 넘어간다.
        sec = int(self.lookahead)
        point.time_from_start = DurationMsg(
            sec=sec, nanosec=int((self.lookahead - sec) * 1e9)
        )
        traj.points = [point]
        self.pub.publish(traj)


def main() -> None:
    rclpy.init()
    node = TwinMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
