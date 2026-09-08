#!/usr/bin/env python3
"""택배박스를 만들고, 컨베이어로 들여보내고, 그 실제 상태를 알려 준다.

시나리오 (cell.yaml의 scenario.mode)

  infeed    박스가 컨베이어를 타고 들어온다. 로봇이 판독 위치에서 흡착으로
            집어 팔레트에 구석부터 하나씩 쌓는다. 팔레트가 차면 반출한 것으로
            보고 비운 뒤 다음 로트를 받는다.

  circulate 기획서 "순환"의 두 팔레트 방식. 기동 시 팔레트 1을 채워 두고,
            로봇이 하나씩 꺼내 벨트에 되올린다.

실제 셀에서는 상류 컨베이어가 박스를 보내 온다. 시뮬레이터는 그 상류를
만들지 않는다. 대기 박스를 셀 밖 바닥에 세워 두었다가 벨트 입구로 하나씩
옮긴다. 셀 경계 밖의 일은 셀의 관심사가 아니다.

옮기는 방법은 모델 삭제/재생성이 아니라 자세 이동이다. 이유가 있다. 석션의
DetachableJoint 플러그인이 box_1 .. box_8이라는 이름에 묶여 있어서, 모델을
지웠다 다시 만들면 그 연결을 다시 잡아야 한다. 이름과 실체를 그대로 두고
자리만 옮기면 그 문제가 아예 생기지 않는다.

/sim/boxes는 정답지다. 실물에는 없다. 읽어도 되는 곳은 셋뿐이다 :
conveyor_driver(힘을 실을 대상), gripper_driver(석션이 붙일 대상),
Dry Run 채점. 판독과 적재 로직은 이것을 보지 않는다.
"""

from __future__ import annotations

import json
import math
import subprocess
import tempfile
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.msg import BoxPose, BoxPoseArray
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

BOX_SDF = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <link name="box_link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface>
          <friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>
          <contact><ode><kp>1e6</kp><kd>100</kd></ode></contact>
        </surface>
      </collision>
      <visual name="body">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>0.66 0.53 0.37 1</ambient>
          <diffuse>0.77 0.62 0.43 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr><metal>
            <albedo_map>{cardboard}</albedo_map>
            <roughness>0.95</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
      <visual name="label">
        <!-- 상면보다 0.1 mm 띄운다. 같은 평면에 겹치면 z-fighting으로
             라벨이 지직거리고, 그 상태로는 QR이 안 읽힌다. -->
        <pose>0 0 {label_z} 0 0 0</pose>
        <geometry><mesh><uri>{label_mesh}</uri></mesh></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <!-- 무광. 광택이면 조명 정반사가 QR 위에 앉아 디코드를 지운다.
                 실물 택배 라벨지도 무광이다. -->
            <albedo_map>{label_map}</albedo_map>
            <roughness>0.95</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>

    <!-- 이 박스의 자세를 이름과 함께 낸다.
         월드의 dynamic_pose/info를 브리지하면 TFMessage의 child_frame_id가
         비어서 어느 박스인지 알 수 없다. gz Pose의 name을 변환기가 보지
         않기 때문이다. PosePublisher는 헤더에 프레임 이름을 채워 준다. -->
    <plugin filename="gz-sim-pose-publisher-system"
            name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <static_publisher>false</static_publisher>
      <update_frequency>30</update_frequency>
    </plugin>
  </model>
</sdf>
"""


class BoxFeeder(Node):
    def __init__(self) -> None:
        super().__init__("box_feeder")

        self.declare_parameter("world", "box_cell")
        self.declare_parameter("friction", 0.6)      # 골판지 대 고무 벨트
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("spawn_delay", 8.0)   # 월드가 준비될 때까지

        self.cell = CellGeometry()
        self.world = str(self.get_parameter("world").value)
        self.mode = self.cell.scenario_mode

        share = Path(get_package_share_directory("box_cell_sim"))
        self.tex_dir = share / "models/box/materials/textures"
        self.label_mesh = share / "models/box/meshes/label_plane.obj"
        self.codes = self._load_codes(
            Path(get_package_share_directory("box_cell_mes")) / "seed_items.json"
        )

        self._last: dict[str, tuple[float, list[float]]] = {}
        self._pose: dict[str, list[float]] = {}
        self._twist: dict[str, list[float]] = {}
        # 아직 들여보내지 않은 박스. infeed 모드에서 순서대로 꺼내 쓴다.
        self._queue: list[str] = []
        self._sdf_dir = Path(tempfile.mkdtemp(prefix="box_cell_sdf_"))

        self.pub = self.create_publisher(BoxPoseArray, "/sim/boxes", 10)
        for i in range(1, self.cell.box_count + 1):
            self.create_subscription(TFMessage, f"/model/box_{i}/pose", self._on_poses, 20)

        self.create_service(Trigger, "/feeder/next", self._on_next)
        self.create_service(Trigger, "/feeder/recycle", self._on_recycle)
        self.create_service(Trigger, "/feeder/status", self._on_status)

        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)
        self._spawned = False
        self.create_timer(float(self.get_parameter("spawn_delay").value), self._try_spawn)

    # ------------------------------------------------------------------ gz 호출
    def _gz(self, service: str, reqtype: str, req: str, timeout_ms: int = 4000) -> bool:
        """gz 서비스를 CLI로 부른다.

        ROS 클라이언트로는 못 부른다. /world/<w>/create와 set_pose는 gz 서비스라
        브리지가 없으면 보이지 않는다. ros_gz_sim의 create 실행 파일도 있지만
        한 번에 ROS 노드가 하나씩 뜨느라 9초씩 걸린다. 박스를 옮길 때마다
        그걸 기다릴 수는 없다.
        """
        cmd = [
            "gz", "service", "-s", f"/world/{self.world}/{service}",
            "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
            "--timeout", str(timeout_ms), "--req", req,
        ]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000 + 5)
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().error(f"gz {service} 호출 실패 : {exc}")
            return False
        if out.returncode != 0 or "true" not in out.stdout.lower():
            self.get_logger().error(
                f"gz {service} 거부 : {(out.stderr or out.stdout).strip()[:200]}"
            )
            return False
        return True

    def _move(self, name: str, x: float, y: float, z: float, yaw: float = 0.0) -> bool:
        req = (
            f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
            f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}"
        )
        return self._gz("set_pose", "gz.msgs.Pose", req)

    # ------------------------------------------------------------------ 스폰
    def _load_codes(self, path: Path) -> dict[str, str]:
        if not path.exists():
            self.get_logger().warn(f"{path}가 없다. 라벨 코드를 임시로 만든다.")
            return {f"box_{i}": f"AXO-{i:04d}" for i in range(1, self.cell.box_count + 1)}
        return {it["box"]: it["code"] for it in json.loads(path.read_text())}

    def _write_sdf(self, idx: int) -> Path:
        name = f"box_{idx}"
        sx, sy, sz = self.cell.box_size
        m = float(self.cell.data["box"]["mass"])
        path = self._sdf_dir / f"{name}.sdf"
        path.write_text(
            BOX_SDF.format(
                name=name, mass=m,
                ixx=m * (sy * sy + sz * sz) / 12.0,
                iyy=m * (sx * sx + sz * sz) / 12.0,
                izz=m * (sx * sx + sy * sy) / 12.0,
                sx=sx, sy=sy, sz=sz,
                mu=float(self.get_parameter("friction").value),
                label_z=sz / 2.0 + 0.0001,
                cardboard=(self.tex_dir / "cardboard.png").as_uri(),
                label_mesh=self.label_mesh.as_uri(),
                label_map=(self.tex_dir / f"{name}.png").as_uri(),
            )
        )
        return path

    def _try_spawn(self) -> None:
        if self._spawned:
            return
        self._spawned = True

        made = 0
        for i in range(1, self.cell.box_count + 1):
            name = f"box_{i}"
            sdf = self._write_sdf(i)
            if self.mode == "circulate":
                # 기획서 순환 : 기동 시 팔레트 1을 가득 채운다.
                slot = self.cell.slot(1, i - 1)
                pos = (slot.x, slot.y, slot.center_z)
            else:
                # infeed : 셀 밖에서 대기한다. 하나씩 벨트 입구로 올린다.
                pos = self.cell.staging_pose(i - 1)
                self._queue.append(name)
            req = (
                f'sdf_filename: "{sdf}", name: "{name}", allow_renaming: false, '
                f"pose: {{position: {{x: {pos[0]}, y: {pos[1]}, z: {pos[2]}}}}}"
            )
            if self._gz("create", "gz.msgs.EntityFactory", req):
                made += 1

        if self.mode == "circulate":
            self.get_logger().info(
                f"박스 {made}/{self.cell.box_count}개를 팔레트 1에 채웠다(순환 모드). "
                "첫 동작은 디팔레타이징이다."
            )
        else:
            self.get_logger().info(
                f"박스 {made}/{self.cell.box_count}개를 셀 밖에 대기시켰다(투입 모드). "
                "/feeder/next 를 부르면 하나씩 벨트로 올라간다."
            )

    # ---------------------------------------------------------------- 서비스
    def _on_next(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """대기 중인 박스 하나를 벨트 입구에 올린다."""
        if not self._spawned:
            res.success = False
            res.message = "아직 박스를 만들지 않았다"
            return res
        if not self._queue:
            res.success = False
            res.message = "대기 중인 박스가 없다"
            return res

        # 벨트가 비어 있어야 한다. 앞 박스가 아직 실려 가는 중이면 겹친다.
        for b in self._current():
            if b.on_belt and not b.held:
                res.success = False
                res.message = f"벨트에 {b.name}가 아직 있다"
                return res

        name = self._queue.pop(0)
        x, y, z = self.cell.entry_pose()
        if not self._move(name, x, y, z):
            self._queue.insert(0, name)
            res.success = False
            res.message = f"{name} 투입 실패"
            return res

        res.success = True
        res.message = name
        self.get_logger().info(
            f"{name} 벨트 투입 (x={x:.3f}). 대기 {len(self._queue)}개 남음."
        )
        return res

    def _on_recycle(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """팔레트를 반출한 것으로 보고, 쌓인 박스를 대기 자리로 되돌린다.

        실제 셀에서는 지게차가 팔레트를 가져가고 빈 팔레트가 들어온다.
        시뮬레이터는 박스를 셀 밖으로 옮기는 것으로 그 자리를 대신한다.
        """
        moved = []
        for b in self._current():
            if b.pallet > 0 and not b.held:
                idx = int(b.name.split("_")[1]) - 1
                x, y, z = self.cell.staging_pose(idx)
                if self._move(b.name, x, y, z):
                    moved.append(b.name)
                    if b.name not in self._queue:
                        self._queue.append(b.name)
        self._queue.sort(key=lambda n: int(n.split("_")[1]))
        res.success = bool(moved)
        res.message = f"{len(moved)}개 반출"
        self.get_logger().info(f"팔레트 반출 : {', '.join(moved) or '없음'}")
        return res

    def _on_status(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        res.success = True
        res.message = f"mode={self.mode} queued={len(self._queue)} known={len(self._pose)}"
        return res

    # -------------------------------------------------------------- 상태 발행
    def _on_poses(self, msg: TFMessage) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        for tr in msg.transforms:
            name = tr.child_frame_id
            if not name.startswith("box_"):
                continue
            t = tr.transform.translation
            r = tr.transform.rotation
            cur = [t.x, t.y, t.z, r.x, r.y, r.z, r.w]
            prev = self._last.get(name)
            if prev is not None:
                dt = now - prev[0]
                if dt > 1e-4:
                    self._twist[name] = [(cur[i] - prev[1][i]) / dt for i in range(3)]
            self._last[name] = (now, cur)
            self._pose[name] = cur

    def _classify(self, pose: list[float]) -> tuple[int, int, bool]:
        """박스가 지금 어디 있는지. 팔레트 번호, 슬롯, 벨트 위 여부."""
        x, y, z = pose[0], pose[1], pose[2]
        x0, x1 = self.cell.belt_x_range
        half_w = float(self.cell.data["conveyor"]["belt"]["width"]) / 2.0
        on_belt = (
            x0 - 0.08 <= x <= x1 + 0.05
            and abs(y - self.cell.belt_center_y) <= half_w
            and abs(z - (self.cell.belt_surface_z + self.cell.box_height / 2)) < 0.05
        )
        for pid in self.cell.pallet_ids:
            for s in self.cell.all_slots(pid):
                if math.hypot(x - s.x, y - s.y) < 0.025 and abs(z - s.center_z) < 0.020:
                    return pid, s.index, False
        eb = self.cell.data["exception_bin"]
        if (
            abs(x - eb["center"][0]) < eb["size"][0] / 2
            and abs(y - eb["center"][1]) < eb["size"][1] / 2
        ):
            return -1, -1, False
        return 0, -1, on_belt

    def _current(self) -> list[BoxPose]:
        out: list[BoxPose] = []
        for name in sorted(self._pose, key=lambda n: int(n.split("_")[1])):
            p = self._pose[name]
            b = BoxPose()
            b.name = name
            b.code = self.codes.get(name, "")
            b.pose.position.x, b.pose.position.y, b.pose.position.z = p[0], p[1], p[2]
            (
                b.pose.orientation.x, b.pose.orientation.y,
                b.pose.orientation.z, b.pose.orientation.w,
            ) = p[3], p[4], p[5], p[6]
            v = self._twist.get(name, [0.0, 0.0, 0.0])
            b.twist.linear.x, b.twist.linear.y, b.twist.linear.z = v
            b.pallet, b.slot, b.on_belt = self._classify(p)
            out.append(b)
        return out

    def _publish(self) -> None:
        if not self._pose:
            return
        msg = BoxPoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.boxes = self._current()
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = BoxFeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
