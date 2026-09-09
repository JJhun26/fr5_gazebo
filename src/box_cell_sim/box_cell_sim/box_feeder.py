#!/usr/bin/env python3
"""택배박스를 만들고, 컨베이어로 들여보내고, 그 실제 상태를 알려 준다.

시나리오 (cell.yaml의 scenario.mode)

  infeed    박스가 컨베이어를 타고 들어온다. 로봇이 판독 위치에서 흡착으로
            집어 팔레트에 구석부터 하나씩 쌓는다. 팔레트가 차면 반출한 것으로
            보고 비운 뒤 다음 로트를 받는다.

  circulate 기획서 "순환"의 두 팔레트 방식. 기동 시 팔레트 1을 채워 두고,
            로봇이 하나씩 꺼내 벨트에 되올린다.

실제 셀에서는 상류 컨베이어가 박스를 보내 온다. 시뮬레이터도 그 상류를
짧게 만들어 두었다(cell.yaml의 conveyor.infeed). 박스는 필요한 순간에
그 위에 하나씩 나타나 실려 온다.

미리 줄 세워 두지 않는 이유가 핵심이다. gz는 안착한 물체를 재우는데,
한 번 잠들면 접촉 상대(롤러)가 나중에 돌기 시작해도 다시 깨어나지 않는다.
위로 50 N을 걸어도 안 뜰 만큼 완전히 얼어 있다. 반대로 이미 도는 롤러 위에
떨어뜨리면 잠들 틈이 없다. 그래서 "만들 때 이미 움직이는 위에 놓는다".

옮기거나 지우지 않는 이유도 같은 계열이다. gz의 set_pose와 remove는
data: true를 돌려주면서 실제로는 아무 일도 하지 않는다. create만 정상이다.
그래서 create 말고는 아무것도 엔티티 조작에 기대지 않는다.

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
from box_cell_common.pallet_pack import Packer
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
      <visual name="tape">
        <!-- 상면을 가로지르는 포장 테이프. 생김새만 낸다.
             라벨보다 아래에 깔아 QR을 가리지 않게 한다.
             두께가 라벨의 띄움보다 크면 안 된다. 0.4 mm 상자를
             상면+0.1 mm에 두면 윗면이 +0.3 mm가 되어 라벨(+0.2 mm)을
             덮는다. 실제로 그렇게 QR 오른쪽 절반이 가려져 전부
             디코드에 실패했다. 두께를 0.1 mm로 줄이고 상면에 붙인다. -->
        <pose>0 0 {tape_z} 0 0 0</pose>
        <geometry><box><size>{tape_x} {tape_w} 0.0001</size></box></geometry>
        <material>
          <ambient>{tape_rgba}</ambient>
          <diffuse>{tape_rgba}</diffuse>
          <specular>0.12 0.12 0.12 1</specular>
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
        self.kinds = self._load_kinds()
        self.world = str(self.get_parameter("world").value)
        self.mode = self.cell.scenario_mode

        share = Path(get_package_share_directory("box_cell_sim"))
        self.tex_dir = share / "models/box/materials/textures"
        self.label_mesh = share / "models/box/meshes/label_plane.obj"
        self.codes = self._load_codes(
            Path(get_package_share_directory("box_cell_mes")) / "seed_items.json"
        )

        # 아직 만들지 않은 박스. 앞에서부터 하나씩 꺼내 쓴다.
        self._pending: list[int] = []
        self._last: dict[str, tuple[float, list[float]]] = {}
        self._pose: dict[str, list[float]] = {}
        self._twist: dict[str, list[float]] = {}
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

    # ------------------------------------------------------------------ 스폰
    def _load_codes(self, path: Path) -> dict[str, str]:
        if not path.exists():
            self.get_logger().warn(f"{path}가 없다. 라벨 코드를 임시로 만든다.")
            return {f"box_{i}": f"AXO-{i:04d}" for i in range(1, self.cell.box_count + 1)}
        return {it["box"]: it["code"] for it in json.loads(path.read_text())}

    def _load_kinds(self) -> dict[str, str]:
        """박스 이름 -> 규격 이름. MES 시드가 원본이다.

        시뮬레이터는 실제 물건을 만들어야 하므로 규격을 미리 알아야 한다.
        셀의 로직은 그렇지 않다. 로직은 바코드를 읽고 MES에 물어야 안다.
        여기서 규격을 아는 것은 "실물 상자가 이미 그 크기로 존재한다"는
        뜻이지, 로봇이 안다는 뜻이 아니다.
        """
        path = Path(get_package_share_directory("box_cell_mes")) / "seed_items.json"
        if not path.exists():
            return {}
        default = str(self.cell.data["box"].get("default_kind", "M"))
        return {it["box"]: it.get("kind", default) for it in json.loads(path.read_text())}

    def _prefill_pose(self, n: int) -> tuple[float, float, float]:
        """순환 모드에서 팔레트 1에 미리 쌓아 둘 n번째 박스의 중심.

        pallet_manager._restore가 쓰는 것과 같은 패커, 같은 순서다.
        한쪽만 고치면 기록과 실물이 어긋난다.
        """
        cfg = self.cell.data["pallet"]["slots"]
        pk = Packer(
            size=float(self.cell.data["pallet"]["size"]),
            margin=float(cfg["margin"]),
            gap=float(cfg["gap"]),
            step=float(cfg["step"]),
            max_layers=int(cfg["layers"]),
        )
        sx, sy, sz = self.cell.default_box_size
        px, py = self.cell.pallet_center(1)
        base = self.cell.table_top + float(self.cell.data["pallet"]["thickness"])
        pl = None
        for i in range(n + 1):
            pl = pk.find(sx, sy, sz)
            if pl is None:
                break
            pk.add(pl, "")
        if pl is None:
            return (px, py, base + sz / 2)
        return (px + pl.x, py + pl.y, base + pl.z_base + pl.sz / 2)

    def _write_sdf(self, idx: int) -> Path:
        name = f"box_{idx}"
        kind = self.kinds.get(name, str(self.cell.data["box"].get("default_kind", "M")))
        size = self.cell.box_size_of(kind)
        if size is None:
            self.get_logger().warn(f"{name}의 규격 {kind}를 모른다. 기본 규격으로 만든다.")
            size = self.cell.default_box_size
            kind = str(self.cell.data["box"].get("default_kind", "M"))
        sx, sy, sz = size
        m = float(self.cell.box_kinds().get(kind, {}).get(
            "mass", self.cell.data["box"].get("mass", 0.45)))
        path = self._sdf_dir / f"{name}.sdf"
        path.write_text(
            BOX_SDF.format(
                name=name, mass=m,
                ixx=m * (sy * sy + sz * sz) / 12.0,
                iyy=m * (sx * sx + sz * sz) / 12.0,
                izz=m * (sx * sx + sy * sy) / 12.0,
                sx=sx, sy=sy, sz=sz,
                mu=float(self.get_parameter("friction").value),
                label_z=sz / 2.0 + 0.0004,
                tape_w=float(self.cell.data["box"]["tape"]["width"]),
                tape_x=sx + 0.001,
                tape_z=sz / 2.0 + 0.00005,
                tape_rgba=" ".join(str(v) for v in self.cell.data["box"]["tape"]["color"]),
                cardboard=(self.tex_dir / "cardboard.png").as_uri(),
                label_mesh=self.label_mesh.as_uri(),
                label_map=(self.tex_dir / f"{name}.png").as_uri(),
            )
        )
        return path

    def _create(self, idx: int, pos: tuple[float, float, float]) -> bool:
        sdf = self._write_sdf(idx)
        req = (
            f'sdf_filename: "{sdf}", name: "box_{idx}", allow_renaming: false, '
            f"pose: {{position: {{x: {pos[0]}, y: {pos[1]}, z: {pos[2]}}}}}"
        )
        return self._gz("create", "gz.msgs.EntityFactory", req)

    def _try_spawn(self) -> None:
        """기동 때 한 번.

        순환 모드는 팔레트 1을 채워 두고 시작한다. 투입 모드는 아무것도
        만들지 않고 대기 목록만 세운다. 박스는 /feeder/next 가 올 때마다
        도는 롤러 위에 하나씩 나타난다.
        """
        if self._spawned:
            return
        self._spawned = True

        if self.mode == "circulate":
            # 순환 모드의 출발 상태. 데모가 시작되기 전에 누군가 쌓아 둔
            # 팔레트다. pallet_manager도 같은 계산으로 같은 자리를 기록하므로
            # 기록과 실물이 어긋나지 않는다(둘 다 기본 규격으로 채운다).
            made = sum(
                self._create(i, self._prefill_pose(i - 1))
                for i in range(1, self.cell.box_count + 1)
            )
            self.get_logger().info(
                f"박스 {made}/{self.cell.box_count}개를 팔레트 1에 채웠다(순환 모드). "
                "첫 동작은 디팔레타이징이다."
            )
        else:
            self._pending = list(range(1, self.cell.box_count + 1))
            x, _y, _z = self.cell.feed_spawn_pose()
            self.get_logger().info(
                f"투입 모드. 박스 {len(self._pending)}개를 차례로 받는다. "
                f"요청이 올 때마다 x={x:.3f}의 도는 롤러 위에 하나씩 나타난다."
            )

    # ---------------------------------------------------------------- 서비스
    def _on_next(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """상류에서 박스 하나를 흘려보낸다.

        부르는 쪽(task_manager)이 **롤러를 먼저 돌려 두어야** 한다.
        멈춘 롤러 위에 떨어진 박스는 안착과 동시에 잠들고, 그다음에 롤러가
        돌아도 다시 깨어나지 않는다.
        """
        if not self._spawned:
            res.success = False
            res.message = "아직 박스를 만들지 않았다"
            return res
        if not self._pending:
            res.success = False
            res.message = "대기 중인 박스가 없다"
            return res

        # 벨트가 비어 있어야 한다. 앞 박스가 아직 실려 오는 중이면 겹친다.
        for b in self._current():
            if b.on_belt and not b.held:
                res.success = False
                res.message = f"벨트에 {b.name}가 아직 있다"
                return res

        idx = self._pending[0]
        # 그 박스의 실제 높이로 떨어뜨린다. 기본 규격 높이로 놓으면
        # 큰 박스가 벨트에 거의 닿은 채로 생겨 그대로 잠든다.
        kind = self.kinds.get(f"box_{idx}", "")
        size = self.cell.box_size_of(kind) or self.cell.default_box_size
        pos = self.cell.feed_spawn_pose(size[2])
        if not self._create(idx, pos):
            res.success = False
            res.message = f"box_{idx} 투입 실패"
            return res
        self._pending.pop(0)

        res.success = True
        res.message = f"box_{idx}"
        self.get_logger().info(
            f"box_{idx} 상류 투입 (x={pos[0]:.3f}). 남은 {len(self._pending)}개."
        )
        return res

    def _on_recycle(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """더 이상 하는 일이 없다.

        예전에는 팔레트에 쌓인 박스를 셀 밖으로 되돌렸다. 지금은 팔레트가 차면
        로봇이 직접 하나씩 꺼내 벨트로 되올린다(task_manager의 순환 전환).
        지게차를 흉내 내려고 박스를 순간이동시키는 것보다, 로봇이 실제로
        옮기는 편이 데모로도 낫고 실물과도 가깝다.
        """
        res.success = True
        res.message = "반출은 로봇이 한다(순환)"
        return res

    def _on_status(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        res.success = True
        res.message = (
            f"mode={self.mode} 남은투입={len(self._pending)} 아는박스={len(self._pose)}"
        )
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

    def _classify(self, pose: list[float], height: float | None = None) -> tuple[int, int, bool]:
        """박스가 지금 어디 있는지. 팔레트 번호, 슬롯, 벨트 위 여부."""
        x, y, z = pose[0], pose[1], pose[2]
        # 상류 인피드 구간까지 벨트로 본다. 거기 서 있는 박스도 벨트 위다.
        x0, x1 = self.cell.on_belt_x_range()
        half_w = float(self.cell.data["conveyor"]["belt"]["width"]) / 2.0
        # 높이는 그 박스의 실제 높이로 본다. 기본 규격으로 재면 S와 XL이
        # 기준에서 각각 반대로 벗어나 여유 50 mm를 거의 다 먹는다.
        h = float(height) if height else float(self.cell.box_height)
        on_belt = (
            x0 - 0.05 <= x <= x1 + 0.05
            and abs(y - self.cell.belt_center_y) <= half_w
            and abs(z - (self.cell.belt_surface_z + h / 2)) < 0.05
        )
        # 팔레트 위인가. 고정 격자가 없어졌으므로 자리 하나하나와 대 보지
        # 않는다. 팔레트 상면 위, 팔레트 테두리 안이면 그 팔레트로 본다.
        # 몇 번째로 놓였는지는 pallet_manager가 안다. 여기서 셀 일이 아니다.
        pal_half = float(self.cell.data["pallet"]["size"]) / 2.0
        deck = self.cell.table_top + float(self.cell.data["pallet"]["thickness"])
        for pid in self.cell.pallet_ids:
            px, py = self.cell.pallet_center(pid)
            if abs(x - px) < pal_half and abs(y - py) < pal_half and z > deck - 0.01:
                return pid, -1, False
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
            # 실제 치수. 정답지다. gripper_driver가 상면 높이를 알아야
            # 컵이 닿았는지 판정할 수 있다. 규격이 하나였을 때는 cell.yaml의
            # 값 하나면 됐는데 이제는 박스마다 다르다.
            sz = self.cell.box_size_of(self.kinds.get(name, "")) or self.cell.default_box_size
            b.pallet, b.slot, b.on_belt = self._classify(p, sz[2])
            b.size.x, b.size.y, b.size.z = sz
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
