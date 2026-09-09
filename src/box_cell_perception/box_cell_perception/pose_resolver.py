#!/usr/bin/env python3
"""픽셀을 mm로 바꾼다. 기획서 5.4 파이프라인의 뒤쪽 절반.

    ... 네 모서리 좌표 -> 호모그래피 -> (x, y, yaw)

기획서의 근거는 이것이다.
  "박스 높이가 40으로 동일해 상면이 항상 같은 평면이다. 체커보드로 구한
   호모그래피 한 장이면 픽셀이 mm로 바뀐다. 깊이 카메라가 필요 없다."

상면이 항상 같은 평면(z = 벨트 상면 + 박스 높이)이므로, 픽셀에서 그 평면
위의 점으로 가는 사상은 호모그래피 하나다. 여기서는 그 행렬을 두 방법으로
얻는다.

  computed  : 카메라 내부 파라미터(camera_info)와 TF의 카메라 자세로 직접
              계산한다. 시뮬레이터에서는 이쪽이 정확하고, 캘리브레이션을
              기다릴 필요가 없다.
  file      : 캘리브레이션 결과 3x3 행렬을 읽는다. 실물에서 쓰는 경로다.
              체커보드로 구한 행렬을 config/homography_<camera>.yaml에 넣으면
              나머지 코드는 그대로 돈다.

회전각은 라벨 네 모서리를 평면에 올린 뒤 변의 기울기로 낸다. 박스가 60 각
정사각형이라 90도 대칭이므로 ±45도 안으로 접는다(기획서 5.4). FR5 J6 범위가
±175도이니 여유가 크다.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from box_cell_common.cell_geometry import CellGeometry, yaw_normalize_square
from box_cell_msgs.msg import LabelCorners, LabelDetection
from box_cell_msgs.srv import ItemQuery
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class PoseResolver(Node):
    def __init__(self) -> None:
        super().__init__("pose_resolver")

        self.declare_parameter("camera", "c1_conveyor")
        self.declare_parameter("source", "computed")     # computed | file
        self.declare_parameter("calibration", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("plane_z", 0.0)           # 0이면 판독면을 자동으로 쓴다

        self.camera = str(self.get_parameter("camera").value)
        self.cell = CellGeometry()
        self.K: np.ndarray | None = None
        self.H: np.ndarray | None = None      # 픽셀 -> 평면 위 world (x, y)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 판독 평면. 규격이 여러 가지라 하나로 못 정한다.
        #
        # 기획서 5.4는 "박스 높이가 40으로 동일해 상면이 항상 같은 평면"이라
        # 호모그래피 한 장으로 끝냈다. 규격이 S/M/L/XL로 나뉘면 그 전제가
        # 깨진다. 그래도 깊이 카메라는 여전히 필요 없다. 코드를 먼저 읽고
        # MES에 그 규격의 높이를 물어, 그 높이의 평면으로 투영하면 된다.
        # 호모그래피가 규격 수만큼 있는 셈이다.
        #
        # plane_z 파라미터를 주면 그 값으로 못박는다. 캘리브레이션 검증용이다.
        self.plane_fixed = float(self.get_parameter("plane_z").value)
        self.plane_z = self.plane_fixed if self.plane_fixed > 0 else self.cell.read_top_z
        self.height_cache: dict[str, float] = {}
        # 콜백 안에서 서비스를 동기로 부른다. 같은 그룹에 두면 응답을
        # 처리할 스레드가 없어 교착된다. Reentrant 그룹 + 다중 스레드 실행기가
        # 짝이다(task_manager와 같은 방식).
        self.cb = ReentrantCallbackGroup()
        self.mes_item = self.create_client(ItemQuery, "/mes/item", callback_group=self.cb)

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CameraInfo, f"/{self.camera}/color/camera_info", self._on_info, sensor_qos
        )
        self.create_subscription(
            LabelCorners, f"/perception/{self.camera}/corners", self._on_corners, 10,
            callback_group=self.cb,
        )
        self.pub = self.create_publisher(LabelDetection, "/perception/label", 10)

        if str(self.get_parameter("source").value) == "file":
            self._load_calibration()

        self.get_logger().info(
            f"pose_resolver({self.camera}) 준비. 판독 평면 z={self.plane_z:.3f} m, "
            f"source={self.get_parameter('source').value}"
        )

    # ------------------------------------------------------------- 호모그래피
    def _load_calibration(self) -> None:
        path = str(self.get_parameter("calibration").value)
        if not path:
            self.get_logger().error("source=file인데 calibration 경로가 비었다")
            return
        data = yaml.safe_load(Path(path).read_text())
        self.H = np.array(data["homography"], dtype=np.float64).reshape(3, 3)
        self.get_logger().info(f"호모그래피를 {path}에서 읽었다")

    def _on_info(self, msg: CameraInfo) -> None:
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(
                f"내부 파라미터 : fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} "
                f"cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}"
            )

    def _compute_homography(self) -> np.ndarray | None:
        """카메라 자세와 내부 파라미터에서 픽셀 -> 평면 사상을 만든다.

        광학 프레임의 광선 d를 world로 돌린 뒤, 평면 z = plane_z와 만나는
        점을 구한다. 그 사상이 곧 호모그래피다. 3x3으로 접어 두면 실물의
        체커보드 캘리브레이션 결과와 같은 형태가 되어 둘을 바꿔 낄 수 있다.
        """
        if self.K is None:
            self.get_logger().warn(
                f"/{self.camera}/color/camera_info를 아직 못 받았다. "
                "브리지 설정과 gz 센서 토픽 이름을 확인할 것.",
                throttle_duration_sec=5.0,
            )
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("world_frame").value),
                f"{self.camera}_color_optical_frame",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.3),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"카메라 TF를 못 읽었다 : {exc}", throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        cam = np.array([t.x, t.y, t.z])

        if abs(cam[2] - self.plane_z) < 1e-6:
            self.get_logger().error("카메라가 판독 평면 위에 있다. 평면 높이를 확인할 것.")
            return None

        # 픽셀 -> 광학 프레임 방향 : Kinv @ [u, v, 1]
        k_inv = np.linalg.inv(self.K)
        # 광선 방향을 world로 : rot @ d
        m = rot @ k_inv                       # 픽셀 -> world 방향 (스케일 자유)
        # 평면 교점 : cam + s * m @ p, 여기서 s는 z가 plane_z가 되게 잡는다
        #   cam_z + s * (m[2] @ p) = plane_z  ->  s = (plane_z - cam_z) / (m[2] @ p)
        # x = cam_x + s * (m[0] @ p), y = cam_y + s * (m[1] @ p)
        # 동차 좌표로 접으면 3x3 하나가 된다.
        dz = self.plane_z - cam[2]
        h = np.vstack(
            [
                dz * m[0] + cam[0] * m[2],
                dz * m[1] + cam[1] * m[2],
                m[2],
            ]
        )
        return h

    def _plane_for(self, code: str) -> float:
        """이 코드의 박스 상면 높이. MES에 묻는다.

        한 번 물은 코드는 기억한다. 같은 박스를 세 번 판독할 때마다 서비스를
        부를 이유가 없고, MES가 잠깐 죽어도 이미 아는 코드는 계속 돈다.

        모르면 기본 규격으로 가정한다. 틀린 평면으로 투영하면 위치가 밀리는데,
        그 상태로 집으러 가지는 않는다. task_manager가 MES 조회 실패를
        예외 처리로 받기 때문이다. 여기서는 계속 답을 내는 쪽이 낫다.
        """
        if code in self.height_cache:
            return self.height_cache[code]
        h = None
        if self.mes_item.service_is_ready():
            req = ItemQuery.Request()
            req.code = code
            res = self.mes_item.call(req)
            if res is not None and res.found and len(res.box_size) == 3:
                h = float(res.box_size[2])
        if h is None or h <= 0.0:
            # 기본값은 기억하지 않는다. MES가 아직 안 떴을 뿐인데 그 답을
            # 캐시에 넣으면 서버가 살아난 뒤에도 영영 틀린 평면으로 투영한다.
            # 실제로 첫 판독이 MES 기동 전에 걸려 그렇게 굳었다.
            h = float(self.cell.default_box_size[2])
            self.get_logger().warn(
                f"{code}의 규격을 MES에서 못 받았다. 기본 규격 높이 {h*1000:.0f} mm로 본다.",
                throttle_duration_sec=10.0,
            )
            return self.cell.read_top_z_for(h)
        z = self.cell.read_top_z_for(h)
        self.height_cache[code] = z
        self.get_logger().info(
            f"{code} 판독 평면 z={z:.3f} m (박스 높이 {h*1000:.0f} mm)"
        )
        return z

    def _project(self, u: float, v: float) -> tuple[float, float] | None:
        if self.H is None:
            return None
        p = self.H @ np.array([u, v, 1.0])
        if abs(p[2]) < 1e-12:
            return None
        return float(p[0] / p[2]), float(p[1] / p[2])

    # ------------------------------------------------------------------ 변환
    def _on_corners(self, msg: LabelCorners) -> None:
        out = LabelDetection()
        out.header = msg.header
        out.header.frame_id = str(self.get_parameter("world_frame").value)
        out.code = msg.code
        out.attempt = msg.attempt
        out.source = msg.source

        if not msg.ok:
            out.ok = False
            self.pub.publish(out)
            return

        # 이 코드의 박스가 몇 mm짜리인지 MES에 묻고, 그 높이의 평면을 쓴다.
        if self.plane_fixed <= 0:
            self.plane_z = self._plane_for(msg.code)

        if str(self.get_parameter("source").value) == "computed":
            self.H = self._compute_homography()
        if self.H is None:
            out.ok = False
            self.get_logger().warn("호모그래피가 아직 없다")
            self.pub.publish(out)
            return

        pts = np.array(msg.corners, dtype=np.float64).reshape(4, 2)
        world = []
        for u, v in pts:
            xy = self._project(u, v)
            if xy is None:
                out.ok = False
                self.pub.publish(out)
                return
            world.append(xy)
        world_arr = np.array(world)

        # 라벨이 상면 중앙이므로 모서리 평균이 곧 박스 중심이다.
        cx, cy = world_arr.mean(axis=0)

        # 회전각은 네 변의 기울기 평균이다. 한 변만 쓰면 모서리 하나의
        # 픽셀 오차가 그대로 각도 오차가 된다.
        angles = []
        for i in range(4):
            d = world_arr[(i + 1) % 4] - world_arr[i]
            angles.append(math.atan2(d[1], d[0]) - i * math.pi / 2)
        # 각도 평균은 벡터로 낸다. 179도와 -179도의 산술평균은 0도가 된다.
        mean = math.atan2(
            float(np.mean([math.sin(a) for a in angles])),
            float(np.mean([math.cos(a) for a in angles])),
        )
        yaw = yaw_normalize_square(mean)

        # 신뢰도 : 디코드 여유와 잰 길이가 실제 치수에 얼마나 맞는지.
        #
        # 비교 대상은 라벨 전체가 아니라 QR 본체다. label_reader가 돌려주는
        # 네 점은 QR의 모서리이고, QR은 라벨에서 사방 여백과 문자열 인쇄 폭을
        # 뺀 만큼이다. 라벨 크기와 견주면 언제나 작게 나와 신뢰도가 깎인다.
        expected = self.cell.qr_side
        sides = [
            float(np.linalg.norm(world_arr[(i + 1) % 4] - world_arr[i])) for i in range(4)
        ]
        side_err = abs(float(np.mean(sides)) - expected) / expected
        confidence = max(0.0, min(1.0, msg.decode_margin * (1.0 - min(1.0, side_err * 4))))

        out.ok = True
        out.center_x = cx
        out.center_y = cy
        out.yaw = yaw
        out.confidence = confidence
        self.pub.publish(out)
        self.get_logger().info(
            f"{msg.code} 중심 ({cx*1000:.1f}, {cy*1000:.1f}) mm, "
            f"회전 {math.degrees(yaw):+.1f} deg, 한 변 {np.mean(sides)*1000:.1f} mm "
            f"(기대 {expected*1000:.0f}), 신뢰도 {confidence:.2f}"
        )


def main() -> None:
    rclpy.init()
    node = PoseResolver()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
