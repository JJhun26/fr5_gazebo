#!/usr/bin/env python3
"""C2 팔레트 탑뷰로 적재 결과를 확인한다 (기획서 D 항목).

왜 필요한가.
    ES45 흡착 그리퍼에는 피드백이 없다. 밸브를 열었다는 것만 알고 잡았는지는
    모른다. 그래서 로봇이 동작을 끝냈다는 것과 박스가 옮겨졌다는 것은 다른
    이야기다.

    헛집기는 정지 센서로 잡는다. 집어 갔으면 판독 자리가 비어야 하니까.
    하지만 그건 "집었는가"까지다. 이송 중에 떨어뜨렸거나, 자리를 빗나가
    옆에 놓았거나, 위층이 무너진 것은 놓은 자리를 봐야 안다.

어떻게 보는가.
    깊이로 본다. 팔레트 데크에서 박스 상면까지의 높이가 기대와 맞으면
    놓인 것이다. 색으로 보면 조명과 상자 색에 휘둘리는데 높이는 그렇지 않다.
    D455에 깊이가 있는 이유이기도 하다(C1은 깊이를 끈다. 거기서는 상면이
    항상 같은 평면이라 필요 없었다).

    기대 발자국을 화면에 투영하고, 그 안의 화소 중 높이가 기대 ±허용 안에
    드는 비율(coverage)을 센다. 절반을 넘으면 놓인 것으로 본다. 한 화소만
    보면 노이즈에 걸리고, 전부를 요구하면 모서리에서 떨어진다.

    깊이 영상이 없으면 판정을 보류한다. **못 봤다를 놓였다로 바꾸지 않는다.**
    그 순간 이 노드는 있으나 마나 한 것이 된다.
"""

from __future__ import annotations

import numpy as np
import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.srv import VerifyStack
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class StackCheck(Node):
    def __init__(self) -> None:
        super().__init__("stack_check")

        self.declare_parameter("camera", "c2_pallet")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("height_tol", 0.020)     # 높이 허용 오차
        self.declare_parameter("min_coverage", 0.50)    # 발자국 중 맞아야 할 비율
        self.declare_parameter("inset", 0.015)          # 발자국에서 안으로 줄일 폭
        self.declare_parameter("wait_sec", 2.0)         # 새 깊이 영상 대기

        self.camera = str(self.get_parameter("camera").value)
        self.cell = CellGeometry()
        self.bridge = CvBridge()
        self.depth: np.ndarray | None = None
        self.depth_stamp = 0.0
        self.K: np.ndarray | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cb = ReentrantCallbackGroup()

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, f"/{self.camera}/depth/image_raw", self._on_depth, sensor_qos,
            callback_group=self.cb,
        )
        self.create_subscription(
            CameraInfo, f"/{self.camera}/depth/camera_info", self._on_info, sensor_qos,
            callback_group=self.cb,
        )
        self.create_service(
            VerifyStack, "/perception/verify_stack", self._on_verify, callback_group=self.cb
        )
        self.get_logger().info(f"stack_check({self.camera}) 준비.")

    def _on_depth(self, msg: Image) -> None:
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"깊이 변환 실패 : {exc}")
            return
        arr = np.asarray(img, dtype=np.float32)
        # 16UC1이면 mm 단위다. 32FC1이면 이미 m다.
        if img.dtype == np.uint16:
            arr = arr / 1000.0
        self.depth = arr
        self.depth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_info(self, msg: CameraInfo) -> None:
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(
                f"깊이 내부 파라미터 fx={self.K[0,0]:.1f} cx={self.K[0,2]:.1f}"
            )

    # ------------------------------------------------------------------ 판정
    def _on_verify(self, req: VerifyStack.Request,
                   res: VerifyStack.Response) -> VerifyStack.Response:
        size = list(req.box_size)
        if len(size) != 3 or min(size) <= 0:
            size = list(self.cell.default_box_size)
        deck = self.cell.table_top + float(self.cell.data["pallet"]["thickness"])
        res.expected_height = float(req.place_pose.position.z - deck)

        if self.depth is None or self.K is None:
            res.ok = False
            res.detail = "깊이 영상이나 내부 파라미터가 없다. 판정 보류."
            self.get_logger().warn(res.detail)
            return res

        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("world_frame").value),
                f"{self.camera}_depth_optical_frame",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception as exc:  # noqa: BLE001
            res.ok = False
            res.detail = f"카메라 TF를 못 읽었다 : {exc}"
            self.get_logger().warn(res.detail)
            return res

        t = tf.transform.translation
        q = tf.transform.rotation
        cam = np.array([t.x, t.y, t.z])
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        rot_inv = rot.T

        depth = self.depth
        h_img, w_img = depth.shape[:2]
        inset = float(self.get_parameter("inset").value)
        hx = max(0.005, size[0] / 2 - inset)
        hy = max(0.005, size[1] / 2 - inset)
        cx, cy = req.place_pose.position.x, req.place_pose.position.y

        hits = 0
        total = 0
        heights: list[float] = []
        tol = float(self.get_parameter("height_tol").value)
        for fx in np.linspace(-hx, hx, 7):
            for fy in np.linspace(-hy, hy, 7):
                # 기대 상면 위의 점을 화면으로 투영한다.
                p_world = np.array([cx + fx, cy + fy, req.place_pose.position.z])
                p_cam = rot_inv @ (p_world - cam)
                if p_cam[2] <= 1e-6:
                    continue
                uv = self.K @ (p_cam / p_cam[2])
                u, v = int(round(uv[0])), int(round(uv[1]))
                if not (0 <= u < w_img and 0 <= v < h_img):
                    continue
                d = float(depth[v, u])
                if not np.isfinite(d) or d <= 0.0:
                    continue
                total += 1
                # 그 화소가 실제로 본 점의 world z
                ray = np.linalg.inv(self.K) @ np.array([u, v, 1.0])
                seen = cam + rot @ (ray / ray[2] * d)
                height = float(seen[2] - deck)
                heights.append(height)
                if abs(height - res.expected_height) <= tol:
                    hits += 1

        if total == 0:
            res.ok = False
            res.detail = "기대 자리가 화각 밖이거나 깊이가 비었다. 판정 보류."
            self.get_logger().warn(res.detail)
            return res

        res.coverage = hits / total
        res.measured_height = float(np.median(heights)) if heights else 0.0
        res.ok = res.coverage >= float(self.get_parameter("min_coverage").value)
        res.detail = (
            f"기대 {res.expected_height*1000:.0f} mm, 실측 중앙값 "
            f"{res.measured_height*1000:.0f} mm, 일치 {res.coverage*100:.0f}% "
            f"({hits}/{total} 화소)"
        )
        (self.get_logger().info if res.ok else self.get_logger().error)(
            f"적재 확인 {'통과' if res.ok else '실패'} : {res.detail}"
        )
        return res


def main() -> None:
    rclpy.init()
    node = StackCheck()
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
