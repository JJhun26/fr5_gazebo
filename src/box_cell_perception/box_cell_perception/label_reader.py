#!/usr/bin/env python3
"""QR 판독. 기획서 5.4의 파이프라인 앞쪽 절반.

    촬영 -> QR 디코드 -> 네 모서리 좌표   <- 여기까지가 이 노드
    -> 호모그래피 -> (x, y, yaw)          <- pose_resolver

이 노드는 픽셀만 안다. 실세계 mm는 모른다. 나누는 이유는 캘리브레이션이다.
하드웨어가 오면 카메라 외부 파라미터를 다시 잡는데, 그때 고쳐야 할 곳이
한 군데여야 한다.

라이브러리는 OpenCV QRCodeDetector를 쓴다. 기획서 5.4는 pyzbar와 판독률을
비교해 고르라고 되어 있다. 파라미터 하나로 갈아 끼울 수 있게 해 두었다.
OpenCV 쪽을 기본으로 둔 이유는 네 모서리 좌표를 함께 돌려주기 때문이다.
pyzbar도 위치를 주지만 QR의 정렬 패턴 기준이라 모서리 정밀도가 떨어진다.

판독 실패는 정상 동작의 일부다. 기획서는 3회까지 재시도하고 그래도 안 되면
예외 통으로 보낸다. 이 노드는 세는 일을 하지 않는다. 매 요청에 대해
성공이든 실패든 한 번 답할 뿐이고, 세는 것은 task_manager가 한다.
"""

from __future__ import annotations

import numpy as np
import rclpy
from box_cell_msgs.msg import LabelCorners
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

try:
    import cv2
except ImportError as exc:  # pragma: no cover - 런타임 환경 문제
    raise SystemExit("opencv-python이 필요하다") from exc


class LabelReader(Node):
    def __init__(self) -> None:
        super().__init__("label_reader")

        self.declare_parameter("camera", "c1_conveyor")
        self.declare_parameter("image_topic", "/c1_conveyor/color/image_raw")
        self.declare_parameter("backend", "opencv")     # opencv | pyzbar
        self.declare_parameter("min_side_px", 40.0)     # 이보다 작으면 믿지 않는다
        self.declare_parameter("publish_debug", True)

        self.camera = str(self.get_parameter("camera").value)
        self.bridge = CvBridge()
        self.latest: np.ndarray | None = None
        self.detector = cv2.QRCodeDetector()

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._on_image, sensor_qos
        )
        self.pub = self.create_publisher(LabelCorners, f"/perception/{self.camera}/corners", 10)
        self.debug_pub = self.create_publisher(Image, f"/perception/{self.camera}/debug", 1)
        self.create_service(Trigger, f"/perception/{self.camera}/read", self._on_read)

        self.get_logger().info(
            f"label_reader({self.camera}) 준비. "
            f"backend={self.get_parameter('backend').value}"
        )

    def _on_image(self, msg: Image) -> None:
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"영상 변환 실패 : {exc}")

    # ------------------------------------------------------------------ 판독
    def _decode(self, img: np.ndarray) -> tuple[str, np.ndarray | None, float]:
        backend = str(self.get_parameter("backend").value)
        if backend == "pyzbar":
            return self._decode_pyzbar(img)
        return self._decode_opencv(img)

    def _decode_opencv(self, img: np.ndarray) -> tuple[str, np.ndarray | None, float]:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        data, points, _ = self.detector.detectAndDecode(gray)
        if not data or points is None:
            return "", None, 0.0
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        # 디코드 여유 대신 모서리 사각형의 정직함을 신뢰도로 쓴다.
        # 네 변의 길이가 고르고 대각이 같으면 라벨을 정면에 가깝게 본 것이다.
        sides = [float(np.linalg.norm(pts[i] - pts[(i + 1) % 4])) for i in range(4)]
        margin = float(min(sides) / max(sides)) if max(sides) > 0 else 0.0
        return data, pts, margin

    def _decode_pyzbar(self, img: np.ndarray) -> tuple[str, np.ndarray | None, float]:
        try:
            from pyzbar import pyzbar
        except ImportError:
            self.get_logger().error("pyzbar가 없다. backend=opencv로 돌린다.")
            return self._decode_opencv(img)
        found = pyzbar.decode(img)
        if not found:
            return "", None, 0.0
        best = found[0]
        pts = np.array([[p.x, p.y] for p in best.polygon], dtype=np.float64)
        if pts.shape[0] != 4:
            return best.data.decode(), None, 0.5
        return best.data.decode(), pts, 0.8

    def _on_read(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        msg = LabelCorners()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.camera}_color_optical_frame"
        msg.source = self.camera

        if self.latest is None:
            msg.ok = False
            self.pub.publish(msg)
            res.success = False
            res.message = "아직 영상이 없다"
            return res

        img = self.latest.copy()
        code, pts, margin = self._decode(img)
        min_side = float(self.get_parameter("min_side_px").value)

        if not code or pts is None:
            msg.ok = False
            self.pub.publish(msg)
            res.success = False
            res.message = "디코드 실패"
            self.get_logger().warn(f"{self.camera} 판독 실패")
            self._publish_debug(img, None, "판독 실패")
            return res

        side = float(np.mean([np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]))
        if side < min_side:
            msg.ok = False
            self.pub.publish(msg)
            res.success = False
            res.message = f"라벨이 {side:.0f} px로 너무 작다 (하한 {min_side:.0f})"
            self._publish_debug(img, pts, res.message)
            return res

        msg.ok = True
        msg.code = code
        msg.corners = [float(v) for v in pts.reshape(-1)]
        msg.decode_margin = margin
        self.pub.publish(msg)

        res.success = True
        res.message = code
        self.get_logger().info(f"{self.camera} 판독 : {code} (한 변 {side:.0f} px, 여유 {margin:.2f})")
        self._publish_debug(img, pts, code)
        return res

    def _publish_debug(self, img: np.ndarray, pts: np.ndarray | None, text: str) -> None:
        if not bool(self.get_parameter("publish_debug").value):
            return
        out = img.copy()
        if pts is not None:
            cv2.polylines(out, [pts.astype(np.int32)], True, (0, 220, 0), 2)
            for i, p in enumerate(pts.astype(int)):
                cv2.circle(out, tuple(p), 4, (0, 120, 255), -1)
                cv2.putText(out, str(i), tuple(p + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 255), 1)
        cv2.putText(out, text, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (30, 30, 220), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(out, encoding="bgr8"))


def main() -> None:
    rclpy.init()
    node = LabelReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
