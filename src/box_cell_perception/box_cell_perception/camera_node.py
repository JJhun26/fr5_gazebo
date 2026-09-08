#!/usr/bin/env python3
"""카메라 한 대의 촬영과 보관.

기획서 5.4의 역할 분담에서 판독은 C1 한 대로 끝나고, 나머지 세 대는
'확인과 기록'을 맡는다. 이 노드가 그 기록 쪽이다.

  /camera/<name>/capture (Trigger)
      지금 프레임을 파일로 남기고 경로를 돌려준다. MES events 테이블의
      image_path가 이 값이다(기획서 5.5).

C2는 적재 결과 확인과 디팔레타이징 전 재확인, 적층 붕괴 감지에 쓴다.
C3는 셀 전경이라 MES 기록 사진과 관람 화면의 소스다.
C4는 판독 실패 시 근접 재시도용이라 label_reader를 한 벌 더 띄워 물린다.

저장 경로는 날짜별로 나눈다. 데모 하루를 통째로 돌리면 수백 장이 쌓이는데,
한 디렉터리에 몰아넣으면 대시보드가 목록을 읽는 데만 오래 걸린다.
"""

from __future__ import annotations

import datetime as dt
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit("opencv-python이 필요하다") from exc


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameter("camera", "c3_scene")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("store", "/tmp/box_cell/photos")
        self.declare_parameter("jpeg_quality", 88)
        self.declare_parameter("max_width", 1280)     # 기록용은 줄여 저장한다

        self.camera = str(self.get_parameter("camera").value)
        topic = str(self.get_parameter("image_topic").value) or f"/{self.camera}/color/image_raw"
        self.store = Path(str(self.get_parameter("store").value))
        self.bridge = CvBridge()
        self.latest = None
        self.info: CameraInfo | None = None
        self.count = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, topic, self._on_image, qos)
        self.create_subscription(CameraInfo, f"/{self.camera}/color/camera_info", self._on_info, qos)
        self.create_service(Trigger, f"/camera/{self.camera}/capture", self._on_capture)
        self.last_pub = self.create_publisher(String, f"/camera/{self.camera}/last_photo", 10)

        self.get_logger().info(f"camera_node({self.camera}) 준비. topic={topic} store={self.store}")

    def _on_image(self, msg: Image) -> None:
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"영상 변환 실패 : {exc}", throttle_duration_sec=5.0)

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_capture(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self.latest is None:
            res.success = False
            res.message = "아직 영상이 없다"
            return res

        img = self.latest
        max_w = int(self.get_parameter("max_width").value)
        if img.shape[1] > max_w:
            scale = max_w / img.shape[1]
            img = cv2.resize(img, (max_w, int(img.shape[0] * scale)), interpolation=cv2.INTER_AREA)

        now = dt.datetime.now()
        day = self.store / now.strftime("%Y-%m-%d")
        day.mkdir(parents=True, exist_ok=True)
        path = day / f"{self.camera}_{now.strftime('%H%M%S_%f')[:-3]}.jpg"
        cv2.imwrite(
            str(path), img, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.get_parameter("jpeg_quality").value)]
        )

        self.count += 1
        self.last_pub.publish(String(data=str(path)))
        res.success = True
        res.message = str(path)
        self.get_logger().info(f"촬영 {self.count} : {path}")
        return res


def main() -> None:
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
