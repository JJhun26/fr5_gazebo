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

촬영은 "트리거 이후에 찍힌 프레임"만 쓴다. 이게 실물 라인의 동작이다.
산업용 카메라는 정지 신호를 받고 나서 셔터를 연다. 마지막으로 도착한
프레임을 그냥 집으면 파이프라인(렌더 -> 브리지 -> 구독)에 고인 옛 그림을
쓰게 된다. 실측으로 그 지연이 0.55 s였고, 벨트 속도 0.25 m/s에서 138 mm
어긋난 위치를 실제 위치라고 믿었다. 박스는 x=440에 서 있는데 판독은
x=302를 내놓았고, 로봇은 아무것도 없는 자리로 내려갔다.

판독 실패는 정상 동작의 일부다. 기획서는 3회까지 재시도하고 그래도 안 되면
예외 통으로 보낸다. 이 노드는 세는 일을 하지 않는다. 매 요청에 대해
성공이든 실패든 한 번 답할 뿐이고, 세는 것은 task_manager가 한다.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from box_cell_common.paths import data_dir
from box_cell_msgs.msg import LabelCorners
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import Trigger

try:
    import cv2
except ImportError as exc:  # pragma: no cover - 런타임 환경 문제
    raise SystemExit("opencv-python이 필요하다") from exc


class LabelReader(Node):
    def __init__(self) -> None:
        super().__init__("label_reader")

        self.declare_parameter("camera", "c1_conveyor")
        # 압축 토픽을 받는다. 원본(image_raw)이 아니다.
        #
        # 1920x1080 RGB8 한 장이 6.22 MB인데 이 호스트의 net.core.rmem_max는
        # 4 MB다. 한 장이 소켓 버퍼보다 커서 UDP 조각이 다 못 들어오고,
        # 실측으로 브리지가 낸 것의 20~30%만 도착했다. 그 상태에서는 트리거
        # 후 3초를 기다려도 새 프레임이 안 와 판독이 통째로 실패한다.
        # 카메라 주기를 낮춰도 소용없다. 장당 크기가 문제이기 때문이다.
        #
        # JPEG로 오면 장당 200~400 KB라 버퍼에 넉넉히 들어간다. 해상도를
        # 깎지 않아도 되고(1920x1080은 D435f 실제 제원이다), 실물 산업
        # 카메라도 압축해 보낸다. 화질은 image_bridge의 jpeg_quality로
        # 잡는다. QR 모듈 경계가 뭉개지면 안 되므로 95로 둔다.
        self.declare_parameter("image_topic", "/c1_conveyor/color/image/compressed")
        self.declare_parameter("backend", "opencv")     # opencv | pyzbar
        self.declare_parameter("min_side_px", 40.0)     # 이보다 작으면 믿지 않는다
        self.declare_parameter("publish_debug", True)
        # 트리거 후 새 프레임을 기다리는 한도. 카메라 주기의 몇 배로 잡는다.
        self.declare_parameter("grab_timeout", 3.0)

        self.camera = str(self.get_parameter("camera").value)
        self.bridge = CvBridge()
        self.latest: np.ndarray | None = None
        self.latest_stamp: float = 0.0      # 촬영 시각(헤더). 수신 시각이 아니다.
        self.detector = cv2.QRCodeDetector()
        self._fail_dir = data_dir() / "read_fail"
        self._fail_n = 0

        # 서비스가 새 프레임을 기다리는 동안에도 영상 콜백은 돌아야 한다.
        # 같은 그룹에 두면 기다리는 동안 프레임이 안 들어와 영원히 막힌다.
        self.img_group = MutuallyExclusiveCallbackGroup()
        self.srv_group = MutuallyExclusiveCallbackGroup()

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        topic = str(self.get_parameter("image_topic").value)
        self.compressed = topic.endswith("/compressed")
        self.create_subscription(
            CompressedImage if self.compressed else Image,
            topic,
            self._on_image,
            sensor_qos,
            callback_group=self.img_group,
        )
        self.pub = self.create_publisher(LabelCorners, f"/perception/{self.camera}/corners", 10)
        self.debug_pub = self.create_publisher(Image, f"/perception/{self.camera}/debug", 1)
        self.create_service(
            Trigger, f"/perception/{self.camera}/read", self._on_read,
            callback_group=self.srv_group,
        )

        self.get_logger().info(
            f"label_reader({self.camera}) 준비. "
            f"backend={self.get_parameter('backend').value}"
        )

    def _on_image(self, msg) -> None:
        try:
            if self.compressed:
                img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"영상 변환 실패 : {exc}")
            return
        # 헤더 시각은 gz 센서가 이 그림을 찍은 시뮬레이션 시각이다.
        # 이 노드에 도착한 시각과는 다르고, 그 차이가 곧 파이프라인 지연이다.
        self.latest = img
        self.latest_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _grab(self) -> tuple[np.ndarray | None, float]:
        """트리거 이후에 찍힌 프레임 하나를 집는다.

        실물 트리거 촬영과 같다. 지금 고여 있는 그림은 버린다.
        돌려주는 둘째 값은 그 프레임을 얼마나 기다렸는지(= 파이프라인 지연)다.
        """
        trigger = self.get_clock().now().nanoseconds * 1e-9
        limit = float(self.get_parameter("grab_timeout").value)
        t0 = time.monotonic()
        while time.monotonic() - t0 < limit:
            if self.latest is not None and self.latest_stamp > trigger:
                return self.latest.copy(), time.monotonic() - t0
            time.sleep(0.01)
        return None, time.monotonic() - t0

    # ------------------------------------------------------------------ 판독
    def _decode(self, img: np.ndarray) -> tuple[str, np.ndarray | None, float]:
        backend = str(self.get_parameter("backend").value)
        if backend == "pyzbar":
            return self._decode_pyzbar(img)
        return self._decode_opencv(img)

    def _decode_opencv(self, img: np.ndarray) -> tuple[str, np.ndarray | None, float]:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        data, points, _ = self.detector.detectAndDecode(gray)

        if (not data) and points is not None:
            # 찾기는 했는데 못 읽은 경우다. 그림을 통째로 던지면 QR이
            # 1920x1080 안에서 150 px 남짓이라 모듈 하나가 5~6 px이고,
            # 렌더러의 밉맵이 그 경계를 흐린다. 그 상태에서 디코더가
            # 자체 축소까지 하면 모듈이 뭉개져 읽지 못한다.
            #
            # QR 자리만 잘라 3배로 키워 다시 던진다. 정보가 늘어나지는
            # 않지만 디코더가 모듈 격자를 잡을 여지가 생긴다. 실측으로
            # 이 한 번의 재시도가 실패 프레임을 전부 살렸다.
            #
            # 좌표는 원본 그림 기준을 그대로 쓴다. pose_resolver의
            # 호모그래피가 원본 픽셀을 전제하기 때문이다. 확대본은
            # 문자열을 되찾는 데만 쓴다.
            data = self._decode_zoomed(gray, points)

        if not data or points is None:
            return "", None, 0.0
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        # 디코드 여유 대신 모서리 사각형의 정직함을 신뢰도로 쓴다.
        # 네 변의 길이가 고르고 대각이 같으면 라벨을 정면에 가깝게 본 것이다.
        sides = [float(np.linalg.norm(pts[i] - pts[(i + 1) % 4])) for i in range(4)]
        margin = float(min(sides) / max(sides)) if max(sides) > 0 else 0.0
        return data, pts, margin

    def _decode_zoomed(self, gray: np.ndarray, points) -> str:
        """찾아 둔 QR 자리만 잘라 키워서 한 번 더 읽는다."""
        try:
            p = np.asarray(points, dtype=np.float64).reshape(-1, 2)
            m = 25
            h, w = gray.shape[:2]
            x0 = max(0, int(p[:, 0].min()) - m)
            y0 = max(0, int(p[:, 1].min()) - m)
            x1 = min(w, int(p[:, 0].max()) + m)
            y1 = min(h, int(p[:, 1].max()) + m)
            if x1 - x0 < 10 or y1 - y0 < 10:
                return ""
            crop = gray[y0:y1, x0:x1]
            big = cv2.resize(crop, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
            return str(self.detector.detectAndDecode(big)[0])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"확대 재시도 실패 : {exc}")
            return ""

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

        img, waited = self._grab()
        if img is None:
            msg.ok = False
            self.pub.publish(msg)
            res.success = False
            res.message = (
                "아직 영상이 없다" if self.latest is None
                else f"트리거 후 새 프레임이 {waited:.2f} s 동안 안 왔다"
            )
            self.get_logger().warn(res.message)
            return res
        self.get_logger().info(f"{self.camera} 트리거 촬영. 프레임 대기 {waited*1000:.0f} ms")

        code, pts, margin = self._decode(img)
        min_side = float(self.get_parameter("min_side_px").value)

        if not code or pts is None:
            msg.ok = False
            self.pub.publish(msg)
            res.success = False
            res.message = "디코드 실패"
            self.get_logger().warn(f"{self.camera} 판독 실패")
            self._publish_debug(img, None, "판독 실패")
            self._dump_failure(img)
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

    def _dump_failure(self, img: np.ndarray) -> None:
        """실패한 그림을 그대로 파일로 남긴다.

        디버그 토픽은 원본 크기라 소켓 버퍼를 넘어 도착하지 않는다.
        실패를 눈으로 보려면 파일로 남기는 수밖에 없다. 최근 것 몇 장만
        돌려 쓴다. 실패가 잦을 때 디스크를 채우면 안 된다.
        """
        if not bool(self.get_parameter("publish_debug").value):
            return
        try:
            self._fail_dir.mkdir(parents=True, exist_ok=True)
            path = self._fail_dir / f"{self.camera}_{self._fail_n % 8}.png"
            cv2.imwrite(str(path), img)
            self._fail_n += 1
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"실패 그림 저장 실패 : {exc}")

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
