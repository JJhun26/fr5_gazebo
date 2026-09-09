#!/usr/bin/env python3
"""판독 성공률과 정확도를 잰다. 조명을 손볼 때 A/B 하는 도구다.

성공률만 봐서는 안 된다. 조명을 낮추면 디코드는 잘 되는데 QR 모서리가
흐려져 위치가 밀릴 수 있다. 흡착 판정은 컵 축에서 30 mm 안을 요구하므로
위치가 밀리면 디코드에 성공하고도 헛집는다. 실제로 그렇게 당했다.
그래서 정답지(/sim/boxes)와 대 본 오차를 함께 낸다.

로봇을 움직이지 않고 판독만 반복하므로 1분이면 답이 나온다. 데모를 10분씩
돌려 사이클 로그로 성공률을 세는 것보다 훨씬 빠르고, 로봇 동작의 실패가
섞이지 않아 원인이 조명인지 아닌지가 분명하게 갈린다.

    ros2 launch box_cell_bringup demo.launch.py headless:=true autostart:=false
    python3 tools/measure_read_rate.py [횟수]

박스를 하나 벨트에 올려 정지 센서까지 보낸 뒤, 그 자리에 세워 둔 채
판독을 N번 시도한다. 매 시도가 새 프레임을 트리거하므로(label_reader가
트리거 이후 프레임만 쓴다) 렌더 노이즈까지 포함한 실제 성공률이 나온다.

함께 내는 노출 수치는 판독 구간(박스 상면이 있는 화면 중앙 띠)의 것이다.
전체 화면 평균은 벨트와 배경에 묻혀 쓸모가 없다.
"""

from __future__ import annotations

import sys
import time

import numpy as np
import rclpy
from box_cell_msgs.msg import BoxPoseArray, LabelDetection
from box_cell_msgs.srv import BeltCommand
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

try:
    import cv2
except ImportError:  # pragma: no cover
    raise SystemExit("opencv-python이 필요하다")


def main() -> int:
    tries = int(sys.argv[1]) if len(sys.argv) > 1 else 30

    rclpy.init()
    node = Node("measure_read_rate")
    bridge = CvBridge()
    latest: dict[str, np.ndarray] = {}

    # label_reader가 내는 디버그 화면을 쓴다. 카메라 원본 토픽은 브리지가
    # 구독자가 붙을 때만 흘려 주어 이 짧은 측정 중에 안 올 때가 있다.
    # 디버그 화면은 판독 때마다 반드시 나오므로 확실하다.
    node.create_subscription(
        Image, "/perception/c1_conveyor/debug",
        lambda m: latest.__setitem__("img", bridge.imgmsg_to_cv2(m, "bgr8")), 10
    )
    truth: dict[str, tuple[float, float]] = {}
    node.create_subscription(
        BoxPoseArray, "/sim/boxes",
        lambda m: truth.update({b.name: (b.pose.position.x, b.pose.position.y)
                                for b in m.boxes}), 10
    )
    seen: list[tuple[float, float]] = []
    node.create_subscription(
        LabelDetection, "/perception/label",
        lambda m: seen.append((m.center_x, m.center_y)) if m.ok else None, 10
    )
    read = node.create_client(Trigger, "/perception/c1_conveyor/read")
    feed = node.create_client(Trigger, "/feeder/next")
    belt = node.create_client(BeltCommand, "/belt/command")
    for cli, name in ((read, "판독"), (feed, "피더"), (belt, "벨트")):
        if not cli.wait_for_service(timeout_sec=20.0):
            print(f"{name} 서비스가 없다")
            return 1

    def call(cli, req, timeout=20.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result()

    # 박스 하나를 판독 자리까지 보낸다
    req = BeltCommand.Request()
    req.command = BeltCommand.Request.FEED_ONE
    call(belt, req)
    call(feed, Trigger.Request())
    print("박스를 판독 자리로 보내는 중...")
    deadline = time.time() + 40
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    req.command = BeltCommand.Request.STOP
    call(belt, req)
    time.sleep(1.0)

    ok = 0
    codes: set[str] = set()
    for i in range(tries):
        res = call(read, Trigger.Request())
        good = bool(res and res.success)
        ok += good
        if good:
            codes.add(res.message)
        rclpy.spin_once(node, timeout_sec=0.1)

    img = latest.get("img")
    print(f"\n판독 {ok}/{tries} 성공 ({100*ok/tries:.0f}%)  코드 {sorted(codes) or '없음'}")

    # 판독 자리에 있는 박스의 정답 좌표와 대 본다
    if seen and truth:
        from box_cell_common.cell_geometry import CellGeometry
        sx, sy = CellGeometry().read_station_xy()
        cand = min(truth.items(), key=lambda kv: abs(kv[1][0] - sx) + abs(kv[1][1] - sy))
        tx, ty = cand[1]
        ex = [abs(p[0] - tx) * 1000 for p in seen]
        ey = [abs(p[1] - ty) * 1000 for p in seen]
        print(f"위치 정확도  {cand[0]} 정답 ({tx*1000:.1f}, {ty*1000:.1f}) mm")
        print(f"  x 오차 평균 {sum(ex)/len(ex):5.1f} 최대 {max(ex):5.1f} mm")
        print(f"  y 오차 평균 {sum(ey)/len(ey):5.1f} 최대 {max(ey):5.1f} mm")
        print("  흡착 판정이 컵 축에서 30 mm 안을 요구하므로 최대 오차가 여기 못 미쳐야 한다")
    if img is None:
        print("영상을 못 받아 노출은 재지 못했다")
        return 0 if ok else 1

    h, w = img.shape[:2]
    band = cv2.cvtColor(img[h // 3: 2 * h // 3, w // 4: 3 * w // 4], cv2.COLOR_BGR2GRAY)
    print(f"판독 구간 노출  평균 {band.mean():5.1f}  "
          f"포화(>=250) {100 * (band >= 250).mean():4.1f}%  "
          f"어두움(<=5) {100 * (band <= 5).mean():4.1f}%")
    print("  평균 110~160, 포화 5% 미만이면 QR 디코드에 넉넉하다")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
