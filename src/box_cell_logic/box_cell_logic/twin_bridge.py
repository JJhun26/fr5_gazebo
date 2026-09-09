#!/usr/bin/env python3
"""디지털 트윈 브리지 (기획서 D4).

무엇을 하는가.
    셀의 상태를 ROS 밖으로 한 창구에 모아 낸다. 지금 셀이 무엇을 하고
    있는지, 팔레트에 무엇이 어떻게 쌓여 있는지, 방금 무슨 일이 있었는지를
    JSON 한 덩어리로 만들어

      1. /tmp/box_cell/twin.json      파일 (누구나 읽는다)
      2. /twin/state                  ROS 토픽 (std_msgs/String, JSON)
      3. http://<host>:8030/twin      HTTP GET (관제 화면, 외부 시스템)
      4. ws://<host>:8030/ws          WebSocket, 바뀔 때마다 밀어 준다

    낼 뿐이고 받지 않는다. 트윈이 셀을 조종하면 그것은 트윈이 아니라 제2의
    상위 제어기다. 운전 지시는 /demo/start, /demo/stop 한 곳으로만 들어온다.

왜 별도 노드인가.
    task_manager에 HTTP를 얹으면 안 된다. 그 노드는 상태 기계를 도는 스레드
    하나로 사이클을 굴린다. 거기에 소켓 대기가 끼면 사이클이 멈춘다. 실제로
    MES 조회를 동기로 부르는 것만으로도 판독 뒤 수백 ms가 붙었다.

    그리고 트윈은 죽어도 되는 것이다. 관제 화면이 끊긴다고 로봇이 서면
    안 된다. 프로세스를 나눠 두면 그 성질이 구조로 보장된다.

무엇을 위한 것인가.
    기획서 D4는 "실물과 시뮬레이션이 같은 화면을 보게 한다"이다. 이 노드가
    내는 스키마는 시뮬레이션인지 실물인지에 의존하지 않는다. 실물 셀에서도
    같은 토픽(/cell/state, /pallet/state, /mes/event)이 나오면 이 노드를
    그대로 띄워 같은 JSON을 얻는다. 그것이 트윈의 최소 조건이다.
    `source` 항목에 어느 쪽인지 적어 화면이 구분할 수 있게 한다.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from box_cell_msgs.msg import BoxPoseArray, CellState, PalletState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class TwinBridge(Node):
    def __init__(self) -> None:
        super().__init__("twin_bridge")

        self.declare_parameter("out_file", "/tmp/box_cell/twin.json")
        self.declare_parameter("http_port", 8030)
        self.declare_parameter("serve_http", True)
        # 시뮬레이션인지 실물인지. 실물 셀에서 띄울 때 real로 바꾼다.
        self.declare_parameter("source", "sim")
        # 이벤트를 몇 개까지 들고 있을지. 화면이 늦게 붙어도 최근 흐름은 보인다.
        self.declare_parameter("event_history", 50)

        self.out_file = Path(str(self.get_parameter("out_file").value))
        self.source = str(self.get_parameter("source").value)
        self.history = int(self.get_parameter("event_history").value)

        self._lock = threading.Lock()
        self._cell: dict = {}
        self._pallets: dict[int, dict] = {}
        self._boxes: list[dict] = []
        self._events: list[dict] = []
        self._seq = 0
        self._subscribers: list = []      # 웹소켓 큐

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CellState, "/cell/state", self._on_cell, 10)
        self.create_subscription(PalletState, "/pallet/state", self._on_pallet, latched)
        self.create_subscription(BoxPoseArray, "/sim/boxes", self._on_boxes, 10)
        self.create_subscription(String, "/mes/event", self._on_event, 20)

        self.pub = self.create_publisher(String, "/twin/state", latched)
        self.create_timer(0.5, self._tick)

        if bool(self.get_parameter("serve_http").value):
            threading.Thread(target=self._serve, daemon=True).start()

        self.get_logger().info(
            f"twin_bridge 준비. source={self.source}, 파일 {self.out_file}, "
            f"HTTP {self.get_parameter('http_port').value}"
        )

    # ---------------------------------------------------------------- 수집
    def _on_cell(self, msg: CellState) -> None:
        with self._lock:
            self._cell = {
                "state": int(msg.state),
                "state_name": msg.state_name,
                "detail": msg.detail,
                "source_pallet": int(msg.source_pallet),
                "target_pallet": int(msg.target_pallet),
                "cycles": int(msg.cycle_count),
                "last_cycle_sec": round(float(msg.last_cycle_sec), 2),
                "exceptions": int(msg.exception_count),
            }

    def _on_pallet(self, msg: PalletState) -> None:
        items = []
        for code, pose, size, layer in zip(
            msg.codes, msg.poses, msg.sizes, msg.layers
        ):
            items.append({
                "code": code,
                "layer": int(layer),
                # 박스 중심. PalletState의 정의를 그대로 옮긴다.
                "center": [round(pose.position.x, 4),
                               round(pose.position.y, 4),
                               round(pose.position.z, 4)],
                "size": [round(size.x, 4), round(size.y, 4), round(size.z, 4)],
            })
        with self._lock:
            self._pallets[int(msg.pallet_id)] = {
                "pallet_id": int(msg.pallet_id),
                "count": len(items),
                "full": bool(msg.full),
                "items": items,
            }

    def _on_boxes(self, msg: BoxPoseArray) -> None:
        """실물 자세. 시뮬레이션에서만 나온다.

        실물 셀에는 이 토픽이 없다. 없으면 그냥 비어 있고, 화면은 나머지로
        그린다. 트윈 스키마가 시뮬레이션에 의존하지 않게 하는 지점이다.
        """
        out = []
        for b in msg.boxes:
            p = b.pose.position
            out.append({
                "name": b.name,
                "xyz": [round(p.x, 4), round(p.y, 4), round(p.z, 4)],
                "size": [round(b.size.x, 4), round(b.size.y, 4), round(b.size.z, 4)],
                "on_belt": bool(b.on_belt),
                "held": bool(b.held),
                "pallet": int(b.pallet),
            })
        with self._lock:
            self._boxes = out

    def _on_event(self, msg: String) -> None:
        # task_manager._event의 구분자는 파이프다.
        parts = msg.data.split("|", 2)
        with self._lock:
            self._events.append({
                "t": round(time.time(), 3),
                "type": parts[0] if parts else "",
                "code": parts[1] if len(parts) > 1 else "",
                "extra": parts[2] if len(parts) > 2 else "",
            })
            del self._events[: max(0, len(self._events) - self.history)]

    # ---------------------------------------------------------------- 발행
    def _snapshot(self) -> dict:
        with self._lock:
            self._seq += 1
            return {
                "schema": "box_cell.twin/1",
                "source": self.source,
                "seq": self._seq,
                "stamp": round(time.time(), 3),
                "cell": dict(self._cell),
                "pallets": [self._pallets[k] for k in sorted(self._pallets)],
                "boxes": list(self._boxes),
                "events": list(self._events),
            }

    def _tick(self) -> None:
        snap = self._snapshot()
        blob = json.dumps(snap, ensure_ascii=False)
        self.pub.publish(String(data=blob))
        try:
            self.out_file.parent.mkdir(parents=True, exist_ok=True)
            # 원자적으로 바꾼다. 읽는 쪽이 반쯤 쓰인 파일을 보면 안 된다.
            tmp = self.out_file.with_suffix(".json.tmp")
            tmp.write_text(blob, encoding="utf-8")
            tmp.replace(self.out_file)
        except OSError as exc:
            self.get_logger().warn(f"트윈 파일 쓰기 실패 : {exc}", throttle_duration_sec=30.0)
        for q in list(self._subscribers):
            try:
                q.append(blob)
                del q[: max(0, len(q) - 5)]
            except Exception:  # noqa: BLE001
                self._subscribers.remove(q)

    # ------------------------------------------------------------------ HTTP
    def _serve(self) -> None:
        """작은 HTTP 창구. 표준 라이브러리만 쓴다.

        FastAPI를 쓰지 않는 이유는 mes_server와 같다. 이 컨테이너에 없을 수
        있는 것에 관제 화면이 걸리면 안 된다. 필요한 것은 GET 하나다.
        """
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_a):  # 조용히
                return

            def do_GET(self):  # noqa: N802
                if self.path.rstrip("/") in ("/twin", "", "/"):
                    body = json.dumps(bridge._snapshot(), ensure_ascii=False).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    # 관제 화면이 다른 포트에서 뜨는 경우가 많다.
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_response(404)
                self.end_headers()

        port = int(self.get_parameter("http_port").value)
        try:
            ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()
        except OSError as exc:
            self.get_logger().warn(f"트윈 HTTP를 못 열었다({port}) : {exc}. 파일과 토픽은 계속 낸다.")


def main() -> None:
    rclpy.init()
    node = TwinBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
