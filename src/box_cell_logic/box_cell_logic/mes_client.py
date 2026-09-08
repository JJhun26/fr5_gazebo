#!/usr/bin/env python3
"""MES 연동. 기획서 4절의 하이브리드 구조를 그대로 따른다.

    "MES는 하이브리드로 둔다. 정본은 서버, 엣지의 mes-buffer가 통신 두절 시
     로컬 큐에 쌓았다가 재전송한다. event_id를 멱등키로 써서 중복 기록을 막는다."

그래서 이 노드는 두 가지 일만 한다.
  1. /mes/event 토픽으로 들어온 이벤트를 서버에 올린다.
     실패하면 로컬 큐에 넣고, 다음 기회에 다시 보낸다.
  2. /mes/item 서비스로 품목을 조회한다. 서버가 죽어 있으면 캐시로 답한다.

중요한 것은 MES가 죽어도 로봇이 서지 않는다는 점이다. 무인 연속 운전이
전제이므로, 기록을 못 남기는 것과 셀이 멈추는 것은 전혀 다른 사고다.
event_id는 셀 이름 + 단조 증가 번호로 만든다. 재전송해도 서버가 같은
event_id를 두 번 기록하지 않는다.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import rclpy
from box_cell_msgs.srv import ItemQuery
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MesClient(Node):
    def __init__(self) -> None:
        super().__init__("mes_client")

        self.declare_parameter("base_url", "http://127.0.0.1:8020")
        self.declare_parameter("cell_id", "cell-01")
        self.declare_parameter("queue_file", "/tmp/box_cell/mes_queue.jsonl")
        self.declare_parameter("timeout", 1.5)
        self.declare_parameter("retry_sec", 5.0)

        self.base = str(self.get_parameter("base_url").value).rstrip("/")
        self.cell_id = str(self.get_parameter("cell_id").value)
        self.queue_file = Path(str(self.get_parameter("queue_file").value))
        self.queue_file.parent.mkdir(parents=True, exist_ok=True)

        self._seq = self._restore_seq()
        self._lock = threading.Lock()
        self.online = False
        self.item_cache: dict[str, dict] = {}

        self.online_pub = self.create_publisher(Bool, "/mes/online", 10)
        self.create_subscription(String, "/mes/event", self._on_event, 20)
        self.create_service(ItemQuery, "/mes/item", self._on_item)
        self.create_timer(float(self.get_parameter("retry_sec").value), self._flush)
        self.create_timer(2.0, self._ping)

        self.get_logger().info(f"mes_client 준비. 서버 {self.base}, 큐 {self.queue_file}")

    # ------------------------------------------------------------------ HTTP
    def _request(self, path: str, payload: dict | None = None) -> dict | None:
        url = f"{self.base}{path}"
        data = json.dumps(payload).encode() if payload is not None else None
        req = urllib.request.Request(
            url, data=data, headers={"Content-Type": "application/json"},
            method="POST" if data else "GET",
        )
        try:
            with urllib.request.urlopen(
                req, timeout=float(self.get_parameter("timeout").value)
            ) as resp:
                return json.loads(resp.read().decode() or "{}")
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError):
            return None

    def _ping(self) -> None:
        was = self.online
        self.online = self._request("/health") is not None
        self.online_pub.publish(Bool(data=self.online))
        if was != self.online:
            self.get_logger().info("MES 서버 " + ("연결됨" if self.online else "끊김. 로컬 큐에 쌓는다."))

    # ------------------------------------------------------------------ 이벤트
    def _restore_seq(self) -> int:
        if not self.queue_file.exists():
            return 0
        last = 0
        for line in self.queue_file.read_text().splitlines():
            try:
                last = max(last, int(json.loads(line)["seq"]))
            except Exception:  # noqa: BLE001
                continue
        return last

    def _on_event(self, msg: String) -> None:
        parts = msg.data.split("|", 2)
        kind = parts[0] if parts else ""
        code = parts[1] if len(parts) > 1 else ""
        extra = parts[2] if len(parts) > 2 else ""

        with self._lock:
            self._seq += 1
            seq = self._seq
        event = {
            "event_id": f"{self.cell_id}-{seq:08d}",
            "seq": seq,
            "ts": time.time(),
            "cell": self.cell_id,
            "type": kind,
            "code": code,
            "extra": extra,
        }
        if self._post_event(event):
            return
        self._enqueue(event)

    def _post_event(self, event: dict) -> bool:
        ok = self._request("/events", event) is not None
        if not ok:
            self.get_logger().warn(
                f"이벤트 {event['event_id']} 전송 실패. 큐에 넣는다.", throttle_duration_sec=10.0
            )
        return ok

    def _enqueue(self, event: dict) -> None:
        with self._lock, self.queue_file.open("a") as f:
            f.write(json.dumps(event, ensure_ascii=False) + "\n")

    def _flush(self) -> None:
        """쌓인 것을 다시 보낸다. event_id가 멱등키라 중복 걱정이 없다."""
        if not self.online or not self.queue_file.exists():
            return
        with self._lock:
            lines = self.queue_file.read_text().splitlines()
        if not lines:
            return

        remaining: list[str] = []
        sent = 0
        for line in lines:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if self._post_event(event):
                sent += 1
            else:
                remaining.append(line)
        with self._lock:
            self.queue_file.write_text("\n".join(remaining) + ("\n" if remaining else ""))
        if sent:
            self.get_logger().info(f"보류 이벤트 {sent}건 재전송, {len(remaining)}건 남음")

    # ------------------------------------------------------------------ 품목
    def _on_item(self, req: ItemQuery.Request, res: ItemQuery.Response) -> ItemQuery.Response:
        data = self._request(f"/items/{req.code}")
        if data and data.get("code"):
            self.item_cache[req.code] = data
            res.found = True
            res.from_cache = False
        elif req.code in self.item_cache:
            data = self.item_cache[req.code]
            res.found = True
            res.from_cache = True
        else:
            res.found = False
            return res
        res.name = data.get("name", "")
        res.category = data.get("category", "")
        res.note = data.get("note", "")
        return res


def main() -> None:
    rclpy.init()
    node = MesClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
