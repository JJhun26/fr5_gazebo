#!/usr/bin/env python3
"""MES 서버. 기획서 5.5.

실물 MES가 없으므로 직접 만든다. FastAPI + SQLite로 시작하고 필요하면
PostgreSQL로 바꾼다.

    테이블
      items   code(PK), name, category, note
      events  id, ts, code, type(READ/PLACE/DEPAL/EXCEPTION/SWAP), pallet,
              layer, idx, image_path

    API
      GET  /items/{code}   품목 조회. 판독 직후 호출.
      POST /events         이벤트 기록. 사진 경로와 시각 포함.
      GET  /events         대시보드용 이력
      GET  /stats          집계

event_id를 UNIQUE로 잡는 것이 요점이다. 엣지의 mes_client가 통신 두절 뒤
재전송할 때 같은 이벤트가 두 번 들어오는데, 여기서 조용히 무시해야 한다.
기획서 4절의 "event_id를 멱등키로 써서 중복 기록을 막는다"가 이 한 줄이다.

FastAPI가 없는 환경을 위해 표준 라이브러리 http.server 폴백을 함께 둔다.
데모 PC에 무엇이 깔려 있을지 확실하지 않은 상태에서 대시보드가 안 뜨는 것이
가장 곤란하다.
"""

from __future__ import annotations

import argparse
import json
import sqlite3
import time
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS items (
    code     TEXT PRIMARY KEY,
    name     TEXT NOT NULL,
    category TEXT,
    note     TEXT
);
CREATE TABLE IF NOT EXISTS events (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id  TEXT UNIQUE,          -- 멱등키. 재전송해도 한 번만 남는다.
    ts        REAL NOT NULL,
    cell      TEXT,
    code      TEXT,
    type      TEXT NOT NULL,
    pallet    INTEGER,
    layer     INTEGER,
    idx       INTEGER,
    image_path TEXT,
    extra     TEXT
);
CREATE INDEX IF NOT EXISTS events_ts ON events(ts);
CREATE INDEX IF NOT EXISTS events_code ON events(code);
"""


class Store:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA)
        self.db.commit()

    def seed(self, items_json: Path) -> int:
        if not items_json.exists():
            return 0
        rows = json.loads(items_json.read_text())
        self.db.executemany(
            "INSERT OR REPLACE INTO items(code, name, category, note) VALUES (?,?,?,?)",
            [(r["code"], r["name"], r.get("category", ""), r.get("note", "")) for r in rows],
        )
        self.db.commit()
        return len(rows)

    def item(self, code: str) -> dict | None:
        row = self.db.execute("SELECT * FROM items WHERE code=?", (code,)).fetchone()
        return dict(row) if row else None

    def add_event(self, payload: dict) -> dict:
        # PLACE 이벤트의 extra는 "p2,1,5,/path/to.jpg" 꼴이다.
        pallet = layer = idx = None
        image = ""
        extra = payload.get("extra", "") or ""
        if payload.get("type") == "PLACE":
            parts = extra.split(",")
            try:
                pallet = int(parts[0].lstrip("p"))
                layer = int(parts[1])
                idx = int(parts[2])
                image = parts[3] if len(parts) > 3 else ""
            except (ValueError, IndexError):
                pass
        try:
            self.db.execute(
                "INSERT INTO events(event_id, ts, cell, code, type, pallet, layer, idx,"
                " image_path, extra) VALUES (?,?,?,?,?,?,?,?,?,?)",
                (
                    payload.get("event_id"),
                    float(payload.get("ts", time.time())),
                    payload.get("cell", ""),
                    payload.get("code", ""),
                    payload.get("type", ""),
                    pallet, layer, idx, image, extra,
                ),
            )
            self.db.commit()
            return {"ok": True, "duplicate": False}
        except sqlite3.IntegrityError:
            # 같은 event_id가 이미 있다. 재전송이다. 조용히 넘어간다.
            return {"ok": True, "duplicate": True}

    def events(self, limit: int = 100) -> list[dict]:
        rows = self.db.execute(
            "SELECT * FROM events ORDER BY id DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in rows]

    def stats(self) -> dict:
        def one(sql: str, *args):
            row = self.db.execute(sql, args).fetchone()
            return row[0] if row else 0

        counts = {
            r["type"]: r["n"]
            for r in self.db.execute("SELECT type, COUNT(*) n FROM events GROUP BY type")
        }
        first = one("SELECT MIN(ts) FROM events")
        last = one("SELECT MAX(ts) FROM events")
        placed = counts.get("PLACE", 0)
        elapsed = (last - first) if (first and last and last > first) else 0.0
        return {
            "by_type": counts,
            "placed": placed,
            "exceptions": counts.get("EXCEPTION", 0),
            "elapsed_sec": elapsed,
            "cycle_sec": (elapsed / placed) if placed else 0.0,
            "read_rate": (
                counts.get("READ", 0) / (counts.get("READ", 0) + counts.get("EXCEPTION", 0))
                if (counts.get("READ", 0) + counts.get("EXCEPTION", 0))
                else 0.0
            ),
        }


# --------------------------------------------------------------------- 폴백 서버
def run_stdlib(store: Store, host: str, port: int, static: Path) -> None:
    """FastAPI가 없을 때. 같은 경로, 같은 응답."""
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, body: bytes, ctype: str = "application/json") -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, obj) -> None:
            self._send(code, json.dumps(obj, ensure_ascii=False).encode())

        def log_message(self, *_args) -> None:  # 조용히
            pass

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?")[0]
            if path == "/health":
                self._json(200, {"ok": True})
            elif path.startswith("/items/"):
                item = store.item(path.rsplit("/", 1)[1])
                self._json(200 if item else 404, item or {"error": "not found"})
            elif path == "/events":
                self._json(200, store.events())
            elif path == "/stats":
                self._json(200, store.stats())
            elif path in ("/", "/index.html"):
                page = static / "dashboard.html"
                if page.exists():
                    self._send(200, page.read_bytes(), "text/html; charset=utf-8")
                else:
                    self._json(404, {"error": "dashboard.html이 없다"})
            else:
                self._json(404, {"error": "unknown path"})

        def do_POST(self) -> None:  # noqa: N802
            length = int(self.headers.get("Content-Length", 0))
            try:
                payload = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                self._json(400, {"error": "bad json"})
                return
            if self.path == "/events":
                self._json(200, store.add_event(payload))
            else:
                self._json(404, {"error": "unknown path"})

    print(f"MES(표준 라이브러리) http://{host}:{port}")
    ThreadingHTTPServer((host, port), Handler).serve_forever()


def run_fastapi(store: Store, host: str, port: int, static: Path) -> None:
    from fastapi import FastAPI, HTTPException
    from fastapi.responses import FileResponse
    import uvicorn

    app = FastAPI(title="box_cell MES", version="0.1.0")

    @app.get("/health")
    def health() -> dict:
        return {"ok": True}

    @app.get("/items/{code}")
    def get_item(code: str) -> dict:
        item = store.item(code)
        if not item:
            raise HTTPException(status_code=404, detail="not found")
        return item

    @app.post("/events")
    def post_event(payload: dict) -> dict:
        return store.add_event(payload)

    @app.get("/events")
    def get_events(limit: int = 100) -> list[dict]:
        return store.events(limit)

    @app.get("/stats")
    def get_stats() -> dict:
        return store.stats()

    @app.get("/")
    def dashboard():
        page = static / "dashboard.html"
        if not page.exists():
            raise HTTPException(status_code=404, detail="dashboard.html이 없다")
        return FileResponse(page)

    print(f"MES(FastAPI) http://{host}:{port}")
    uvicorn.run(app, host=host, port=port, log_level="warning")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", type=Path, default=Path("/tmp/box_cell/mes.sqlite3"))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8020)
    ap.add_argument("--seed", type=Path, default=None)
    ap.add_argument("--static", type=Path, default=None)
    args = ap.parse_args()

    here = Path(__file__).resolve().parent
    seed = args.seed or (here / "seed_items.json")
    static = args.static or (here.parent / "static")

    store = Store(args.db)
    n = store.seed(seed)
    print(f"품목 {n}건 적재, DB {args.db}")

    try:
        run_fastapi(store, args.host, args.port, static)
    except ImportError:
        print("fastapi/uvicorn이 없다. 표준 라이브러리 서버로 돌린다.")
        run_stdlib(store, args.host, args.port, static)


if __name__ == "__main__":
    main()
