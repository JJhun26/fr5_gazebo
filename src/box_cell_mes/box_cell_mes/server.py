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
      GET  /tables         테이블 목록과 스키마, 행 수
      GET  /table/{name}   그 테이블의 원본 행 (limit, offset)
      GET  /query?sql=     읽기 전용 질의 (SELECT / WITH / PRAGMA)

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
import os
import sqlite3
import time
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS items (
    code      TEXT PRIMARY KEY,
    name      TEXT NOT NULL,
    category  TEXT,
    note      TEXT,
    -- 박스 정보. 셀이 판독한 코드로 여기를 조회해 규격을 알아낸다.
    -- 규격이 여러 가지가 되면서 MES가 치수의 유일한 원본이 되었다.
    -- 상세는 box_cell_msgs/srv/ItemQuery.srv 주석 참고.
    kind      TEXT,               -- S | M | L | XL
    weight_kg REAL,
    handling  TEXT                -- normal | fragile | hazmat | oversize
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
        self.path = path
        self.db = sqlite3.connect(str(path), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA)
        self._migrate()
        self.db.commit()

    def _migrate(self) -> None:
        """이미 있는 DB에 뒤늦게 생긴 열을 붙인다.

        DB 파일은 도커 볼륨에 남아 실행 사이에 살아 있다. 그래서
        CREATE TABLE IF NOT EXISTS는 옛 파일을 고치지 못한다. 규격/중량/
        취급 열을 추가했을 때 실제로 seed가 'no column named kind'로 죽어
        MES가 통째로 안 떴고, 셀은 모든 코드를 미등록으로 보고 박스를
        전부 예외 통에 버렸다.
        """
        want = {
            "items": {"kind": "TEXT", "weight_kg": "REAL", "handling": "TEXT"},
        }
        for table, cols in want.items():
            have = {r["name"] for r in self.db.execute(f"PRAGMA table_info({table})")}
            for col, typ in cols.items():
                if col not in have:
                    self.db.execute(f"ALTER TABLE {table} ADD COLUMN {col} {typ}")

    def seed(self, items_json: Path) -> int:
        if not items_json.exists():
            return 0
        rows = json.loads(items_json.read_text())
        self.db.executemany(
            "INSERT OR REPLACE INTO items(code, name, category, note, kind, weight_kg,"
            " handling) VALUES (?,?,?,?,?,?,?)",
            [
                (
                    r["code"],
                    r["name"],
                    r.get("category", ""),
                    r.get("note", ""),
                    r.get("kind", ""),
                    float(r.get("weight_kg", 0.0)),
                    r.get("handling", "normal"),
                )
                for r in rows
            ],
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

    # ------------------------------------------------------------ DB 브라우저
    #
    # 대시보드가 집계만 보여 주면 MES가 아니라 상태 표시등이다. 운영자가
    # "그 코드가 언제 뭐로 들어왔지"를 물을 수 있어야 데이터베이스다.
    # 그래서 스키마, 테이블 원본, 읽기 전용 질의를 함께 연다.

    def tables(self) -> list[dict]:
        """테이블 목록과 스키마. sqlite_ 로 시작하는 내부 테이블은 뺀다."""
        out = []
        names = [
            r["name"] for r in self.db.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
                " AND name NOT LIKE 'sqlite_%' ORDER BY name"
            )
        ]
        for name in names:
            cols = [
                {
                    "name": c["name"],
                    "type": c["type"] or "",
                    "pk": bool(c["pk"]),
                    "notnull": bool(c["notnull"]),
                }
                # 테이블 이름은 sqlite_master에서 온 것이라 사용자 입력이 아니다.
                for c in self.db.execute(f"PRAGMA table_info({name})")
            ]
            rows = self.db.execute(f"SELECT COUNT(*) n FROM {name}").fetchone()["n"]
            out.append({"name": name, "rows": rows, "columns": cols})
        return out

    def table_rows(self, table: str, limit: int = 50, offset: int = 0) -> dict:
        """테이블 한 쪽. 최신 것이 위로 오게 rowid 역순으로 낸다."""
        known = {t["name"] for t in self.tables()}
        if table not in known:
            return {"error": f"모르는 테이블 '{table}'"}
        limit = max(1, min(int(limit), 500))
        offset = max(0, int(offset))
        total = self.db.execute(f"SELECT COUNT(*) n FROM {table}").fetchone()["n"]
        rows = self.db.execute(
            f"SELECT * FROM {table} ORDER BY rowid DESC LIMIT ? OFFSET ?",
            (limit, offset),
        ).fetchall()
        cols = [c["name"] for c in self.db.execute(f"PRAGMA table_info({table})")]
        return {
            "table": table, "total": total, "limit": limit, "offset": offset,
            "columns": cols, "rows": [dict(r) for r in rows],
        }

    def query(self, sql: str, limit: int = 200) -> dict:
        """읽기 전용 질의.

        운영자가 직접 물어볼 수 있어야 데이터베이스다. 그런데 대시보드는
        인증이 없으므로 **쓰기는 절대 열지 않는다**. 막는 방법이 두 겹이다.

          1. 문장을 SELECT/WITH/PRAGMA 하나로 제한한다. 세미콜론으로 문장을
             더 붙이는 것도 막는다.
          2. 그것을 통과해도 **읽기 전용으로 다시 연 커넥션**에서 실행한다.
             1번을 뚫는 표현이 있더라도 SQLite가 쓰기를 거부한다. 걸러 내기만
             믿지 않는 것이 요점이다.
        """
        text = (sql or "").strip().rstrip(";").strip()
        if not text:
            return {"error": "질의가 비었다"}
        if ";" in text:
            return {"error": "문장은 하나만 된다"}
        head = text.split(None, 1)[0].upper()
        if head not in ("SELECT", "WITH", "PRAGMA"):
            return {"error": "읽기 전용이다. SELECT / WITH / PRAGMA만 된다."}
        # PRAGMA는 스키마를 들여다보는 것만 허용한다. 값을 넣는 형태
        # (PRAGMA foo=bar)는 설정을 바꾸는 쪽이다. 읽기 전용 커넥션이라
        # 실제로 파일을 건드리지는 못하지만, 열어 둘 이유가 없다.
        if head == "PRAGMA" and "=" in text:
            return {"error": "PRAGMA는 조회 형태만 된다"}

        limit = max(1, min(int(limit), 1000))
        t0 = time.time()
        try:
            ro = sqlite3.connect(f"file:{self.path}?mode=ro", uri=True)
            ro.row_factory = sqlite3.Row
            try:
                cur = ro.execute(text)
                rows = cur.fetchmany(limit)
                cols = [d[0] for d in cur.description] if cur.description else []
                more = cur.fetchone() is not None
            finally:
                ro.close()
        except sqlite3.Error as exc:
            return {"error": str(exc)}
        return {
            "columns": cols,
            "rows": [dict(r) for r in rows],
            "truncated": more,
            "elapsed_ms": round((time.time() - t0) * 1000, 1),
        }

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
            from urllib.parse import parse_qs, unquote, urlsplit
            parts = urlsplit(self.path)
            path = parts.path
            q = parse_qs(parts.query)

            def arg(name: str, default: str = "") -> str:
                return q.get(name, [default])[0]

            if path == "/health":
                self._json(200, {"ok": True, "db": str(store.path)})
            elif path.startswith("/items/"):
                item = store.item(unquote(path.rsplit("/", 1)[1]))
                self._json(200 if item else 404, item or {"error": "not found"})
            elif path == "/events":
                self._json(200, store.events(int(arg("limit", "100"))))
            elif path == "/stats":
                self._json(200, store.stats())
            elif path == "/tables":
                self._json(200, store.tables())
            elif path.startswith("/table/"):
                res = store.table_rows(
                    unquote(path.rsplit("/", 1)[1]),
                    int(arg("limit", "50")), int(arg("offset", "0")),
                )
                self._json(400 if "error" in res else 200, res)
            elif path == "/query":
                res = store.query(arg("sql"), int(arg("limit", "200")))
                self._json(400 if "error" in res else 200, res)
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
        return {"ok": True, "db": str(store.path)}

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

    @app.get("/tables")
    def get_tables() -> list[dict]:
        return store.tables()

    @app.get("/table/{name}")
    def get_table(name: str, limit: int = 50, offset: int = 0) -> dict:
        res = store.table_rows(name, limit, offset)
        if "error" in res:
            raise HTTPException(status_code=400, detail=res["error"])
        return res

    @app.get("/query")
    def get_query(sql: str, limit: int = 200) -> dict:
        res = store.query(sql, limit)
        if "error" in res:
            raise HTTPException(status_code=400, detail=res["error"])
        return res

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
    # 산출물 위치는 BOX_CELL_DATA_DIR로 옮길 수 있다. 이 패키지는 ROS 없이도
    # 돌아야 해서(python3 server.py) box_cell_common을 부르지 않고 환경 변수만 본다.
    data_dir = Path(os.environ.get("BOX_CELL_DATA_DIR") or "/tmp/box_cell")
    ap.add_argument("--db", type=Path, default=data_dir / "mes.sqlite3")
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
