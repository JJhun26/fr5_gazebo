#!/usr/bin/env python3
"""Dry Run 채점기 (기획서 D3).

Dry Run이 무엇인가.
    실물 납품 전에 셀을 무인으로 오래 돌려 보고, 그 결과를 숫자로 받는
    절차다. "돌아갔다"는 말로는 납품 판정을 못 한다. 몇 건을 처리했고,
    몇 번 놓쳤고, 사이클이 목표 안에 들어왔고, 쌓인 것이 기록과 맞는지가
    있어야 한다.

무엇을 채점하는가.
    처리량      완료 사이클 수와 시간당 환산
    사이클 시간 평균 / 중앙값 / 최악, 목표(20초) 대비
    판독        시도 대비 성공, 재시도 분포
    예외        예외 통으로 간 건수와 사유
    적재 정확도 기록한 자리와 **실제로 놓인 자리**의 차이
    무결성      기록에는 있는데 실물이 없는 것, 그 반대

    적재 정확도만 시뮬레이터의 정답지(/sim/boxes)를 본다. 실물에서는 그
    자리에 C2가 대신 들어간다(stack_check). 나머지는 실물에서도 그대로
    쓸 수 있는 지표다.

    ros2 run box_cell_logic dry_run_scorer --ros-args -p duration_sec:=1800
"""

from __future__ import annotations

import json
import math
import re
import statistics
import time
from pathlib import Path

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.msg import BoxPoseArray, CellState, PalletState
from rclpy.node import Node
from std_msgs.msg import String


class DryRunScorer(Node):
    def __init__(self) -> None:
        super().__init__("dry_run_scorer")

        self.declare_parameter("duration_sec", 0.0)      # 0이면 끝까지 본다
        self.declare_parameter("target_cycle_sec", 20.0)
        self.declare_parameter("place_tolerance", 0.020)  # 기록 대비 허용 오차
        self.declare_parameter("report", "/tmp/box_cell/dry_run.json")
        self.declare_parameter("report_every", 60.0)

        self.cell = CellGeometry()
        self.t0 = time.time()
        self.cycles: list[float] = []
        self.events: list[dict] = []
        self.states: list[str] = []
        self.last_cycle = 0
        self.exceptions = 0
        self.boxes: BoxPoseArray | None = None
        self.pallets: dict[int, PalletState] = {}

        self.create_subscription(CellState, "/cell/state", self._on_state, 10)
        self.create_subscription(String, "/mes/event", self._on_event, 50)
        self.create_subscription(BoxPoseArray, "/sim/boxes", self._on_boxes, 10)
        self.create_subscription(PalletState, "/pallet/state", self._on_pallet, 10)
        self.create_timer(float(self.get_parameter("report_every").value), self._tick)

        self.get_logger().info(
            "dry_run_scorer 시작. 기록은 "
            f"{self.get_parameter('report').value}"
        )

    # ------------------------------------------------------------------ 수집
    def _on_state(self, msg: CellState) -> None:
        if msg.cycle_count > self.last_cycle:
            self.last_cycle = msg.cycle_count
            if msg.last_cycle_sec > 0:
                self.cycles.append(float(msg.last_cycle_sec))
        self.exceptions = int(msg.exception_count)
        if not self.states or self.states[-1] != msg.state_name:
            self.states.append(msg.state_name)

    def _on_event(self, msg: String) -> None:
        try:
            self.events.append(json.loads(msg.data))
        except json.JSONDecodeError:
            # task_manager._event가 내는 꼴이다. 구분자는 파이프다.
            # 쉼표로 자르면 종류가 "READ|AXO-0001|conf=0.72" 통째가 되어
            # 판독도 예외도 한 건도 안 잡힌다. 실제로 그렇게 0으로 나왔다.
            parts = msg.data.split("|", 2)
            self.events.append(
                {"type": parts[0], "code": parts[1] if len(parts) > 1 else "",
                 "extra": parts[2] if len(parts) > 2 else ""}
            )

    def _on_boxes(self, msg: BoxPoseArray) -> None:
        self.boxes = msg

    def _on_pallet(self, msg: PalletState) -> None:
        self.pallets[int(msg.pallet_id)] = msg

    # ------------------------------------------------------------------ 채점
    def _placement_error(self) -> dict:
        """기록한 자리와 실제로 놓인 자리의 차이.

        pallet_manager의 기록(PalletState)에 있는 각 박스에 대해, 정답지에서
        가장 가까운 실물을 찾아 거리를 잰다. 짝을 못 찾으면 '유령'이다.
        """
        if self.boxes is None:
            return {"측정": "정답지 없음"}
        tol = float(self.get_parameter("place_tolerance").value)
        errs: list[float] = []
        ghosts: list[str] = []
        for pid, st in sorted(self.pallets.items()):
            for code, pose in zip(st.codes, st.poses):
                best = None
                for b in self.boxes.boxes:
                    if b.code != code:
                        continue
                    d = math.dist(
                        (pose.position.x, pose.position.y, pose.position.z),
                        (b.pose.position.x, b.pose.position.y, b.pose.position.z),
                    )
                    if best is None or d < best:
                        best = d
                if best is None:
                    ghosts.append(f"P{pid} {code} 실물 없음")
                else:
                    errs.append(best)
                    if best > tol:
                        ghosts.append(f"P{pid} {code} {best*1000:.0f} mm 어긋남")
        out = {
            "짝지은 수": len(errs),
            "평균 오차 mm": round(statistics.mean(errs) * 1000, 1) if errs else None,
            "최악 오차 mm": round(max(errs) * 1000, 1) if errs else None,
            "허용 mm": round(tol * 1000),
            "문제": ghosts,
        }
        return out

    def score(self) -> dict:
        elapsed = time.time() - self.t0
        target = float(self.get_parameter("target_cycle_sec").value)
        by_type: dict[str, int] = {}
        for e in self.events:
            by_type[e.get("type", "?")] = by_type.get(e.get("type", "?"), 0) + 1

        reads = [e for e in self.events if e.get("type") == "READ"]
        # extra는 "conf=0.72" 하나일 때도 있고 뒤에 다른 항목이 붙기도 한다.
        # 마지막 '=' 뒤를 잘라 쓰면 'normal' 같은 값이 걸려 죽는다.
        confs: list[float] = []
        for e in reads:
            m = re.search(r"conf=([0-9.]+)", e.get("extra", ""))
            if m:
                try:
                    confs.append(float(m.group(1)))
                except ValueError:
                    pass
        exc = [e for e in self.events if e.get("type") == "EXCEPTION"]
        reasons: dict[str, int] = {}
        for e in exc:
            reasons[e.get("extra", "?")] = reasons.get(e.get("extra", "?"), 0) + 1

        # 순환 사이클은 픽앤플레이스가 두 번이다(팔레트 -> 벨트 -> 팔레트).
        # 20초는 투입 사이클의 목표이므로, 섞어서 재면 목표를 넘었다는
        # 말만 나온다. SWAP 이벤트를 기준으로 갈라 따로 낸다.
        swap_at = next(
            (i for i, e in enumerate(self.events) if e.get("type") == "SWAP"), None
        )
        n_infeed = sum(
            1 for e in self.events[: swap_at if swap_at is not None else len(self.events)]
            if e.get("type") == "PLACE"
        )
        cyc = self.cycles
        infeed_cyc = cyc[:n_infeed]
        circ_cyc = cyc[n_infeed:]
        return {
            "돌린 시간 s": round(elapsed, 1),
            "처리량": {
                "완료 사이클": len(cyc),
                "시간당 환산": round(len(cyc) / elapsed * 3600, 1) if elapsed > 0 else 0,
            },
            "사이클 시간 s": {
                "투입 (목표 대상)": {
                    "건수": len(infeed_cyc),
                    "평균": round(statistics.mean(infeed_cyc), 1) if infeed_cyc else None,
                    "중앙값": round(statistics.median(infeed_cyc), 1) if infeed_cyc else None,
                    "최악": round(max(infeed_cyc), 1) if infeed_cyc else None,
                    "목표": target,
                    "목표 안": sum(1 for c in infeed_cyc if c <= target),
                },
                # 참고값이다. 픽앤플레이스가 두 번이라 목표와 견주지 않는다.
                "순환 (참고)": {
                    "건수": len(circ_cyc),
                    "평균": round(statistics.mean(circ_cyc), 1) if circ_cyc else None,
                    "최악": round(max(circ_cyc), 1) if circ_cyc else None,
                },
            },
            "판독": {
                "성공": len(reads),
                "평균 신뢰도": round(statistics.mean(confs), 2) if confs else None,
            },
            "예외": {"건수": self.exceptions, "사유": reasons},
            "이벤트": by_type,
            "적재 정확도": self._placement_error(),
            "판정": self._verdict(infeed_cyc, target),
        }

    def _verdict(self, cyc: list[float], target: float) -> list[str]:
        """사람이 읽는 한 줄짜리 판정들. 통과/실패를 단정하지 않는다.

        Dry Run은 합격증이 아니라 측정이다. 무엇이 목표를 못 맞췄는지
        그대로 적고, 받아들일지는 사람이 정한다.
        """
        out: list[str] = []
        if not cyc:
            out.append("완료된 사이클이 없다. 셀이 돌지 않았거나 너무 짧게 봤다.")
            return out
        over = [c for c in cyc if c > target]
        if over:
            out.append(
                f"사이클 {len(over)}/{len(cyc)}건이 목표 {target:.0f}초를 넘었다 "
                f"(최악 {max(over):.1f}초)."
            )
        else:
            out.append(f"사이클 {len(cyc)}건 전부 목표 {target:.0f}초 안.")
        if self.exceptions:
            # 사유를 함께 적는다. hazmat나 oversize는 설계대로 걸러 낸
            # 것이므로 "확인할 것"이 아니라 정상이다. 사유 없이 건수만
            # 내면 정상 동작을 결함처럼 읽게 된다.
            reasons: dict[str, int] = {}
            for e in self.events:
                if e.get("type") == "EXCEPTION":
                    r = e.get("extra") or "사유 없음"
                    reasons[r] = reasons.get(r, 0) + 1
            detail = ", ".join(f"{k} {v}건" for k, v in sorted(reasons.items()))
            expected = {"hazmat", "oversize"}
            unexpected = sum(v for k, v in reasons.items() if k not in expected)
            out.append(
                f"예외 {self.exceptions}건 ({detail})."
                + (f" 그중 {unexpected}건은 설계된 분류가 아니다." if unexpected
                   else " 전부 MES 규칙대로 걸러 낸 것이다.")
            )
        pe = self._placement_error()
        if pe.get("문제"):
            out.append(f"적재 기록과 실물이 어긋난 것 {len(pe['문제'])}건.")
        elif pe.get("짝지은 수"):
            out.append(f"적재 {pe['짝지은 수']}건 전부 기록과 맞는다 "
                       f"(최악 {pe['최악 오차 mm']} mm).")
        return out

    # ------------------------------------------------------------------ 출력
    def _tick(self) -> None:
        limit = float(self.get_parameter("duration_sec").value)
        self.write()
        if limit > 0 and time.time() - self.t0 >= limit:
            self.get_logger().info("Dry Run 시간이 다 됐다. 채점을 마친다.")
            self.report()
            raise SystemExit(0)

    def write(self) -> None:
        path = Path(str(self.get_parameter("report").value))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.score(), ensure_ascii=False, indent=2) + "\n")

    def report(self) -> None:
        s = self.score()
        self.get_logger().info("=== Dry Run 채점 ===")
        for line in json.dumps(s, ensure_ascii=False, indent=2).split("\n"):
            self.get_logger().info(line)


def main() -> None:
    rclpy.init()
    node = DryRunScorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.report()
            node.write()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
