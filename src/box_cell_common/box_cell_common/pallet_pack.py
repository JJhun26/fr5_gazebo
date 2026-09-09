#!/usr/bin/env python3
"""팔레트에 크기가 제각각인 박스를 놓을 자리를 찾는다.

왜 필요한가.
    기획서 5.3은 자리를 격자 공식으로 정했다.

        col = index % 2,  row = (index // 2) % 2,  layer = index // 4

    박스가 모두 60 각 40 높이라는 전제 위에서만 성립한다. 실물 택배 상자는
    규격이 제각각이고(S/M/L/XL), 그러면 몇 번 슬롯이라는 말 자체가 없다.
    그래서 자리를 계산이 아니라 **탐색**으로 정한다.

어떻게 놓는가.
    층을 아래에서부터 채운다. 한 층 안에서는 팔레트의 한 구석에서 시작해
    y, x 순으로 훑으며 처음 들어가는 자리에 놓는다(first-fit). 훑는 순서가
    곧 "구석부터 하나씩"이다. 놓을 때 90도 돌려 보는 것도 시도한다.
    가로로 안 들어가도 세로로는 들어가는 일이 흔하다.

    위층은 아래층이 받쳐 주는 자리에만 놓는다. 네 모서리와 중심이 모두
    아래 박스 위에 있어야 한다. 이 검사가 없으면 허공에 뜬 박스가 생기고,
    시뮬레이터에서는 떨어지고 실물에서는 무너진다.

좌표계
    팔레트 중심을 원점으로 한 국소 좌표로 계산하고, 마지막에 world로 옮긴다.
    z는 팔레트 상면(데크 윗면)에서 잰 높이다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Placement:
    """놓인 박스 하나. 치수는 놓인 자세 기준이다(yaw가 이미 반영됐다)."""

    code: str
    x: float          # 팔레트 중심 기준 국소 x (박스 중심)
    y: float
    z_base: float     # 팔레트 상면에서 박스 밑면까지
    sx: float         # 놓인 자세의 x 치수
    sy: float
    sz: float
    yaw: float
    layer: int

    @property
    def z_top(self) -> float:
        return self.z_base + self.sz

    def overlaps(self, x: float, y: float, sx: float, sy: float, gap: float) -> bool:
        return (
            abs(x - self.x) < (sx + self.sx) / 2 + gap
            and abs(y - self.y) < (sy + self.sy) / 2 + gap
        )

    def covers(self, px: float, py: float, tol: float = 1e-6) -> bool:
        return (
            abs(px - self.x) <= self.sx / 2 + tol
            and abs(py - self.y) <= self.sy / 2 + tol
        )


@dataclass
class Packer:
    """팔레트 한 장의 적재 상태와 자리 찾기."""

    size: float                 # 팔레트 한 변
    margin: float               # 가장자리에서 띄우는 거리
    gap: float                  # 박스 사이 최소 간극
    step: float                 # 자리 탐색 눈금
    max_layers: int
    placed: list[Placement] = field(default_factory=list)

    # ------------------------------------------------------------------ 조회
    @property
    def half(self) -> float:
        """놓을 수 있는 반폭. 가장자리 여백을 뺀 값이다."""
        return self.size / 2 - self.margin

    def layer_of(self, layer: int) -> list[Placement]:
        return [p for p in self.placed if p.layer == layer]

    def layer_top(self, layer: int) -> float:
        """그 층 위에 다음 층이 앉을 높이."""
        boxes = self.layer_of(layer)
        return max((p.z_top for p in boxes), default=0.0)

    # ------------------------------------------------------------------ 배치
    def _support(self, layer: int, x: float, y: float, sx: float,
                 sy: float) -> float | None:
        """이 자리에 놓을 수 있으면 밑면이 앉을 높이, 못 놓으면 None.

        위층은 받쳐 주는 자리에만 놓는다. 네 모서리와 중심이 모두 아래 박스
        위에 있어야 한다. 이 검사가 없으면 허공에 뜬 박스가 생긴다.

        받침 높이는 **그 자리를 실제로 받치는 박스들의 윗면**이다. 층
        전체의 최댓값을 쓰면 안 된다. 같은 층에 45짜리와 75짜리가 섞여
        있을 때 45짜리 위에 놓을 박스가 30 mm 떠서 떨어지기 때문이다.

        받치는 박스들의 윗면이 서로 다르면 그 자리는 버린다. 걸쳐 놓으면
        기운다. 실물에서도 단차 위에 상자를 걸치지 않는다.
        """
        if abs(x) + sx / 2 > self.half or abs(y) + sy / 2 > self.half:
            return None
        for p in self.layer_of(layer):
            if p.overlaps(x, y, sx, sy, self.gap):
                return None
        if layer == 0:
            return 0.0

        below = self.layer_of(layer - 1)
        if not below:
            return None
        hx, hy = sx / 2, sy / 2
        tops: list[float] = []
        for px, py in ((x - hx, y - hy), (x + hx, y - hy),
                       (x - hx, y + hy), (x + hx, y + hy), (x, y)):
            under = [p for p in below if p.covers(px, py)]
            if not under:
                return None
            tops.append(max(p.z_top for p in under))
        if max(tops) - min(tops) > 1e-6:
            return None            # 단차에 걸친다
        return tops[0]

    def find(self, sx: float, sy: float, sz: float) -> Placement | None:
        """이 크기의 박스를 놓을 자리. 없으면 None.

        층 -> y -> x 순으로 처음 들어가는 자리를 고른다. 그 순서가 곧
        "구석부터 하나씩"이다.
        """
        for layer in range(self.max_layers):
            if layer > 0 and not self.layer_of(layer - 1):
                break
            for yaw, (ax, ay) in ((0.0, (sx, sy)), (math.pi / 2, (sy, sx))):
                if ax > 2 * self.half or ay > 2 * self.half:
                    continue
                y = -self.half + ay / 2
                while y <= self.half - ay / 2 + 1e-9:
                    x = -self.half + ax / 2
                    while x <= self.half - ax / 2 + 1e-9:
                        z_base = self._support(layer, x, y, ax, ay)
                        if z_base is not None:
                            return Placement("", x, y, z_base, ax, ay, sz, yaw, layer)
                        x += self.step
                    y += self.step
        return None

    def add(self, p: Placement, code: str) -> Placement:
        p.code = code
        self.placed.append(p)
        return p

    def pop_last(self) -> Placement | None:
        """가장 나중에 놓은 것을 뺀다. 반출은 반드시 역순이어야 한다."""
        return self.placed.pop() if self.placed else None

    # ------------------------------------------------------------- 직렬화
    def to_json(self) -> list[dict]:
        return [vars(p) for p in self.placed]

    @classmethod
    def from_json(cls, cfg: dict, rows: list[dict]) -> "Packer":
        pk = cls(**cfg)
        pk.placed = [Placement(**r) for r in rows]
        return pk
