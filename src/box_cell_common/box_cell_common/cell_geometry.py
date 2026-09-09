"""cell.yaml을 코드 쪽에서 읽는다.

URDF(xacro)와 검증 스크립트가 읽는 그 파일이다. 치수를 노드에 다시 적으면
언젠가 반드시 어긋나고, 어긋난 줄도 모른 채 데모 당일에 알게 된다.

좌표계 두 개를 오간다.
  table : 기획서가 쓰는 좌표계. 원점은 상판 좌측 하단 모서리의 상면.
  world : Gazebo/TF의 좌표계. table을 z로 상판 높이(0.750)만큼 올린 것.
둘의 차이는 z 평행이동 하나뿐이다. 그래도 함수 이름에 어느 쪽인지 항상
박아 둔다. mm/m와 함께, 이 종류의 착각이 로봇을 상판에 박는다.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


def default_cell_path() -> Path:
    """설치된 box_cell_description의 cell.yaml 경로.

    ROS 없이 도구로 돌릴 때를 위해 BOX_CELL_YAML 환경 변수를 먼저 본다.
    """
    override = os.environ.get("BOX_CELL_YAML")
    if override:
        return Path(override)
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("box_cell_description")) / "config" / "cell.yaml"


@dataclass(frozen=True)
class Slot:
    """팔레트 위 한 자리. index가 기획서 5.3의 격자 인덱스다."""

    pallet_id: int
    index: int
    col: int
    row: int
    layer: int
    x: float
    y: float
    top_z: float        # 이 자리에 박스를 놓았을 때 박스 상면의 world z
    center_z: float     # 그 박스 무게중심의 world z


class CellGeometry:
    """셀 제원 전체. 읽기 전용."""

    def __init__(self, path: Path | str | None = None) -> None:
        self.path = Path(path) if path else default_cell_path()
        self.data: dict[str, Any] = yaml.safe_load(self.path.read_text())

    # ---------------------------------------------------------------- 기본값
    @property
    def table_top(self) -> float:
        """상판 상면의 world z."""
        return float(self.data["frame"]["table_top_height"])

    @property
    def box_size(self) -> tuple[float, float, float]:
        return tuple(float(v) for v in self.data["box"]["size"])  # type: ignore[return-value]

    @property
    def box_height(self) -> float:
        return self.box_size[2]

    @property
    def box_count(self) -> int:
        return int(self.data["box"]["count"])

    @property
    def tcp_offset(self) -> float:
        return float(self.data["tool"]["tcp_offset"])

    @property
    def slots_per_pallet(self) -> int:
        s = self.data["pallet"]["slots"]
        return int(s["cols"]) * int(s["rows"]) * int(s["layers"])

    @property
    def pallet_ids(self) -> list[int]:
        return [int(u["id"]) for u in self.data["pallet"]["units"]]

    def to_world(self, x: float, y: float, z_table: float) -> tuple[float, float, float]:
        """table 좌표를 world로. 차이는 z 하나다."""
        return (x, y, z_table + self.table_top)

    # ---------------------------------------------------------------- 팔레트
    def pallet_center(self, pallet_id: int) -> tuple[float, float]:
        for u in self.data["pallet"]["units"]:
            if int(u["id"]) == pallet_id:
                return (float(u["center"][0]), float(u["center"][1]))
        raise KeyError(f"팔레트 {pallet_id}은 cell.yaml에 없다")

    def slot(self, pallet_id: int, index: int) -> Slot:
        """기획서 5.3의 격자 공식 그대로.

            col   = index % 2          x = Px + (col - 0.5) * 70
            row   = (index // 2) % 2   y = Py + (row - 0.5) * 70
            layer = index // 4         z = Pz + 30 + layer * 40 + 40

        마지막 +40이 박스 높이다. 흡착 TCP가 박스 상면을 잡기 때문에
        여기서 나오는 z가 곧 TCP가 가야 할 높이다.
        """
        cfg = self.data["pallet"]["slots"]
        cols, rows = int(cfg["cols"]), int(cfg["rows"])
        pitch = float(cfg["pitch"])
        if not 0 <= index < self.slots_per_pallet:
            raise IndexError(f"슬롯 인덱스 {index}가 범위를 벗어났다")
        col = index % cols
        row = (index // cols) % rows
        layer = index // (cols * rows)
        px, py = self.pallet_center(pallet_id)
        base = float(self.data["pallet"]["thickness"])
        top_z = self.table_top + base + layer * self.box_height + self.box_height
        return Slot(
            pallet_id=pallet_id,
            index=index,
            col=col,
            row=row,
            layer=layer,
            x=px + (col - (cols - 1) / 2.0) * pitch,
            y=py + (row - (rows - 1) / 2.0) * pitch,
            top_z=top_z,
            center_z=top_z - self.box_height / 2.0,
        )

    def all_slots(self, pallet_id: int) -> list[Slot]:
        return [self.slot(pallet_id, i) for i in range(self.slots_per_pallet)]

    @property
    def approach_height(self) -> float:
        return float(self.data["pallet"]["slots"]["approach_height"])

    # ------------------------------------------------------------- 컨베이어
    @property
    def belt_surface_z(self) -> float:
        """벨트 상면의 world z."""
        return self.table_top + float(self.data["conveyor"]["belt"]["surface_z"])

    @property
    def belt_center_y(self) -> float:
        return float(self.data["conveyor"]["belt"]["center_y"])

    @property
    def belt_friction(self) -> float:
        """벨트 표면과 박스 사이의 마찰. 형상과 이송력 계산이 같은 값을 쓴다."""
        return float(self.data["conveyor"]["belt"].get("friction", 0.6))

    @property
    def belt_speed(self) -> float:
        return float(self.data["conveyor"]["speed"])

    @property
    def belt_x_range(self) -> tuple[float, float]:
        b = self.data["conveyor"]["belt"]
        return (float(b["x_start"]), float(b["x_end"]))

    @property
    def stop_sensor_x(self) -> float:
        return float(self.data["conveyor"]["stop_sensor_x"])

    @property
    def infeed_x(self) -> float:
        """로봇이 박스를 벨트에 되올려 놓는 자리(유효 구간 안)."""
        return float(self.data["conveyor"]["infeed_x"])

    def read_station_xy(self) -> tuple[float, float]:
        c = self.data["read_station"]["center"]
        return (float(c[0]), float(c[1]))

    @property
    def read_top_z(self) -> float:
        """판독 위치에 선 박스의 상면 world z. TCP가 가야 할 높이다."""
        return self.belt_surface_z + self.box_height

    # ------------------------------------------------------------- 예외 통
    def exception_drop_pose(self) -> tuple[float, float, float]:
        eb = self.data["exception_bin"]
        return (
            float(eb["center"][0]),
            float(eb["center"][1]),
            self.table_top + float(eb["height"]) + float(eb["drop_height"]),
        )

    # ---------------------------------------------------------------- 튜닝
    @property
    def home_joints(self) -> list[float]:
        return [float(v) for v in self.data["tuning"]["home_joints"]]

    @property
    def read_hover_z(self) -> float:
        return self.table_top + float(self.data["tuning"]["read_hover_z"])

    @property
    def pallet_hover_z(self) -> float:
        return self.table_top + float(self.data["tuning"]["pallet_hover_z"])

    @property
    def tuning(self) -> dict[str, Any]:
        return self.data["tuning"]

    @property
    def physics_step(self) -> float:
        return float(self.data.get("simulation", {}).get("physics_step", 0.001))

    # -------------------------------------------------------------- 시나리오
    @property
    def scenario(self) -> dict[str, Any]:
        return self.data.get("scenario", {"mode": "circulate"})

    @property
    def scenario_mode(self) -> str:
        return str(self.scenario.get("mode", "circulate"))

    # ---------------------------------------------------------------- 카메라
    def camera(self, key: str) -> dict[str, Any]:
        return self.data["cameras"][key]

    @property
    def camera_keys(self) -> list[str]:
        return list(self.data["cameras"])

    # ------------------------------------------------- Planning Scene 충돌체
    def static_obstacles(self) -> list[dict[str, Any]]:
        """MoveIt Planning Scene에 등록할 고정 충돌체.

        기획서 5.2가 요구하는 목록 그대로다 : 상판, 컨베이어, 카메라 지주,
        팔레트 2개, 예외 통. 등록하지 않은 물체는 MoveIt에게 없는 것이다.

        상판은 받침 개구(400 각)를 피해 네 조각으로 넣는다. 통짜로 넣으면
        로봇이 자기 받침판과 충돌한다고 보고 아무 데도 못 간다.

        반환 형식 : {name, size(xyz), pose(xyz world)}. 전부 축 정렬 박스다.
        """
        tbl = self.data["table"]
        op = tbl["opening"]
        th = float(tbl["top_thickness"])
        ox0 = float(op["center"][0]) - float(op["size"]) / 2
        ox1 = float(op["center"][0]) + float(op["size"]) / 2
        oy0 = float(op["center"][1]) - float(op["size"]) / 2
        oy1 = float(op["center"][1]) + float(op["size"]) / 2
        sx, sy = float(tbl["size_x"]), float(tbl["size_y"])
        z_top = self.table_top - th / 2

        out: list[dict[str, Any]] = [
            {"name": "table_left", "size": (ox0, sy, th), "pose": (ox0 / 2, sy / 2, z_top)},
            {"name": "table_right", "size": (sx - ox1, sy, th), "pose": ((sx + ox1) / 2, sy / 2, z_top)},
            {"name": "table_front", "size": (ox1 - ox0, oy0, th), "pose": ((ox0 + ox1) / 2, oy0 / 2, z_top)},
            {"name": "table_back", "size": (ox1 - ox0, sy - oy1, th), "pose": ((ox0 + ox1) / 2, (sy + oy1) / 2, z_top)},
        ]

        belt = self.data["conveyor"]["belt"]
        bl = float(belt["x_end"]) - float(belt["x_start"])
        bw = float(belt["width"])
        bz = float(belt["surface_z"])
        rail = self.data["conveyor"]["side_rail"]
        rail_h = float(rail["height"])
        rail_t = float(rail["thickness"])

        # 벨트 몸통은 상판에서 벨트 상면까지만이다. 가이드 레일까지 한 덩어리로
        # 넣으면 벨트 위의 작업 공간이 통째로 장애물 안에 묻힌다. 그러면 박스를
        # 벨트에 내려놓는 하강 궤적이 끝까지 안 풀린다(직선 77%에서 멈춘다).
        # 레일은 따로 둔다. 실제로도 벨트 표면과 레일은 다른 물건이다.
        out.append(
            {
                "name": "conveyor",
                "size": (bl, bw, bz),
                "pose": (
                    (float(belt["x_start"]) + float(belt["x_end"])) / 2,
                    float(belt["center_y"]),
                    self.table_top + bz / 2,
                ),
            }
        )
        for tag, sign in (("near", -1), ("far", 1)):
            out.append(
                {
                    "name": f"conveyor_rail_{tag}",
                    "size": (bl, rail_t, rail_h),
                    "pose": (
                        (float(belt["x_start"]) + float(belt["x_end"])) / 2,
                        float(belt["center_y"]) + sign * (bw / 2 - rail_t / 2),
                        self.table_top + bz + rail_h / 2,
                    ),
                }
            )
        gb = float(self.data["conveyor"]["gearbox"]["size"])
        out.append(
            {
                "name": "conveyor_gearbox",
                "size": (gb, gb, gb),
                "pose": (
                    float(self.data["conveyor"]["gearbox"]["x_max"]) - gb / 2,
                    float(belt["center_y"]),
                    self.belt_surface_z - gb / 2,
                ),
            }
        )

        pal = self.data["pallet"]
        for u in pal["units"]:
            out.append(
                {
                    "name": f"pallet_{u['id']}",
                    "size": (float(pal["size"]), float(pal["size"]), float(pal["thickness"])),
                    "pose": (
                        float(u["center"][0]),
                        float(u["center"][1]),
                        self.table_top + float(pal["thickness"]) / 2,
                    ),
                }
            )

        eb = self.data["exception_bin"]
        out.append(
            {
                "name": "exception_bin",
                "size": (float(eb["size"][0]), float(eb["size"][1]), float(eb["height"])),
                "pose": (
                    float(eb["center"][0]),
                    float(eb["center"][1]),
                    self.table_top + float(eb["height"]) / 2,
                ),
            }
        )

        # 카메라 갠트리. 기둥 두 개가 도달 고리(외경 884) 안에 있다.
        # 이것을 빼먹으면 팔레트로 가는 궤적이 기둥을 관통한다.
        gan = self.data["gantry"]
        gs = float(gan["section"])
        for post in gan["posts"]:
            out.append(
                {
                    "name": f"gantry_post_{post['id']}",
                    "size": (gs, gs, float(gan["post_height"])),
                    "pose": (
                        float(post["xy"][0]),
                        float(post["xy"][1]),
                        self.table_top + float(gan["post_height"]) / 2,
                    ),
                }
            )
        cb = gan["cross_beam"]
        out.append(
            {
                "name": "gantry_cross_beam",
                "size": (gs, float(cb["y_max"]) - float(cb["y_min"]) + gs, gs),
                "pose": (
                    float(cb["x"]),
                    (float(cb["y_min"]) + float(cb["y_max"])) / 2,
                    self.table_top + float(cb["z"]),
                ),
            }
        )
        for key in ("arm_conveyor", "arm_pallet"):
            arm = gan[key]
            out.append(
                {
                    "name": f"gantry_{key}",
                    "size": (float(arm["x_max"]) - float(arm["x_min"]), gs, gs),
                    "pose": (
                        (float(arm["x_min"]) + float(arm["x_max"])) / 2,
                        float(arm["y"]),
                        self.table_top + float(arm["z"]),
                    ),
                }
            )

        # 카메라 본체도 넣는다. 손목이 스치기 쉬운 높이에 있다.
        for key in ("c1_conveyor", "c2_pallet"):
            cam = self.data["cameras"][key]
            out.append(
                {
                    "name": f"camera_{key}",
                    "size": (0.06, 0.14, 0.06),
                    "pose": (
                        float(cam["xyz"][0]),
                        float(cam["xyz"][1]),
                        self.table_top + float(cam["xyz"][2]),
                    ),
                }
            )

        return out


def yaw_normalize_square(yaw: float) -> float:
    """정사각형 박스의 회전각을 ±45도 안으로 접는다.

    박스가 60 각 정사각형이라 90도 대칭이다. 라벨이 87도 돌아 보이면
    실제로는 -3도 돌아간 것과 같고, 로봇은 -3도만 돌면 된다.
    FR5 J6 범위가 ±175도이므로 이렇게 접어 두면 손목이 한계에 닿을 일이 없다
    (기획서 5.4).
    """
    y = math.fmod(yaw, math.pi / 2)
    if y > math.pi / 4:
        y -= math.pi / 2
    elif y < -math.pi / 4:
        y += math.pi / 2
    return y
