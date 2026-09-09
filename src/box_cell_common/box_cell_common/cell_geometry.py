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
        """기본 규격의 치수.

        규격이 여러 가지가 되면서 "그 박스"의 치수는 코드를 알아야 나온다
        (MES 조회). 이 값은 아직 규격을 모를 때 가정하는 값일 뿐이다.
        판독 평면 계산이나 화면 표시처럼 틀려도 사이클이 안 죽는 곳에만 쓴다.
        """
        return self.default_box_size

    @property
    def box_height(self) -> float:
        return self.box_size[2]

    @property
    def box_count(self) -> int:
        return int(self.data["box"]["count"])

    @property
    def tcp_offset(self) -> float:
        return float(self.data["tool"]["tcp_offset"])

    def pallet_slot_cfg(self) -> dict:
        """적재 탐색 설정. cell.yaml의 pallet.slots."""
        return self.data["pallet"]["slots"]

    def sample_placements(self, pallet_id: int, kind: str | None = None,
                          count: int | None = None) -> list[tuple[float, float, float]]:
        """이 팔레트에 그 규격을 채웠을 때 나오는 자리들. (x, y, 상면 z) world.

        고정 격자가 없어졌으므로 검증 도구가 "슬롯 8개"를 셀 수 없다.
        대신 실제 적재에 쓰는 것과 **같은 패커**를 돌려 대표 자리를 얻는다.
        도구가 다른 계산을 쓰면 통과해도 의미가 없다.
        """
        from box_cell_common.pallet_pack import Packer

        cfg = self.pallet_slot_cfg()
        pk = Packer(
            size=float(self.data["pallet"]["size"]),
            margin=float(cfg["margin"]),
            gap=float(cfg["gap"]),
            step=float(cfg["step"]),
            max_layers=int(cfg["layers"]),
        )
        size = self.box_size_of(kind) if kind else None
        sx, sy, sz = size or self.default_box_size
        px, py = self.pallet_center(pallet_id)
        base = self.table_top + float(self.data["pallet"]["thickness"])
        out: list[tuple[float, float, float]] = []
        limit = count if count is not None else int(self.box_count)
        for _ in range(limit):
            pl = pk.find(sx, sy, sz)
            if pl is None:
                break
            pk.add(pl, "")
            out.append((px + pl.x, py + pl.y, base + pl.z_base + pl.sz))
        return out

    @property
    def pallet_ids(self) -> list[int]:
        return [int(u["id"]) for u in self.data["pallet"]["units"]]

    def pallet_center(self, pallet_id: int) -> tuple[float, float]:
        for u in self.data["pallet"]["units"]:
            if int(u["id"]) == pallet_id:
                return (float(u["center"][0]), float(u["center"][1]))
        raise KeyError(f"팔레트 {pallet_id}은 cell.yaml에 없다")


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

    @property
    def infeed_x_start(self) -> float:
        """상류 인피드 구간의 끝. 벨트 유효 구간보다 위쪽(-x)이다."""
        return float(self.data["conveyor"]["infeed"]["x_start"])

    def feed_spawn_pose(self, height: float | None = None) -> tuple[float, float, float]:
        """박스가 나타나는 자리. 이미 도는 벨트 위에 떨어뜨린다.

        미리 줄 세워 두지 않는다. gz는 안착한 물체를 재우고, 한 번 잠들면
        접촉 상대가 나중에 돌기 시작해도 다시 깨어나지 않는다(위로 50 N을
        걸어도 안 뜬다). 필요한 순간에 도는 벨트 위로 떨어뜨리면 그 문제가
        생길 틈이 없다.

        높이는 그 박스의 실제 높이여야 한다. 기본 규격 높이로 계산하면
        떨어지는 거리가 규격마다 달라진다. 실제로 제일 큰 XL(95 mm)은
        기본값 60 mm 기준으로 놓여 바닥 간격이 2.5 mm밖에 안 됐고, 거의
        떨어지지 않으니 생기자마자 접촉해 잠들어 벨트가 돌아도 끝까지
        움직이지 않았다. 로트의 마지막 박스 하나가 통째로 멈춘 원인이다.
        """
        inf = self.data["conveyor"]["infeed"]
        h = float(height) if height else float(self.box_height)
        return (
            float(inf["spawn_x"]),
            self.belt_center_y,
            self.belt_surface_z + h / 2.0 + float(inf.get("spawn_drop", 0.020)),
        )

    def on_belt_x_range(self) -> tuple[float, float]:
        """벨트 위로 볼 x 범위. 상류 인피드 구간까지 포함한다."""
        return (self.infeed_x_start, float(self.data["conveyor"]["belt"]["x_end"]))

    # ------------------------------------------------------------ 박스 규격
    def box_kinds(self) -> dict:
        """규격 이름 -> {size, mass}. cell.yaml의 box.kinds."""
        return self.data["box"].get("kinds", {})

    def box_size_of(self, kind: str) -> tuple[float, float, float] | None:
        """규격 이름의 실제 치수. 모르는 이름이면 None.

        치수는 여기에만 있고 MES에는 규격 이름만 있다. 두 곳에 mm를 적으면
        반드시 어긋나기 때문이다. 실물에서도 WMS는 "규격 M"이라고 말하지
        "130x100x60"이라고 말하지 않는다.
        """
        k = self.box_kinds().get(kind)
        if not k:
            return None
        s = k["size"]
        return (float(s[0]), float(s[1]), float(s[2]))

    @property
    def default_box_size(self) -> tuple[float, float, float]:
        """규격을 모를 때 가정하는 치수. 판독 평면 계산에만 쓴다."""
        size = self.box_size_of(str(self.data["box"].get("default_kind", "M")))
        return size or (0.100, 0.085, 0.045)

    @property
    def qr_side(self) -> float:
        """라벨에 인쇄되는 QR 한 변(m). 여백은 뺀 본체만이다.

        송장이 세로로 긴 직사각이라 QR 크기는 세로가 정한다.
          높이 - 여백 x2 - 바코드 띠 - 코드 문자열 띠
        tools/make_labels.py가 이 식으로 그리고, pose_resolver가 이 값과
        실측 한 변을 견줘 신뢰도를 낸다. 두 곳에 따로 적으면 반드시
        어긋나므로 여기 한 곳에 둔다. 실제로 라벨을 정사각에서 직사각으로
        바꿀 때 pose_resolver만 옛 키(label.size)를 보다가 죽었다.
        """
        lab = self.data["box"]["label"]
        return (
            float(lab["height"])
            - 2 * float(lab.get("quiet", 0.009))
            - float(lab.get("barcode_strip", 0.009))
            - float(lab.get("text_strip", 0.0035))
        )

    def read_top_z_for(self, height: float | None = None) -> float:
        """그 높이의 박스가 판독 자리에 섰을 때 상면의 world z.

        규격이 여러 가지라 판독 평면이 하나가 아니다(기획서 5.4의 전제가
        깨진 자리). 코드를 읽고 MES에서 규격을 받은 뒤 이 값을 다시 낸다.
        """
        h = float(height) if height else float(self.default_box_size[2])
        return self.belt_surface_z + h

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
        # 카메라 지주. 기둥은 상판 바깥 바닥에 서지만 가로 팔은 도달 고리
        # 안으로 들어온다. 등록하지 않으면 MoveIt에게는 없는 물건이다.
        cfr = self.data["camera_frame"]
        gs = float(cfr["section"])
        floor_z = float(cfr["floor_z"])
        arm_z = float(cfr["arm_z"])
        post_top = arm_z + gs / 2
        for post in cfr["posts"]:
            out.append(
                {
                    "name": f"camera_post_{post['id']}",
                    "size": (gs, gs, post_top - floor_z),
                    "pose": (
                        float(post["xy"][0]),
                        float(post["xy"][1]),
                        self.table_top + (post_top + floor_z) / 2,
                    ),
                }
            )
        for arm in cfr["arms"]:
            out.append(
                {
                    "name": f"camera_arm_{arm['id']}",
                    "size": (float(arm["x_max"]) - float(arm["x_min"]), gs, gs),
                    "pose": (
                        (float(arm["x_min"]) + float(arm["x_max"])) / 2,
                        float(arm["y"]),
                        self.table_top + arm_z,
                    ),
                }
            )
        ys = [float(p["xy"][1]) for p in cfr["posts"]]
        out.append(
            {
                "name": "camera_frame_tie",
                "size": (gs, max(ys) - min(ys), gs),
                "pose": (
                    float(cfr["posts"][0]["xy"][0]),
                    (max(ys) + min(ys)) / 2,
                    self.table_top + arm_z,
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
