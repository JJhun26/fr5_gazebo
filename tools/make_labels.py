#!/usr/bin/env python3
"""박스 상면 라벨 텍스처와 MES 시드 데이터를 만든다.

기획서 5.4의 판독 파이프라인은 QR 하나로 끝난다.
  촬영 -> QR 디코드 -> 네 모서리 좌표 -> 호모그래피 -> (x, y, yaw)
라벨이 상면 중앙에 붙으므로 모서리 평균이 곧 박스 중심이고, 변의 기울기가
회전각이다. 그러려면 시뮬레이터의 라벨도 실물처럼 '네 모서리가 또렷한
정사각형 QR'이어야 한다. 그래서 여백(quiet zone)과 테두리를 실제 라벨
규격대로 넣는다.

치수는 cell.yaml의 box.label에서 온다. 두 곳에 적으면 어긋난다.

크기를 40 mm에서 50 mm로 키우고 C1을 575 mm에서 375 mm로 내린 이유는
렌더러의 밉맵 필터다. 화면에서 라벨이 100 px 아래로 내려가면 QR 21 모듈이
모듈당 5 px가 안 되고, 그 크기에서는 축소 필터가 모듈을 뭉개 디코드가
통째로 실패한다. 실물 카메라도 렌즈 MTF 때문에 사정이 크게 다르지 않다.
코드 문자열을 짧게 유지해 QR 버전 1(21 모듈)에 머무르는 것도 같은 이유다.

    <venv>/bin/python tools/make_labels.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import qrcode
import yaml
from PIL import Image, ImageDraw, ImageFont
from qrcode.constants import ERROR_CORRECT_M

ROOT = Path(__file__).resolve().parent.parent
TEX_DIR = ROOT / "src/box_cell_sim/models/box/materials/textures"
MESH = ROOT / "src/box_cell_sim/models/box/meshes/label_plane.obj"
SEED_JSON = ROOT / "src/box_cell_mes/box_cell_mes/seed_items.json"
CELL_YAML = ROOT / "src/box_cell_description/config/cell.yaml"
_LABEL = yaml.safe_load(CELL_YAML.read_text())["box"]["label"]

PX = 512                 # 텍스처 한 변
LABEL_MM = _LABEL["size"] * 1000.0
QUIET_MM = _LABEL.get("quiet", 0.0025) * 1000.0        # QR 사방 여백
TEXT_STRIP_MM = _LABEL.get("text_strip", 0.004) * 1000.0

# 8개 박스의 품목. MES items 테이블(code, name, category, note)과 같은 내용이다.
ITEMS = [
    ("AXO-0001", "무선 이어폰", "전자", "소형 · 파손 주의"),
    ("AXO-0002", "보조 배터리", "전자", "리튬 · 항공 제한"),
    ("AXO-0003", "USB 허브", "전자", ""),
    ("AXO-0004", "커피 원두 1kg", "식품", "직사광선 피할 것"),
    ("AXO-0005", "머그컵 2입", "생활", "파손 주의"),
    ("AXO-0006", "면 티셔츠 L", "의류", ""),
    ("AXO-0007", "노트 5권", "문구", ""),
    ("AXO-0008", "공구 세트", "공구", "중량 주의"),
]

def load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def make_label(code: str) -> Image.Image:
    mm = PX / LABEL_MM
    img = Image.new("RGB", (PX, PX), "white")
    draw = ImageDraw.Draw(img)

    # 라벨 테두리. 실물 라벨지의 인쇄 경계다.
    draw.rectangle([0, 0, PX - 1, PX - 1], outline=(40, 40, 40), width=max(1, int(0.3 * mm)))

    qr = qrcode.QRCode(version=1, error_correction=ERROR_CORRECT_M, box_size=10, border=0)
    qr.add_data(code)
    qr.make(fit=True)
    qr_img = qr.make_image(fill_color="black", back_color="white").convert("RGB")

    quiet = int(QUIET_MM * mm)
    strip = int(TEXT_STRIP_MM * mm)
    side = PX - 2 * quiet - strip
    qr_img = qr_img.resize((side, side), Image.NEAREST)
    img.paste(qr_img, (quiet, quiet))

    font = load_font(int(strip * 0.72))
    box = draw.textbbox((0, 0), code, font=font)
    draw.text(
        ((PX - (box[2] - box[0])) / 2, PX - quiet - strip + (strip - (box[3] - box[1])) / 2 - box[1]),
        code,
        fill=(20, 20, 20),
        font=font,
    )
    return img


def make_cardboard() -> Image.Image:
    """옆면과 바닥용 골판지 무늬. 판독과 무관하지만 화면에서 티가 난다."""
    size = 256
    img = Image.new("RGB", (size, size), (196, 158, 110))
    draw = ImageDraw.Draw(img)
    for y in range(0, size, 7):
        draw.line([(0, y), (size, y)], fill=(186, 148, 100), width=1)
    for x in range(0, size, 53):
        draw.line([(x, 0), (x, size)], fill=(178, 141, 95), width=2)
    return img


def write_mesh() -> None:
    """라벨 판 메시. 크기가 cell.yaml에서 오므로 여기서 함께 만든다.

    기본 도형(box)에 텍스처를 얹지 않고 굳이 메시를 쓰는 이유는 UV를 직접
    쥐기 위해서다. 판독은 QR 네 모서리의 픽셀 좌표에 전적으로 기대는데,
    렌더러가 만들어 주는 UV에 맡기면 모서리가 어디에 찍힐지 보장되지 않는다.
    """
    h = LABEL_MM / 2000.0
    MESH.parent.mkdir(parents=True, exist_ok=True)
    MESH.write_text(
        f"""# 박스 상면 라벨 판. 한 변 {LABEL_MM:.0f} mm, +z 법선, UV (0,0)-(1,1).
# tools/make_labels.py가 cell.yaml의 box.label.size에서 만들어 낸다.
# 직접 고치지 말 것.
o label_plane
v {-h:.4f} {-h:.4f} 0.0
v {h:.4f} {-h:.4f} 0.0
v {h:.4f} {h:.4f} 0.0
v {-h:.4f} {h:.4f} 0.0
vt 0.0 0.0
vt 1.0 0.0
vt 1.0 1.0
vt 0.0 1.0
vn 0.0 0.0 1.0
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""
    )
    print(f"  {MESH.relative_to(ROOT)}   한 변 {LABEL_MM:.0f} mm")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=TEX_DIR)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    for i, (code, *_rest) in enumerate(ITEMS, start=1):
        path = args.out / f"box_{i}.png"
        make_label(code).save(path)
        print(f"  {path.relative_to(ROOT)}   code={code}")

    write_mesh()

    cb = args.out / "cardboard.png"
    make_cardboard().save(cb)
    print(f"  {cb.relative_to(ROOT)}")

    SEED_JSON.parent.mkdir(parents=True, exist_ok=True)
    SEED_JSON.write_text(
        json.dumps(
            [
                {"code": c, "name": n, "category": g, "note": t, "box": f"box_{i}"}
                for i, (c, n, g, t) in enumerate(ITEMS, start=1)
            ],
            ensure_ascii=False,
            indent=2,
        )
        + "\n"
    )
    print(f"  {SEED_JSON.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
