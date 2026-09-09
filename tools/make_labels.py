#!/usr/bin/env python3
"""박스 상면 송장 라벨 텍스처를 만든다.

품목표(규격, 중량, 취급 구분)는 MES 시드가 원본이다. 이 스크립트는 그것을
읽어 라벨에 인쇄한다. 반대가 아니다. 규격이 여러 가지가 되면서 치수의
원본이 MES로 옮겨 갔기 때문이다(box_cell_msgs/srv/ItemQuery.srv 주석).

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

# 텍스처 해상도. 라벨이 직사각이므로 가로 세로가 다르다.
# 텍스처 해상도. 화면에서 축소될 때 모듈이 뭉개지지 않을 만큼 높게 잡는다.
# 8이면 QR 모듈 하나가 텍스처에서 16 texel인데, 화면에서 7 px로 줄면
# 밉맵 필터가 경계를 흐린다. 12로 올려 여유를 둔다.
PX_PER_MM = 12
LABEL_W_MM = _LABEL["width"] * 1000.0
LABEL_H_MM = _LABEL["height"] * 1000.0
QUIET_MM = _LABEL.get("quiet", 0.009) * 1000.0          # QR 사방 여백
BARCODE_MM = _LABEL.get("barcode_strip", 0.009) * 1000.0
TEXT_STRIP_MM = _LABEL.get("text_strip", 0.0035) * 1000.0
PX_W = int(LABEL_W_MM * PX_PER_MM)
PX_H = int(LABEL_H_MM * PX_PER_MM)

# 품목은 MES 시드가 원본이다. 여기서 다시 적으면 어긋난다.
# 라벨에 규격과 중량을 인쇄하므로 그 값도 시드에서 온다.
ITEMS = json.loads(SEED_JSON.read_text()) if SEED_JSON.exists() else []


# 한글이 있는 글꼴을 먼저 찾는다. 송장에 품목명과 분류가 한글로 찍힌다.
# DejaVu에는 한글 글자가 없어서 전부 네모로 나온다.
FONT_PATHS = (
    "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)


def load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for path in FONT_PATHS:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def fit_text(draw: ImageDraw.ImageDraw, text: str, max_w: int, size: int) -> ImageFont.ImageFont:
    """폭에 들어갈 때까지 글꼴을 줄인다. 잘려 나가는 것보다 낫다."""
    while size > 6:
        font = load_font(size)
        if draw.textlength(text, font=font) <= max_w:
            return font
        size -= 1
    return load_font(6)


def make_barcode(code: str, w_px: int, h_px: int) -> Image.Image:
    """Code128 바코드. 사람이 스캐너로 찍는 몫이다.

    기계가 읽는 것은 QR이다(기획서 7절 권장). QR은 네 모서리 좌표를 함께
    주므로 위치와 회전까지 한 번에 나오는데, 1D 바코드는 그게 안 된다.
    그래도 송장에 바코드가 없으면 송장으로 안 보이고, 실물 창고의 핸디
    스캐너는 여전히 1D를 쓴다. 그래서 둘 다 넣는다.

    python-barcode가 없으면 줄무늬만 그린다. 그 경우 스캐너로는 안 읽히지만
    QR 판독에는 영향이 없다. 라벨을 못 만들어 판독이 통째로 죽는 것보다 낫다.
    """
    try:
        import barcode
        from barcode.writer import ImageWriter

        cls = barcode.get_barcode_class("code128")
        img = cls(code, writer=ImageWriter()).render(
            {
                "module_height": max(2.0, h_px / PX_PER_MM),
                "font_size": 0,
                "text_distance": 0,
                "quiet_zone": 1.0,
                "write_text": False,
            }
        )
        return img.convert("RGB").resize((w_px, h_px), Image.LANCZOS)
    except Exception:  # noqa: BLE001 - 라이브러리가 없거나 렌더가 실패해도 계속 간다
        img = Image.new("RGB", (w_px, h_px), "white")
        d = ImageDraw.Draw(img)
        x = 2
        widths = [1, 2, 1, 3, 1, 1, 2, 1, 2, 3, 1, 1, 3, 2, 1, 1, 2, 1]
        i = 0
        while x < w_px - 3:
            w = widths[i % len(widths)]
            if i % 2 == 0:
                d.rectangle([x, 0, x + w - 1, h_px - 1], fill=(15, 15, 15))
            x += w
            i += 1
        return img


def make_label(item: dict) -> Image.Image:
    """택배 송장 한 장.

    실물 송장처럼 위에서 아래로 : 발송 정보 -> QR -> 바코드 -> 코드 문자열.
    QR만 기계가 읽고 나머지는 사람이 본다.

    QR 여백(quiet zone)은 규격이 사방 4모듈을 요구한다. 이걸 지키지 않으면
    디코더가 찾기 패턴의 경계를 잡을 때도, 못 잡을 때도 있다. 실제로 여백이
    1.3모듈이던 시절에는 8개 중 2개가 판독 3회를 다 실패해 예외 통으로 갔다.
    """
    code = item["code"]
    mm = PX_PER_MM
    img = Image.new("RGB", (PX_W, PX_H), "white")
    draw = ImageDraw.Draw(img)

    quiet = int(QUIET_MM * mm)
    bar_h = int(BARCODE_MM * mm)
    strip = int(TEXT_STRIP_MM * mm)

    # 송장 테두리
    draw.rectangle([0, 0, PX_W - 1, PX_H - 1], outline=(40, 40, 40), width=max(1, int(0.3 * mm)))

    # --- QR. 세로가 남는 자리를 정한다. 여백은 사방으로 같게.
    side = PX_H - 2 * quiet - bar_h - strip
    qr = qrcode.QRCode(version=1, error_correction=ERROR_CORRECT_M, box_size=10, border=0)
    qr.add_data(code)
    qr.make(fit=True)
    qr_img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
    qr_img = qr_img.resize((side, side), Image.NEAREST)
    qr_x = (PX_W - side) // 2
    img.paste(qr_img, (qr_x, quiet))

    # --- QR 오른쪽 빈 자리에 발송 정보. QR 여백을 침범하지 않는다.
    right_x = qr_x + side + quiet
    if PX_W - right_x > 12 * mm:
        col_w = PX_W - quiet - right_x
        lines = [
            ("AXISONE", int(3.6 * mm), (20, 20, 20)),
            (item.get("name", ""), int(2.6 * mm), (30, 30, 30)),
            (f"규격 {item.get('kind', '?')}", int(2.6 * mm), (40, 40, 40)),
            (f"{item.get('weight_kg', 0):.2f} kg", int(2.6 * mm), (40, 40, 40)),
            (item.get("category", ""), int(2.4 * mm), (95, 95, 95)),
        ]
        y = quiet
        for text, size, fill in lines:
            if not text:
                continue
            font = fit_text(draw, text, col_w, size)
            draw.text((right_x, y), text, font=font, fill=fill)
            y += int(font.size * 1.40)
        # 취급 주의는 눈에 띄게. 이 표시가 예외 처리의 근거다.
        hand = item.get("handling", "normal")
        if hand != "normal":
            h_font = fit_text(draw, hand.upper(), col_w - 4, int(2.8 * mm))
            bh = int(h_font.size * 1.5)
            draw.rectangle([right_x, y + 2, PX_W - quiet, y + 2 + bh], fill=(198, 40, 40))
            draw.text((right_x + 3, y + 2 + (bh - h_font.size) // 2 - 1), hand.upper(),
                      font=h_font, fill=(255, 255, 255))

    # --- 바코드 띠
    bx0, bx1 = quiet, PX_W - quiet
    by0 = PX_H - strip - bar_h
    img.paste(make_barcode(code, bx1 - bx0, bar_h), (bx0, by0))

    # --- 코드 문자열
    font = load_font(int(strip * 0.80))
    box = draw.textbbox((0, 0), code, font=font)
    draw.text(
        ((PX_W - (box[2] - box[0])) / 2, PX_H - strip + (strip - (box[3] - box[1])) / 2 - box[1]),
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
    hw = LABEL_W_MM / 2000.0
    hh = LABEL_H_MM / 2000.0
    MESH.parent.mkdir(parents=True, exist_ok=True)
    MESH.write_text(
        f"""# 박스 상면 송장 판. {LABEL_W_MM:.0f} x {LABEL_H_MM:.0f} mm, +z 법선, UV (0,0)-(1,1).
# tools/make_labels.py가 cell.yaml의 box.label에서 만들어 낸다.
# 직접 고치지 말 것.
o label_plane
v {-hw:.4f} {-hh:.4f} 0.0
v {hw:.4f} {-hh:.4f} 0.0
v {hw:.4f} {hh:.4f} 0.0
v {-hw:.4f} {hh:.4f} 0.0
vt 0.0 0.0
vt 1.0 0.0
vt 1.0 1.0
vt 0.0 1.0
vn 0.0 0.0 1.0
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""
    )
    print(f"  {MESH.relative_to(ROOT)}   {LABEL_W_MM:.0f} x {LABEL_H_MM:.0f} mm")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=TEX_DIR)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    for i, item in enumerate(ITEMS, start=1):
        path = args.out / f"box_{i}.png"
        make_label(item).save(path)
        print(f"  {path.relative_to(ROOT)}   {item['code']} 규격 {item.get('kind','?')}"
              f" {item.get('handling','normal')}")

    write_mesh()

    cb = args.out / "cardboard.png"
    make_cardboard().save(cb)
    print(f"  {cb.relative_to(ROOT)}")

    # 시드는 여기서 만들지 않는다. 읽기만 한다.
    #
    # 전에는 이 스크립트가 품목표를 들고 있다가 seed_items.json을 써 냈다.
    # 이제는 반대다. MES 시드가 원본이고 라벨이 거기서 나온다. 박스 규격,
    # 중량, 취급 구분이 전부 시드에 있고 라벨에 인쇄되기 때문이다.
    # 품목을 고치려면 seed_items.json을 고치고 이 스크립트를 다시 돌린다.
    if not ITEMS:
        print(f"  경고 : {SEED_JSON.relative_to(ROOT)}가 비었다. 라벨을 못 만든다.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
