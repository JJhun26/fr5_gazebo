#!/usr/bin/env python3
"""XML 주석 안의 '--'를 고치고 well-formed 여부를 검사한다.

xacro/URDF/SDF를 한글 주석과 함께 쓰다 보면 'C1 -- 컨베이어' 같은 표기가
XML 주석 규칙(주석 안에 '--' 금지)에 걸린다. 저장 직후 이 스크립트를 돌린다.
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def fix_comments(text: str) -> str:
    def repl(m: re.Match[str]) -> str:
        body = m.group(0)[4:-3]
        return "<!--" + body.replace("--", "—") + "-->"

    return re.sub(r"<!--.*?-->", repl, text, flags=re.S)


def main(paths: list[str]) -> int:
    bad = 0
    for p in paths:
        path = Path(p)
        original = path.read_text()
        fixed = fix_comments(original)
        if fixed != original:
            path.write_text(fixed)
        try:
            ET.parse(path)
            print(f"ok        {path}")
        except ET.ParseError as exc:
            print(f"BROKEN    {path}: {exc}")
            bad += 1
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
