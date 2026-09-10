"""런타임 산출물이 어디에 쌓이는가.

컨테이너에서는 /tmp/box_cell 하나로 충분했다. 이미지 안이고, 쓰는 사람도
하나고, 컨테이너가 죽으면 같이 사라졌다.

호스트에 바로 깔면 사정이 다르다. /tmp는 여러 사람이 같이 쓰고, systemd의
PrivateTmp를 쓰는 서비스와 셸에서 띄운 노드가 서로 다른 /tmp를 보며,
재부팅마다 지워진다. 실물 셀에서 MES 큐와 적재 기록이 재부팅으로 사라지면
그건 사고다.

그래서 한 곳으로 모으고 환경 변수로 옮길 수 있게 한다.

    BOX_CELL_DATA_DIR=/var/lib/box_cell ros2 launch box_cell_bringup demo.launch.py

기본값은 예전 그대로 /tmp/box_cell이다. 지금 도는 것을 깨지 않는다.
"""

from __future__ import annotations

import os
from pathlib import Path

DEFAULT_DATA_DIR = "/tmp/box_cell"


def data_dir() -> Path:
    """산출물 디렉터리. 없으면 만든다."""
    d = Path(os.environ.get("BOX_CELL_DATA_DIR") or DEFAULT_DATA_DIR)
    try:
        d.mkdir(parents=True, exist_ok=True)
    except OSError:
        # 만들지 못해도 경로는 돌려준다. 쓰는 쪽에서 실패가 드러나는 편이
        # 여기서 조용히 다른 경로로 바꿔치기하는 것보다 낫다.
        pass
    return d


def data_path(*parts: str) -> str:
    """산출물 파일 경로를 문자열로. 파라미터 기본값에 그대로 쓴다."""
    return str(data_dir().joinpath(*parts))
