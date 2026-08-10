"""지도 창구. 화면이 로봇 위치를 지도 위에 찍을 수 있게 한다.

지도는 두 조각으로 나눠 준다.

  /api/map/meta    해상도와 원점. 지도 좌표(m) → 그림 좌표(px) 변환에 쓴다
  /api/map/image   그림 자체(PNG)

그림과 계수를 한 응답에 담지 않는 이유는 크기가 다르기 때문이다. 계수는
몇 바이트라 화면이 뜰 때 한 번 받으면 되고, 그림은 수 KB 라 브라우저가
캐시하게 두는 편이 낫다. `<img src>` 로 직접 받으면 캐시가 저절로 된다.

PGM 은 브라우저가 못 읽는다. 그래서 서버가 뜰 때 한 번 PNG 로 바꿔 메모리에
들고 있는다. 지도는 시연 중에 바뀌지 않으므로 매 요청마다 다시 만들 이유가
없다.
"""

import logging
import os

import cv2
import yaml
from fastapi import APIRouter, HTTPException, Response

LOGGER = logging.getLogger(__name__)

router = APIRouter(prefix="/api/map", tags=["map"])

# 저장소 안의 공용 지도. 두 로봇이 같은 지도를 쓴다.


def _repo_root() -> str:
    """maps/map.yaml 이 있는 디렉터리를 위로 거슬러 찾는다.

    단수를 세어 올라가면 소스에서 실행할 때만 맞는다. colcon 이 설치한
    install/aed_hmi/lib/... 아래에서 돌면 깊이가 달라 엉뚱한 곳을 가리킨다.
    실제로 launch 로 띄웠을 때 install/aed_hmi/lib/maps/map.yaml 을 찾아
    지도가 안 떴다. 그래서 개수가 아니라 존재로 찾는다.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(here, "maps", "map.yaml")):
            return here
        parent = os.path.dirname(here)
        if parent == here:      # 루트까지 갔는데 없다
            return here
        here = parent


MAP_YAML = os.environ.get(
    "AED_HMI_MAP", os.path.join(_repo_root(), "maps", "map.yaml")
)


class _Map:
    """지도 한 장. 서버가 뜰 때 한 번 읽어 두고 그대로 쓴다."""

    def __init__(self, yaml_path: str) -> None:
        self.ok = False
        self.png: bytes = b""
        self.meta: dict = {}
        try:
            self._load(yaml_path)
            self.ok = True
        except Exception:
            # 지도가 없어도 나머지 화면은 떠야 한다.
            LOGGER.warning("지도를 못 읽었다: %s", yaml_path, exc_info=True)

    def _load(self, yaml_path: str) -> None:
        with open(yaml_path, encoding="utf-8") as handle:
            data = yaml.safe_load(handle)

        image_path = data["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        grid = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        if grid is None:
            raise FileNotFoundError(image_path)
        height, width = grid.shape[:2]

        # PGM 은 값이 클수록 밝다(=빈 공간). 화면에서는 벽이 진하게 보여야
        # 하므로 그대로 쓴다. 다만 미지 영역(회색)은 배경과 구분되게 낮춘다.
        if grid.ndim == 2:
            grid = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)

        ok, buffer = cv2.imencode(".png", grid)
        if not ok:
            raise RuntimeError("PNG 인코딩 실패")

        self.png = buffer.tobytes()
        origin = data.get("origin", [0.0, 0.0, 0.0])
        self.meta = {
            "width": width,
            "height": height,
            "resolution": float(data["resolution"]),
            "origin_x": float(origin[0]),
            "origin_y": float(origin[1]),
            # 화면이 좌표를 픽셀로 바꿀 때 쓰는 식을 같이 적어 둔다.
            # px = (x - origin_x) / resolution
            # py = height - (y - origin_y) / resolution      ← y 축이 뒤집힌다
            "note": "px=(x-origin_x)/resolution, py=height-(y-origin_y)/resolution",
        }


_MAP: _Map | None = None


def _map() -> _Map:
    global _MAP
    if _MAP is None:
        _MAP = _Map(MAP_YAML)
    return _MAP


@router.get("/meta")
async def meta():
    """지도 좌표를 그림 좌표로 바꾸는 계수."""
    loaded = _map()
    if not loaded.ok:
        raise HTTPException(status_code=404, detail="지도 없음")
    return loaded.meta


@router.get("/image")
async def image():
    """지도 그림. 시연 중 바뀌지 않으므로 브라우저가 캐시하게 둔다."""
    loaded = _map()
    if not loaded.ok:
        raise HTTPException(status_code=404, detail="지도 없음")
    return Response(
        content=loaded.png,
        media_type="image/png",
        headers={"Cache-Control": "public, max-age=3600"},
    )
