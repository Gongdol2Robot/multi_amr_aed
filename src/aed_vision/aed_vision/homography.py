"""고정 카메라의 2D 영상 좌표를 로봇이 쓰는 2D map 좌표로 변환한다.

호모그래피는 바닥이 하나의 평면이라는 가정 아래 동작한다. 검출 bbox의 하단
중심을 사람의 발 위치로 보고 3x3 측량 행렬을 적용한다. 로봇 카메라는 계속
움직이므로 고정 행렬을 사용할 수 없고, 설정된 대표 좌표를 대신 사용한다.
"""

import os
from math import hypot

import numpy as np
import yaml


def _distance_to_segment(point, start, end) -> float:
    """2D 점에서 선분까지의 최단 거리를 계산한다."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return hypot(point[0] - start[0], point[1] - start[1])
    ratio = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    nearest_x = start[0] + ratio * dx
    nearest_y = start[1] + ratio * dy
    return hypot(point[0] - nearest_x, point[1] - nearest_y)


def _config_dir() -> str:
    """설치 환경 또는 소스 트리에서 호모그래피 설정 디렉터리를 찾는다."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory("aed_vision"), "config"
        )
    except Exception:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(here, "config")


def _default_config_path(camera_id: str = None) -> str:
    """카메라별 측량 결과를 찾고, 없으면 예시 항등행렬로 떨어진다.

    카메라마다 보는 각도가 달라 행렬을 공유할 수 없다. 그래서 파일을
    camera_id 별로 나눈다. camera_id 는 EmergencyEvent.camera_id 와 맞춘다.
    """
    config_dir = _config_dir()
    if camera_id:
        surveyed = os.path.join(config_dir, f"homography_{camera_id}.yaml")
        if os.path.exists(surveyed):
            return surveyed
        raise FileNotFoundError(
            f"{camera_id} 의 호모그래피 설정이 없습니다: {surveyed} — "
            f"tools/fit_homography.py 로 먼저 측량하세요."
        )
    return os.path.join(config_dir, "homography.example.yaml")


class Homography:
    """측량 행렬과 유효 측량 구역을 보관하고 픽셀을 map 좌표로 바꾼다."""

    def __init__(self, matrix, correspondences=None, camera=None,
                 survey_area=None):
        """픽셀→map 변환 행렬과 측량 메타데이터를 저장한다."""
        self.matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        self.correspondences = correspondences or []
        self.camera = camera or {}
        self.survey_area = survey_area or []

    @classmethod
    def load(cls, path: str = None, camera_id: str = None) -> "Homography":
        """camera_id 를 주면 그 카메라의 설정을 읽는다. 예: "cam1", "cam2"."""
        config_path = path or _default_config_path(camera_id)
        with open(config_path, encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
        return cls(
            config["homography"],
            config.get("correspondences"),
            config.get("camera"),
            config.get("survey_area"),
        )

    def box_to_map(
        self, x1: float, y1: float, x2: float, y2: float,
        image_size=None,
    ):
        """검출 박스의 바닥 접점을 측량 해상도에 맞춰 map 좌표로 바꾼다."""
        # 사람 몸통에 가까운 bbox 중심이 아니라 바닥 접점을 근사하는 하단
        # 중심을 사용해야 로봇이 접근할 실제 지면 좌표에 가까워진다.
        u, v = (x1 + x2) / 2.0, y2
        # 호모그래피 행렬은 측량 당시의 해상도(self.camera width/height) 기준으로
        # 만들어졌다. 실제 입력 해상도(image_size)가 다르면 비율로 스케일링해
        # 같은 물리적 지점이 같은 픽셀 좌표를 가리키도록 맞춘다.
        if image_size and self.camera:
            width, height = image_size
            u *= float(self.camera.get("width", width)) / width
            v *= float(self.camera.get("height", height)) / height
        # 동차좌표에 3x3 행렬을 곱한 뒤 perspective divide로 map 좌표를 얻는다.
        point = self.matrix @ np.array([u, v, 1.0])
        # point는 [X*w, Y*w, w] 형태의 동차좌표다. 앞의 두 값을 w로
        # 나누는 perspective divide를 해야 실제 map 좌표 (X, Y)가 된다.
        return float(point[0] / point[2]), float(point[1] / point[2])

    def inside_survey_area(
        self, x: float, y: float, margin: float = 0.0
    ) -> bool:
        """map 좌표가 측량 다각형 내부 또는 허용 여유 거리 안인지 판정한다."""
        # 명시한 측량 경계를 우선하고, 없으면 대응점의 map 좌표를 사용한다.
        polygon = (
            [tuple(point) for point in self.survey_area]
            if self.survey_area
            else [tuple(item["map"]) for item in self.correspondences]
        )
        if len(polygon) < 3:
            # 다각형을 정의할 점이 부족하면(측량 미비) 항상 안쪽으로 간주해
            # 잘못된 "영역 밖" 경고로 정상 검출을 버리지 않는다.
            return True
        # Ray casting(짝수/홀수 규칙): 점 (x, y)에서 수평 반직선을 오른쪽으로
        # 쏘아 다각형 변과 몇 번 교차하는지 센다. 교차 횟수가 홀수면 내부다.
        inside = False
        for index, (x1, y1) in enumerate(polygon):
            x2, y2 = polygon[(index + 1) % len(polygon)]
            # 이 변이 점의 y높이를 위아래로 가로지르는 경우에만 교차를 검사한다.
            if (y1 > y) != (y2 > y):
                # 변과 y=고정 수평선의 교차 x좌표를 선형보간으로 구한다.
                intersection = (x2 - x1) * (y - y1) / (y2 - y1) + x1
                if x < intersection:
                    inside = not inside
        if inside or margin <= 0.0:
            return inside
        # 다각형 안쪽이 아니어도, margin(m) 이내로 경계에 붙어 있으면 측량
        # 오차 범위로 보고 통과시킨다. 각 변에 대해 점에서 가장 가까운 점을
        # 구해 거리(hypot)를 계산한다.
        for index, start in enumerate(polygon):
            end = polygon[(index + 1) % len(polygon)]
            if _distance_to_segment((x, y), start, end) <= margin:
                return True
        return False
