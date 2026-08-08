"""Convert points between a fixed camera image and a ROS map."""

import os
from math import hypot

import numpy as np
import yaml


def _config_dir() -> str:
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
    """Transform floor-plane points between pixel and map coordinates."""

    def __init__(self, matrix, correspondences=None, camera=None,
                 survey_area=None):
        self.matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        self.inverse = np.linalg.inv(self.matrix)
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

    def pixel_to_map(self, u: float, v: float):
        # 동차좌표(homogeneous coordinate)로 변환 후 3x3 호모그래피 행렬을 곱한다.
        # 결과의 z(point[2])로 나눠 다시 2D로 투영(perspective divide)해야
        # 원근 왜곡이 반영된 실제 map 좌표가 나온다.
        point = self.matrix @ np.array([u, v, 1.0])
        return float(point[0] / point[2]), float(point[1] / point[2])

    def map_to_pixel(self, x: float, y: float):
        # pixel_to_map의 역변환. 역행렬(self.inverse)을 미리 구해뒀으므로
        # 매 호출마다 다시 invert하지 않는다.
        point = self.inverse @ np.array([x, y, 1.0])
        return float(point[0] / point[2]), float(point[1] / point[2])

    def box_to_map(
        self, x1: float, y1: float, x2: float, y2: float,
        image_size=None,
    ):
        """검출 박스 중심점을 측량 해상도에 맞춰 map 좌표로 바꾼다."""
        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        # 호모그래피 행렬은 측량 당시의 해상도(self.camera width/height) 기준으로
        # 만들어졌다. 실제 입력 해상도(image_size)가 다르면 비율로 스케일링해
        # 같은 물리적 지점이 같은 픽셀 좌표를 가리키도록 맞춘다.
        if image_size and self.camera:
            width, height = image_size
            u *= float(self.camera.get("width", width)) / width
            v *= float(self.camera.get("height", height)) / height
        return self.pixel_to_map(u, v)

    def survey_polygon(self):
        """측량 영역의 경계. 안쪽 측량점은 꼭짓점이 아니므로 껍질을 우선한다."""
        if self.survey_area:
            return [tuple(point) for point in self.survey_area]
        return [tuple(item["map"]) for item in self.correspondences]

    def inside_survey_area(
        self, x: float, y: float, margin: float = 0.0
    ) -> bool:
        polygon = self.survey_polygon()
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
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_squared = dx * dx + dy * dy
            # 점을 변 위로 투영한 비율(0=start, 1=end)을 구하고 [0,1]로 clamp해
            # "변의 연장선"이 아니라 "변 위의" 최근접점만 나오게 한다.
            ratio = 0.0 if length_squared == 0.0 else max(
                0.0,
                min(
                    1.0,
                    ((x - start[0]) * dx + (y - start[1]) * dy)
                    / length_squared,
                ),
            )
            nearest_x = start[0] + ratio * dx
            nearest_y = start[1] + ratio * dy
            if hypot(x - nearest_x, y - nearest_y) <= margin:
                return True
        return False


def load_all() -> dict:
    """설치된 카메라별 호모그래피를 모두 읽어 camera_id 로 색인한다.

    검출은 어느 카메라에서 올지 모르므로, EmergencyEvent.camera_id 로 바로
    꺼내 쓸 수 있게 미리 전부 읽어 둔다. 측량이 안 된 카메라는 아예 없다.
    """
    import glob

    result = {}
    pattern = os.path.join(_config_dir(), "homography_*.yaml")
    for path in sorted(glob.glob(pattern)):
        name = os.path.basename(path)[len("homography_"):-len(".yaml")]
        result[name] = Homography.load(path=path)
    return result
