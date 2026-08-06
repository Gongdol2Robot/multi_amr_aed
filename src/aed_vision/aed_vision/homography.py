"""Convert points between a fixed camera image and a ROS map."""

import os

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
        point = self.matrix @ np.array([u, v, 1.0])
        return float(point[0] / point[2]), float(point[1] / point[2])

    def map_to_pixel(self, x: float, y: float):
        point = self.inverse @ np.array([x, y, 1.0])
        return float(point[0] / point[2]), float(point[1] / point[2])

    def box_to_map(self, x1: float, y1: float, x2: float, y2: float):
        """Use the lower-center of a detection box as its floor contact."""
        return self.pixel_to_map((x1 + x2) / 2.0, y2)

    def survey_polygon(self):
        """측량 영역의 경계. 안쪽 측량점은 꼭짓점이 아니므로 껍질을 우선한다."""
        if self.survey_area:
            return [tuple(point) for point in self.survey_area]
        return [tuple(item["map"]) for item in self.correspondences]

    def inside_survey_area(self, x: float, y: float) -> bool:
        polygon = self.survey_polygon()
        if len(polygon) < 3:
            return True
        inside = False
        for index, (x1, y1) in enumerate(polygon):
            x2, y2 = polygon[(index + 1) % len(polygon)]
            if (y1 > y) != (y2 > y):
                intersection = (x2 - x1) * (y - y1) / (y2 - y1) + x1
                if x < intersection:
                    inside = not inside
        return inside


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

