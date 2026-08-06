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


def _default_config_path() -> str:
    """현장 측량 결과가 있으면 그것을 쓰고, 없으면 예시 항등행렬로 떨어진다."""
    config_dir = _config_dir()
    surveyed = os.path.join(config_dir, "homography.yaml")
    if os.path.exists(surveyed):
        return surveyed
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
    def load(cls, path: str = None) -> "Homography":
        config_path = path or _default_config_path()
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
