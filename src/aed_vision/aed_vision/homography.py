"""Convert points between a fixed camera image and a ROS map."""

import os

import numpy as np
import yaml


def _default_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory("aed_vision"),
            "config",
            "homography.example.yaml",
        )
    except Exception:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(here, "config", "homography.example.yaml")


class Homography:
    """Transform floor-plane points between pixel and map coordinates."""

    def __init__(self, matrix, correspondences=None, camera=None):
        self.matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        self.inverse = np.linalg.inv(self.matrix)
        self.correspondences = correspondences or []
        self.camera = camera or {}

    @classmethod
    def load(cls, path: str = None) -> "Homography":
        config_path = path or _default_config_path()
        with open(config_path, encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
        return cls(
            config["homography"],
            config.get("correspondences"),
            config.get("camera"),
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

