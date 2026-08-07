"""카메라별 픽셀-map 좌표 변환 테스트."""

import unittest
from pathlib import Path

from aed_vision.homography import Homography


CONFIG_DIR = Path(__file__).parents[1] / "config"


class HomographyTest(unittest.TestCase):
    def test_box_to_map_uses_bbox_center(self):
        homography = Homography([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ])
        self.assertEqual(
            homography.box_to_map(10, 20, 30, 60),
            (20.0, 40.0),
        )

    def test_survey_points_map_within_calibration_error(self):
        for path in sorted(CONFIG_DIR.glob("homography_cam*.yaml")):
            homography = Homography.load(str(path))
            for item in homography.correspondences:
                actual = homography.pixel_to_map(*item["pixel"])
                expected = item["map"]
                error = sum(
                    (actual[index] - expected[index]) ** 2
                    for index in (0, 1)
                ) ** 0.5
                with self.subTest(camera=path.stem, point=item["label"]):
                    self.assertLess(error, 0.04)

    def test_box_coordinates_scale_to_survey_resolution(self):
        homography = Homography.load(str(CONFIG_DIR / "homography_cam1.yaml"))
        full = homography.box_to_map(300, 100, 346.6, 216.5)
        half = homography.box_to_map(
            150, 50, 173.3, 108.25, image_size=(320, 240)
        )
        self.assertAlmostEqual(full[0], half[0])
        self.assertAlmostEqual(full[1], half[1])

    def test_margin_accepts_calibration_boundary(self):
        homography = Homography.load(str(CONFIG_DIR / "homography_cam2.yaml"))
        x, y = homography.pixel_to_map(223.6, 133.8)
        self.assertTrue(homography.inside_survey_area(x, y, margin=0.15))


if __name__ == "__main__":
    unittest.main()
