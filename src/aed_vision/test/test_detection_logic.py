"""비전 검출 후처리 로직 단위 테스트."""

import unittest

from aed_vision.detection_logic import (
    Box,
    TemporalConfirmation,
    classify_crowd,
    count_crowd_people,
    apply_crowd_time_penalty,
    crowd_time_multiplier,
    intersection_over_union,
)


class DetectionLogicTest(unittest.TestCase):
    """ROS와 모델 없이 실행 가능한 후처리 동작을 검증한다."""

    def test_temporal_confirmation_requires_six_of_ten_frames(self):
        confirmation = TemporalConfirmation(window_size=10, required_hits=6)
        states = [
            confirmation.update(value)
            for value in [1, 1, 0, 1, 0, 1, 1, 1]
        ]
        self.assertFalse(states[-2])
        self.assertTrue(states[-1])
        self.assertEqual(confirmation.hit_count, 6)

    def test_temporal_confirmation_releases_when_hits_leave_window(self):
        confirmation = TemporalConfirmation(window_size=3, required_hits=2)
        self.assertFalse(confirmation.update(True))
        self.assertTrue(confirmation.update(True))
        self.assertTrue(confirmation.update(False))
        self.assertFalse(confirmation.update(False))

    def test_invalid_confirmation_configuration(self):
        with self.assertRaises(ValueError):
            TemporalConfirmation(window_size=2, required_hits=3)

    def test_iou_for_partially_overlapping_boxes(self):
        actual = intersection_over_union(
            Box(0, 0, 10, 10), Box(5, 0, 15, 10)
        )
        self.assertAlmostEqual(actual, 1.0 / 3.0)

    def test_crowd_count_uses_roi_and_excludes_fallen_overlap(self):
        people = [
            Box(10, 10, 30, 50),  # ROI 밖
            Box(40, 20, 70, 80),  # fallen과 동일해 제외
            Box(70, 20, 90, 80),  # 유효한 인파
        ]
        fallen = [Box(40, 20, 70, 80)]
        count = count_crowd_people(
            people,
            fallen,
            frame_size=(100, 100),
            roi=(0.3, 0.0, 1.0, 1.0),
            overlap_threshold=0.4,
        )
        self.assertEqual(count, 1)

    def test_classify_crowd(self):
        expected_states = [
            (0, 0),
            (1, 1),
            (2, 2),
            (3, 3),
            (5, 3),
        ]
        for person_count, expected in expected_states:
            with self.subTest(person_count=person_count):
                self.assertEqual(classify_crowd(person_count), expected)

    def test_crowd_time_penalty(self):
        self.assertEqual(crowd_time_multiplier(0), 1.0)
        self.assertEqual(crowd_time_multiplier(1), 1.1)
        self.assertEqual(crowd_time_multiplier(2), 1.2)
        self.assertIsNone(crowd_time_multiplier(3))
        self.assertIsNone(crowd_time_multiplier(10))
        self.assertAlmostEqual(apply_crowd_time_penalty(100.0, 1), 110.0)
        self.assertAlmostEqual(apply_crowd_time_penalty(100.0, 2), 120.0)
        self.assertIsNone(apply_crowd_time_penalty(100.0, 3))

    def test_crowd_values_must_not_be_negative(self):
        with self.assertRaises(ValueError):
            classify_crowd(-1)
        with self.assertRaises(ValueError):
            apply_crowd_time_penalty(-1.0, 0)


if __name__ == "__main__":
    unittest.main()
