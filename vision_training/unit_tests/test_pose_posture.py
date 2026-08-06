import unittest

from vision_training.testing.pose_posture import PostureHistory, classify_posture


def person(shoulders, hips, knees, confidence=1.0):
    points = [[0.0, 0.0, 0.0] for _ in range(17)]
    body_parts = (
        ((5, 6), shoulders),
        ((11, 12), hips),
        ((13, 14), knees),
    )
    for indexes, center in body_parts:
        for index in indexes:
            points[index] = [center[0], center[1], confidence]
    return points


class PosePostureTest(unittest.TestCase):
    def test_standing(self):
        pose = person((50, 20), (52, 60), (53, 90))
        self.assertEqual(
            classify_posture(pose, (30, 5, 75, 105))[0], "STANDING"
        )

    def test_sitting(self):
        pose = person((50, 20), (52, 55), (85, 65))
        self.assertEqual(
            classify_posture(pose, (25, 5, 105, 90))[0], "SITTING"
        )

    def test_fallen(self):
        pose = person((30, 50), (80, 55), (115, 58))
        self.assertEqual(
            classify_posture(pose, (10, 30, 135, 80))[0], "FALLEN"
        )

    def test_fallen_requires_repeated_hits(self):
        history = PostureHistory(window=5, fallen_hits=3)
        outputs = [
            history.update(7, posture, frame)
            for frame, posture in enumerate(
                ("STANDING", "FALLEN", "STANDING", "FALLEN", "FALLEN")
            )
        ]
        self.assertNotEqual(outputs[3], "FALLEN")
        self.assertEqual(outputs[4], "FALLEN")

    def test_first_fallen_frame_is_not_confirmed(self):
        history = PostureHistory(window=5, fallen_hits=3)
        self.assertEqual(history.update(7, "FALLEN", 0), "UNKNOWN")

    def test_history_is_separate_per_track(self):
        history = PostureHistory(window=3, fallen_hits=2)
        history.update(1, "FALLEN", 0)
        history.update(1, "FALLEN", 1)
        self.assertEqual(history.update(2, "STANDING", 1), "STANDING")


if __name__ == "__main__":
    unittest.main()
