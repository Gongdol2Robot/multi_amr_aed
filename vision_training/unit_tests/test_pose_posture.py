import unittest

from vision_training.testing.pose_posture import classify_posture


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


if __name__ == "__main__":
    unittest.main()
