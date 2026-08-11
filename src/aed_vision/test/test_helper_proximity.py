"""RGB 화면 좌표 기반 조력자-환자 근접 필터 테스트."""

import numpy as np
import pytest

from aed_vision.detection_logic import Box, filter_helpers_near_fallen
from aed_vision.inference_pipeline import InferencePipeline


def test_keeps_helper_near_fallen_person() -> None:
    fallen = [Box(250, 250, 390, 390)]
    nearby = Box(390, 150, 470, 360)

    assert filter_helpers_near_fallen(
        [nearby], fallen, (640, 480), 0.30
    ) == [nearby]


def test_rejects_helper_far_from_fallen_person() -> None:
    fallen = [Box(20, 300, 140, 450)]
    far_away = Box(520, 20, 620, 220)

    assert filter_helpers_near_fallen(
        [far_away], fallen, (640, 480), 0.30
    ) == []


def test_rejects_every_helper_when_fallen_person_is_missing() -> None:
    helper = Box(200, 100, 300, 400)

    assert filter_helpers_near_fallen(
        [helper], [], (640, 480), 0.30
    ) == []


def test_rejects_invalid_distance_ratio() -> None:
    with pytest.raises(ValueError):
        filter_helpers_near_fallen(
            [Box(1, 1, 2, 2)], [Box(1, 1, 2, 2)], (640, 480), 0.0
        )


def test_rescue2_helper_requires_fallen_target_in_same_frame() -> None:
    pipeline = InferencePipeline.__new__(InferencePipeline)
    pipeline.person_model = None
    pipeline.skip_person_without_fallen = True
    pipeline.detect_people_as_helpers = False
    pipeline.enable_crowd = False
    pipeline.detection_backend = "mannequin_detect"
    pipeline.crowd_roi = [0.0, 0.0, 1.0, 1.0]
    pipeline.overlap_threshold = 0.4
    pipeline.helper_max_distance_ratio = 0.3
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    fallen = Box(250, 250, 390, 390)
    helper_rc_car = Box(390, 150, 470, 360)

    with_fallen = pipeline._detect_people(
        frame, [], [fallen], [helper_rc_car]
    )
    without_fallen = pipeline._detect_people(
        frame, [], [], [helper_rc_car]
    )

    assert with_fallen[1] == [helper_rc_car]
    assert without_fallen[1] == []


def test_robot_skips_coco_model_when_no_fallen_target_exists() -> None:
    class _PersonModel:
        def __init__(self) -> None:
            self.calls = 0

        def predict(self, *_args, **_kwargs):
            self.calls += 1
            raise AssertionError("COCO model must not run without fallen")

    pipeline = InferencePipeline.__new__(InferencePipeline)
    pipeline.person_model = _PersonModel()
    pipeline.skip_person_without_fallen = True
    pipeline.person_class_id = 0
    pipeline.detect_people_as_helpers = True
    pipeline.enable_crowd = False
    pipeline.detection_backend = "person_pose"
    pipeline.crowd_roi = [0.0, 0.0, 1.0, 1.0]
    pipeline.overlap_threshold = 0.4
    pipeline.helper_max_distance_ratio = 0.3

    result = pipeline._detect_people(
        np.zeros((480, 640, 3), dtype=np.uint8), [], [], []
    )

    assert pipeline.person_model.calls == 0
    assert result[1] == []
    assert result[2] == 0
