"""저장소에 포함된 실제 가중치를 사용하는 선택적 CPU 스모크 테스트."""

import os
from pathlib import Path

import cv2
import pytest

from aed_vision.inference_pipeline import InferenceOutput, InferencePipeline


pytestmark = [
    pytest.mark.model,
    pytest.mark.skipif(
        os.environ.get("AED_VISION_MODEL_TESTS") != "1",
        reason="set AED_VISION_MODEL_TESTS=1 to run local model tests",
    ),
]

ROOT = Path(__file__).resolve().parents[3]
MODELS = ROOT / "src/aed_vision/models"
IMAGE = ROOT / "docs/images/camera2_view_640x480.jpg"


def _pipeline(backend: str) -> InferencePipeline:
    return InferencePipeline(
        rescue_weights=str(MODELS / "rescue2_yolo11n.pt"),
        person_weights=str(MODELS / "coco_yolo11n.pt"),
        pose_weights=str(MODELS / "yolo11n-pose.pt"),
        posture_classifier_weights=(
            str(MODELS / "mannequin_posture_svm.xml")
            if backend == "mannequin_detect" else ""
        ),
        detection_backend=backend,
        enable_crowd=backend == "mannequin_detect",
        detect_people_as_helpers=backend == "person_pose",
        rescue_conf=0.5,
        person_conf=0.5,
        iou=0.5,
        imgsz=640,
        device="cpu",
        crowd_roi=[0.0, 0.0, 1.0, 1.0],
        overlap_threshold=0.4,
        helper_max_distance_ratio=0.3,
        pose_keypoint_conf=0.15,
        pose_min_keypoints=5,
        pose_min_box_area=0.005,
        pose_min_torso_keypoints=2,
        mannequin_bbox_fallback=True,
        mannequin_fallen_aspect_threshold=1.03,
        run_pose_for_mannequin=False,
    )


@pytest.mark.parametrize("backend", ["mannequin_detect", "person_pose"])
def test_local_model_pipeline_predicts_and_renders(backend: str) -> None:
    frame = cv2.imread(str(IMAGE))
    assert frame is not None
    pipeline = _pipeline(backend)
    if backend == "person_pose":
        # 낙상은 Pose, 동시 조력자는 별도 COCO person 모델이 담당해야 한다.
        assert pipeline.pose_model is not None
        assert pipeline.person_model is not None

    output = pipeline.predict(frame)
    assert isinstance(output, InferenceOutput)
    assert output.detection_backend == backend
    assert isinstance(output.fallen, list)
    assert isinstance(output.helpers, list)
    assert isinstance(output.person_count, int)
    assert output.inference_ms >= 0.0
    debug = pipeline.render_debug(output, "smoke_test")
    assert debug.shape == frame.shape
