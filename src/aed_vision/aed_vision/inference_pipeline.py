"""YOLO 모델 로딩, 구조·혼잡 추론과 디버그 영상 렌더링."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

from .detection_logic import (
    Box,
    classify_crowd,
    crowd_time_multiplier,
    filter_nonfallen_people,
)
from .pose_posture import TORSO_INDEXES, classify_posture


RESCUE_CLASS_NAMES = ["fallen_person", "helper_rc_car"]
DISPLAY_NAMES = {0: "fallen_person", 1: "helper"}
DETECTION_BACKENDS = ("person_pose", "mannequin_detect")
POSTURE_COLORS = {
    "STANDING": (0, 200, 0),
    "SITTING": (0, 200, 255),
    "FALLEN": (0, 0, 255),
    "UNKNOWN": (160, 160, 160),
}
# COCO 17관절: 코(0), 눈(1~2), 귀(3~4), 어깨(5~6), 팔(7~10),
# 엉덩이(11~12), 무릎(13~14), 발목(15~16).
POSE_SKELETON = (
    (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 6), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
    (0, 1), (0, 2), (1, 3), (2, 4),
)


def _model_path(value: str, description: str) -> Path:
    # "package://<pkg>/<relative>" 형식이면 ROS 패키지의 설치 경로(share
    # 디렉터리) 기준으로 절대경로를 만든다. 이렇게 하면 YAML에 노트북마다
    # 다른 절대경로를 적지 않아도 된다(README의 이유와 동일).
    if value.startswith("package://"):
        from ament_index_python.packages import get_package_share_directory

        package, separator, relative = value[10:].partition("/")
        if not separator or not package or not relative:
            raise RuntimeError(
                f"Invalid package model URI for {description}: {value}"
            )
        path = (
            Path(get_package_share_directory(package)) / relative
        ).resolve()
    else:
        # 일반 경로는 환경변수와 ~를 확장해 사용한다.
        path = Path(os.path.expandvars(value)).expanduser().resolve()
    if not path.is_file():
        # 모델 파일이 없으면(예: colcon build 전) 여기서 바로 실패시켜
        # 나중에 알 수 없는 추론 에러로 이어지지 않게 한다.
        raise RuntimeError(f"{description} not found: {path}")
    return path


def _names(model) -> list[str]:
    # Ultralytics 모델의 클래스 이름은 버전/모델에 따라 dict 또는 list로 온다.
    # dict일 때는 정수 키 순서(class id 순)로 정렬해 리스트로 통일한다.
    names = model.names
    if isinstance(names, dict):
        return [str(names[key]) for key in sorted(names, key=int)]
    if isinstance(names, (list, tuple)):
        return [str(name) for name in names]
    raise RuntimeError(f"Unsupported model names: {names}")


def _boxes(result, class_id: int) -> list[Box]:
    """YOLO 추론 결과에서 특정 class_id의 bbox만 뽑아 순수 Box 리스트로 바꾼다.

    Ultralytics 텐서(result.boxes)를 여기서 즉시 CPU/Python 값으로 꺼내므로,
    이후 detection_logic의 순수 함수들은 GPU/torch를 몰라도 된다.
    """
    if result.boxes is None:
        return []
    selected = result.boxes[result.boxes.cls == class_id]
    return [
        Box(*xyxy, confidence=float(confidence))
        for xyxy, confidence in zip(
            selected.xyxy.cpu().tolist(), selected.conf.cpu().tolist()
        )
    ]


@dataclass
class PoseEvidence:
    box: Box
    keypoints: np.ndarray
    posture: str
    aspect_ratio: float
    torso_angle_deg: float
    visible_keypoints: int


@dataclass
class InferenceOutput:
    rescue_result: object
    person_result: object | None
    fallen: list[Box]
    helpers: list[Box]
    person_count: int
    crowd_level: int | None
    crowd_time_multiplier: float | None
    crowd_traversable: bool
    inference_ms: float
    detection_backend: str
    pose_evidence: list[PoseEvidence]


class InferencePipeline:
    """카메라 모드에 맞는 YOLO 모델과 후처리를 캡슐화한다."""

    def __init__(
        self,
        *,
        rescue_weights: str,
        person_weights: str,
        pose_weights: str,
        detection_backend: str,
        enable_crowd: bool,
        detect_people_as_helpers: bool,
        rescue_conf: float,
        person_conf: float,
        iou: float,
        imgsz: int,
        device: str,
        crowd_roi: list[float],
        crowded_threshold: int,
        overlap_threshold: float,
        pose_keypoint_conf: float,
        pose_min_keypoints: int,
        pose_min_box_area: float,
        pose_min_torso_keypoints: int,
    ) -> None:
        from ultralytics import YOLO

        self.enable_crowd = enable_crowd
        self.detect_people_as_helpers = detect_people_as_helpers
        self.detection_backend = detection_backend.strip().lower()
        if self.detection_backend not in DETECTION_BACKENDS:
            raise ValueError(
                f"detection_backend must be one of {DETECTION_BACKENDS}, "
                f"got {detection_backend!r}"
            )
        self.rescue_conf = rescue_conf
        self.person_conf = person_conf
        self.pose_keypoint_conf = pose_keypoint_conf
        self.pose_min_keypoints = pose_min_keypoints
        self.pose_min_box_area = pose_min_box_area
        self.pose_min_torso_keypoints = pose_min_torso_keypoints
        if not 0.0 <= self.pose_keypoint_conf <= 1.0:
            raise ValueError("pose_keypoint_conf must be between 0 and 1")
        if not 1 <= self.pose_min_keypoints <= 17:
            raise ValueError("pose_min_keypoints must be between 1 and 17")
        if not 0.0 <= self.pose_min_box_area <= 1.0:
            raise ValueError("pose_min_box_area must be between 0 and 1")
        if not 1 <= self.pose_min_torso_keypoints <= len(TORSO_INDEXES):
            raise ValueError("pose_min_torso_keypoints must be between 1 and 4")
        self.crowd_roi = crowd_roi
        # 하위 호환을 위해 파라미터는 받지만 혼잡 등급은 이제 사람 수 1/2/3을
        # 직접 사용한다.
        self.crowded_threshold = crowded_threshold
        self.overlap_threshold = overlap_threshold
        self.options = {"iou": iou, "imgsz": imgsz, "verbose": False}
        if device:
            self.options["device"] = device

        # backend에 따라 두 모델 중 하나만 로드한다(메모리·연산량 절약).
        # mannequin_detect: 목각인형 파인튜닝 검출 모델.
        # person_pose(기본): 실사람 관절 검출용 Pose 모델.
        self.rescue_model = None
        self.pose_model = None
        if self.detection_backend == "mannequin_detect":
            self.rescue_model = YOLO(
                str(_model_path(rescue_weights, "rescue weights"))
            )
            rescue_names = _names(self.rescue_model)
            # 클래스 순서가 어긋나면 class_id 0/1을 fallen/helper로 오인하므로
            # 로딩 시점에 바로 검증한다.
            if rescue_names != RESCUE_CLASS_NAMES:
                raise RuntimeError(
                    f"Rescue classes must be {RESCUE_CLASS_NAMES}, "
                    f"got {rescue_names}"
                )
        else:
            self.pose_model = YOLO(
                str(_model_path(pose_weights, "person pose weights"))
            )
            if self.pose_model.task != "pose":
                raise RuntimeError(
                    f"Person pose weights must have task=pose, "
                    f"got {self.pose_model.task}"
                )

        # 자세 판정과 혼잡도 판정은 목적과 품질 기준이 다르다. person_pose의
        # bbox는 관절 품질 필터를 통과한 사람만 남으므로 인파를 누락할 수 있다.
        # 따라서 골목(enable_crowd)에서는 backend와 무관하게 COCO person 모델을
        # 별도로 로드한다. mannequin_detect의 helper 판정도 같은 모델을 쓴다.
        self.person_model = None
        self.person_class_id = -1
        if (
            enable_crowd
            or (
                self.detection_backend == "mannequin_detect"
                and detect_people_as_helpers
            )
        ):
            self.person_model = YOLO(
                str(_model_path(person_weights, "COCO person weights"))
            )
            person_names = _names(self.person_model)
            if "person" not in person_names:
                raise RuntimeError(
                    "The person model does not contain COCO person"
                )
            self.person_class_id = person_names.index("person")

    def _pose_detections(self, result, frame_shape):
        """Pose 결과를 사람 bbox, 쓰러진 bbox, 판단 근거로 변환한다."""
        people: list[Box] = []
        fallen: list[Box] = []
        evidence: list[PoseEvidence] = []
        if (
            result.boxes is None or result.keypoints is None
            or result.keypoints.conf is None
        ):
            return people, fallen, evidence

        height, width = frame_shape[:2]
        frame_area = float(height * width)
        boxes = result.boxes.xyxy.cpu().numpy()
        box_confidences = result.boxes.conf.cpu().numpy()
        points = result.keypoints.xy.cpu().numpy()
        scores = result.keypoints.conf.cpu().numpy()
        for xyxy, box_conf, xy, confidence in zip(
            boxes, box_confidences, points, scores
        ):
            box = Box(*xyxy, confidence=float(box_conf))
            box_area = max(box.x2 - box.x1, 0.0) * max(box.y2 - box.y1, 0.0)
            # 관절별 confidence가 임계값 이상인 것만 "보인다"고 취급한다.
            visible = confidence >= self.pose_keypoint_conf
            torso_visible = int(visible[list(TORSO_INDEXES)].sum())
            # 품질 필터: (1) bbox가 화면에서 너무 작거나(멀리 있어 신뢰 낮음),
            # (2) 전체 보이는 관절 수가 부족하거나, (3) 자세 판정에 필요한
            # 몸통(어깨·엉덩이) 관절이 부족하면 이 사람은 후보에서 제외한다.
            if (
                box_area / max(frame_area, 1.0) < self.pose_min_box_area
                or int(visible.sum()) < self.pose_min_keypoints
                or torso_visible < self.pose_min_torso_keypoints
            ):
                continue
            keypoints = np.column_stack((xy, confidence))
            posture, metrics = classify_posture(
                keypoints, xyxy, keypoint_conf=self.pose_keypoint_conf
            )
            # 필터를 통과한 사람은 person_count(인파)에도 포함시킨다.
            people.append(box)
            if posture == "FALLEN":
                fallen.append(box)
            evidence.append(
                PoseEvidence(
                    box=box,
                    keypoints=keypoints,
                    posture=posture,
                    aspect_ratio=metrics["aspect_ratio"],
                    torso_angle_deg=metrics["torso_angle_deg"],
                    visible_keypoints=int(visible.sum()),
                )
            )
        return people, fallen, evidence

    def predict(self, frame) -> InferenceOutput:
        started = perf_counter()
        pose_evidence: list[PoseEvidence] = []
        # 1단계: 쓰러짐(fallen) 검출. backend에 따라 소스가 다르다.
        if self.detection_backend == "mannequin_detect":
            # 파인튜닝 모델 하나로 fallen_person(class 0)과 helper_rc_car(class 1)를
            # 동시에 얻는다.
            rescue_result = self.rescue_model.predict(
                frame, conf=self.rescue_conf, **self.options
            )[0]
            fallen = _boxes(rescue_result, 0)
            helpers = _boxes(rescue_result, 1)
            pose_people: list[Box] = []
        else:
            # Pose 모델은 helper를 구분하지 않으므로 helpers는 빈 채로 시작하고,
            # 필요하면(detect_people_as_helpers) 아래에서 people로 채운다.
            rescue_result = self.pose_model.predict(
                frame, conf=self.person_conf, **self.options
            )[0]
            pose_people, fallen, pose_evidence = self._pose_detections(
                rescue_result, frame.shape
            )
            helpers = []
        person_result = None
        person_count = 0
        crowd_level = None
        time_multiplier = None
        crowd_traversable = True

        # 2단계: 인원수/혼잡도 계산. 혼잡도를 사용하는 골목 모드는 반드시
        # COCO person 결과를 사용한다. ROI 밖 사람과 쓰러진 환자 bbox와 겹치는
        # 검출은 filter_nonfallen_people에서 제외한다.
        if self.person_model is not None:
            person_result = self.person_model.predict(
                frame,
                conf=self.person_conf,
                classes=[self.person_class_id],
                **self.options,
            )[0]
            height, width = frame.shape[:2]
            people = filter_nonfallen_people(
                _boxes(person_result, self.person_class_id),
                fallen,
                (width, height),
                self.crowd_roi,
                self.overlap_threshold,
            )
            person_count = len(people)
            if self.detect_people_as_helpers:
                helpers = people
            if self.enable_crowd:
                crowd_level = classify_crowd(person_count)
                time_multiplier = crowd_time_multiplier(person_count)
                crowd_traversable = time_multiplier is not None
        elif self.detection_backend == "person_pose" and self.detect_people_as_helpers:
            # 혼잡도 없이 Pose를 쓰는 로봇 모드는 별도 COCO 호출을 하지 않고,
            # 관절 품질 필터를 통과한 사람 bbox를 helper 후보로 재사용한다.
            height, width = frame.shape[:2]
            people = filter_nonfallen_people(
                pose_people,
                fallen,
                (width, height),
                self.crowd_roi,
                self.overlap_threshold,
            )
            person_count = len(people)
            helpers = people

        return InferenceOutput(
            rescue_result,
            person_result,
            fallen,
            helpers,
            person_count,
            crowd_level,
            time_multiplier,
            crowd_traversable,
            (perf_counter() - started) * 1000.0,
            self.detection_backend,
            pose_evidence,
        )

    def render_debug(self, output: InferenceOutput, camera_id: str):
        if output.detection_backend == "mannequin_detect":
            # Ultralytics 기본 plot()에 class id 대신 사람이 읽을 이름을
            # 보여주려고 표시 직전에만 names를 덮어쓴다.
            output.rescue_result.names = DISPLAY_NAMES
            image = output.rescue_result.plot()
        else:
            # Pose 결과는 자체 plot()을 쓰지 않고 자세·골격을 직접 그려
            # posture 라벨과 색상(POSTURE_COLORS)을 함께 표시한다.
            image = output.rescue_result.orig_img.copy()
            for evidence in output.pose_evidence:
                color = POSTURE_COLORS[evidence.posture]
                box = evidence.box
                cv2.rectangle(
                    image, (int(box.x1), int(box.y1)),
                    (int(box.x2), int(box.y2)), color, 2,
                )
                for first, second in POSE_SKELETON:
                    first_point = evidence.keypoints[first]
                    second_point = evidence.keypoints[second]
                    if (
                        first_point[2] >= self.pose_keypoint_conf
                        and second_point[2] >= self.pose_keypoint_conf
                    ):
                        cv2.line(
                            image,
                            (int(first_point[0]), int(first_point[1])),
                            (int(second_point[0]), int(second_point[1])),
                            color,
                            2,
                            cv2.LINE_AA,
                        )
                for x, y, confidence in evidence.keypoints:
                    if confidence >= self.pose_keypoint_conf:
                        cv2.circle(image, (int(x), int(y)), 3, color, -1)
                cv2.putText(
                    image,
                    f"{evidence.posture} ar={evidence.aspect_ratio:.2f} "
                    f"torso={evidence.torso_angle_deg:.0f}",
                    (int(box.x1), max(int(box.y1) - 8, 22)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2,
                )
        boxes = getattr(output.person_result, "boxes", None)
        if boxes is not None:
            for x1, y1, x2, y2 in boxes.xyxy.int().cpu().tolist():
                cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 255), 2)

        if self.enable_crowd:
            height, width = image.shape[:2]
            x1, y1, x2, y2 = (
                int(value * size)
                for value, size in zip(
                    self.crowd_roi, (width, height, width, height)
                )
            )
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 2)
            text = (
                f"{camera_id} | crowd={output.crowd_level} | "
                f"people={output.person_count}"
            )
        elif self.detect_people_as_helpers:
            text = f"{camera_id} | helper candidates={len(output.helpers)}"
        else:
            text = f"{camera_id} | backend={output.detection_backend}"
        cv2.putText(
            image,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return image
