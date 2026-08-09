"""YOLO 모델 로딩, 구조·혼잡 추론과 디버그 영상 렌더링."""

from __future__ import annotations

import os
import warnings
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

from .detection_logic import (
    Box,
    classify_crowd,
    crowd_time_multiplier,
    filter_helpers_near_fallen,
    filter_nonfallen_people,
    is_fallen_bbox_candidate,
)
from .pose_posture import TORSO_INDEXES, classify_posture
from .posture_classifier import PostureClassifier


RESCUE_CLASS_NAMES = ["mannequin", "helping_person"]
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
    """ROS 패키지 URI나 일반 경로를 검증된 모델 파일 절대경로로 바꾼다."""
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
    """Ultralytics 모델의 클래스 이름을 class ID 순서의 문자열 목록으로 만든다."""
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
        overlap_threshold: float,
        helper_max_distance_ratio: float,
        pose_keypoint_conf: float,
        pose_min_keypoints: int,
        pose_min_box_area: float,
        pose_min_torso_keypoints: int,
        mannequin_bbox_fallback: bool = True,
        mannequin_fallen_aspect_threshold: float = 1.03,
        posture_classifier_weights: str = "",
    ) -> None:
        """추론 설정을 검증하고 선택한 backend에 필요한 YOLO 모델을 로드한다."""
        import torch
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
        self.mannequin_bbox_fallback = mannequin_bbox_fallback
        self.mannequin_fallen_aspect_threshold = (
            mannequin_fallen_aspect_threshold
        )
        self.posture_classifier = None
        if not 0.0 <= self.pose_keypoint_conf <= 1.0:
            raise ValueError("pose_keypoint_conf must be between 0 and 1")
        if not 1 <= self.pose_min_keypoints <= 17:
            raise ValueError("pose_min_keypoints must be between 1 and 17")
        if not 0.0 <= self.pose_min_box_area <= 1.0:
            raise ValueError("pose_min_box_area must be between 0 and 1")
        if not 1 <= self.pose_min_torso_keypoints <= len(TORSO_INDEXES):
            raise ValueError(
                "pose_min_torso_keypoints must be between 1 and 4"
            )
        if self.mannequin_fallen_aspect_threshold <= 0.0:
            raise ValueError(
                "mannequin_fallen_aspect_threshold must be positive"
            )
        self.crowd_roi = crowd_roi
        self.overlap_threshold = overlap_threshold
        self.helper_max_distance_ratio = helper_max_distance_ratio
        if not 0.0 < self.helper_max_distance_ratio <= 1.0:
            raise ValueError(
                "helper_max_distance_ratio must be in (0, 1]"
            )
        self.options = {"iou": iou, "imgsz": imgsz, "verbose": False}

        # YAML에서 cuda/cuda:N을 명시했더라도 CUDA 런타임이나 해당 번호의 GPU가
        # 없으면 노드 시작을 실패시키지 않고 CPU 추론으로 전환한다. 빈 문자열은
        # Ultralytics의 자동 장치 선택을 유지하며, cpu 등 다른 값도 그대로 넘긴다.
        requested_device = device.strip()
        if requested_device.lower().startswith("cuda"):
            cuda_unavailable = not torch.cuda.is_available()
            if not cuda_unavailable and ":" in requested_device:
                try:
                    # CUDA 자체가 사용 가능해도 cuda:1처럼 존재하지 않는 GPU를
                    # 요청할 수 있으므로 실제 장치 개수까지 확인한다.
                    cuda_index = int(requested_device.rsplit(":", 1)[1])
                    cuda_unavailable = not (
                        0 <= cuda_index < torch.cuda.device_count()
                    )
                except ValueError:
                    # 잘못된 device 문자열은 Ultralytics가 명확한 오류로 보고한다.
                    pass
            if cuda_unavailable:
                # warnings를 사용해 ROS 로그뿐 아니라 단독 실행/테스트에서도
                # fallback 사실이 노출되도록 한다.
                warnings.warn(
                    f"Requested inference device {requested_device!r} is not "
                    "available; falling back to CPU.",
                    RuntimeWarning,
                    stacklevel=2,
                )
                requested_device = "cpu"
        if requested_device:
            self.options["device"] = requested_device

        # mannequin_detect는 검출 bbox를 만든 뒤 그 crop에 Pose를 적용하고,
        # person_pose는 전체 프레임에 Pose를 바로 적용한다.
        self.rescue_model = None
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
            if posture_classifier_weights.strip():
                self.posture_classifier = PostureClassifier(
                    _model_path(
                        posture_classifier_weights,
                        "mannequin posture classifier",
                    )
                )
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

    def _mannequin_pose_detections(self, frame, detection_result):
        """mannequin bbox마다 SVM과 Pose를 적용해 낙상 후보를 반환한다.

        1단계 rescue detector의 mannequin bbox를 받아 너무 작은 검출을 제거하고,
        bbox 주변 crop에 전용 HOG+SVM과 사람용 Pose를 실행한다. 최종 자세는
        SVM, bbox fallback, Pose 순으로 보완하며 여기서 만든 ``fallen``은 아직
        프레임 단위 후보다. 시간 누적 응급 확정은 VisionDetector가 담당한다.
        """
        # 최종 낙상 후보 bbox와, 자세 판정 근거(관절·각도 등)를 따로 모은다.
        fallen: list[Box] = []
        evidence: list[PoseEvidence] = []
        # 1단계 detector가 아무 객체도 찾지 못했다면 판정할 대상도 없다.
        if detection_result.boxes is None:
            return fallen, evidence

        # rescue 모델의 class 0은 mannequin, class 1은 helping_person이다.
        # 여기서는 자세를 판정해야 하는 mannequin(class 0)만 선택한다.
        selected = detection_result.boxes[detection_result.boxes.cls == 0]
        frame_height, frame_width = frame.shape[:2]
        # bbox가 화면에서 차지하는 비율을 계산하기 위한 전체 프레임 면적이다.
        frame_area = float(frame_height * frame_width)
        for xyxy, box_conf in zip(
            selected.xyxy.cpu().numpy(), selected.conf.cpu().numpy()
        ):
            # GPU tensor를 원본 영상 기준 픽셀 좌표와 Python 실수로 변환한다.
            x1, y1, x2, y2 = (float(value) for value in xyxy)
            # 잘못된 0 크기 bbox가 나와도 0으로 나누지 않도록 최소 1픽셀로 둔다.
            width = max(x2 - x1, 1.0)
            height = max(y2 - y1, 1.0)
            box = Box(x1, y1, x2, y2, confidence=float(box_conf))
            box_area = width * height
            # 화면에서 너무 작은 mannequin은 Pose 관절 신뢰도가 낮으므로 버린다.
            if box_area / max(frame_area, 1.0) < self.pose_min_box_area:
                continue
            # 사람용 Pose가 목각인형에서 실패할 수 있으므로, 설정이 켜져 있고
            # detector bbox의 가로/세로 비율이 임계값 이상이면 낙상 예비 후보로
            # 기록한다. 이것은 아직 시간 누적을 거친 최종 응급 확정이 아니다.
            bbox_fallen = (
                self.mannequin_bbox_fallback
                and is_fallen_bbox_candidate(
                    box, self.mannequin_fallen_aspect_threshold
                )
            )

            # 뒤에서 전용 HOG+SVM 결과가 나오면 이 값을 덮어쓴다. Pose가 없거나
            # 품질 필터를 통과하지 못할 때 keep_bbox_fallback()이 사용할 값이다.
            fallback_fallen = bbox_fallen

            def keep_bbox_fallback() -> None:
                """Pose 실패 시 SVM 또는 bbox 판정이 참인 후보를 보존한다."""
                # fallback까지 정상으로 판단했다면 이 mannequin은 버린다.
                if not fallback_fallen:
                    return
                # 검출 bbox는 낙상 후보로 유지하되 실제 관절은 없으므로 17개
                # keypoint를 모두 confidence=0으로 채워 디버그 렌더러가 안전하게
                # 같은 PoseEvidence 형식을 처리할 수 있도록 한다.
                fallen.append(box)
                evidence.append(
                    PoseEvidence(
                        box=box,
                        keypoints=np.zeros((17, 3), dtype=float),
                        posture="FALLEN",
                        aspect_ratio=box.aspect_ratio,
                        torso_angle_deg=-1.0,
                        visible_keypoints=0,
                    )
                )

            # detector bbox만 자르면 팔·다리 끝이 잘릴 수 있으므로 네 방향으로
            # bbox 크기의 35%만큼 여유를 준다. 좌표는 프레임 경계를 넘지 않게
            # clamp한다.
            pad_x, pad_y = width * 0.35, height * 0.35
            left = max(0, int(x1 - pad_x))
            top = max(0, int(y1 - pad_y))
            right = min(frame_width, int(x2 + pad_x))
            bottom = min(frame_height, int(y2 + pad_y))
            # clamp 후에도 폭이나 높이가 없는 비정상 crop이면 Pose를 실행하지
            # 않고 앞에서 계산한 fallback 결과만 사용한다.
            if right <= left or bottom <= top:
                keep_bbox_fallback()
                continue
            crop = frame[top:bottom, left:right]
            # 목각인형 전용 HOG+SVM이 연결돼 있으면 crop의 FALLEN 여부를 먼저
            # 판정한다. 모델이 없을 때의 None은 "정상"이 아니라 "판정 없음"이다.
            classifier_fallen = (
                self.posture_classifier.predict_fallen(crop)
                if self.posture_classifier is not None else None
            )
            # SVM 결과가 있으면 bbox 비율보다 우선한다. 따라서 SVM의 정상 판정은
            # 단순히 가로로 긴 bbox가 낙상으로 남는 것도 막을 수 있다.
            fallback_fallen = (
                classifier_fallen
                if classifier_fallen is not None else bbox_fallen
            )
            # Pose는 전체 프레임이 아니라 확장 crop에만 실행한다. 작은 mannequin을
            # 모델 입력에서 크게 보이게 하고 주변 사람의 관절과 섞이는 것을 줄인다.
            pose_result = self.pose_model.predict(
                crop, conf=self.rescue_conf, **self.options
            )[0]
            # bbox, keypoint 또는 관절 confidence 중 하나라도 없으면 자세 각도를
            # 계산할 수 없으므로 SVM/bbox fallback으로 종료한다.
            if (
                pose_result.boxes is None
                or len(pose_result.boxes) == 0
                or pose_result.keypoints is None
                or pose_result.keypoints.conf is None
            ):
                keep_bbox_fallback()
                continue
            # 한 crop에서 여러 Pose가 나올 수 있으므로 bbox confidence가 가장 높은
            # 한 사람만 현재 mannequin의 관절로 선택한다.
            pose_index = int(
                np.argmax(pose_result.boxes.conf.cpu().numpy())
            )
            xy = pose_result.keypoints.xy[pose_index].cpu().numpy()
            confidence = (
                pose_result.keypoints.conf[pose_index].cpu().numpy()
            )
            # Pose 좌표는 crop의 왼쪽 위가 (0, 0)인 지역 좌표다. detector bbox와
            # 디버그 영상에 함께 쓰기 위해 원본 프레임 좌표로 되돌린다.
            xy[:, 0] += left
            xy[:, 1] += top
            # 관절별 confidence가 기준 이상인 점만 보이는 관절로 센다.
            # 몸통 관절은 양쪽 어깨와 양쪽 엉덩이 네 점이다.
            visible = confidence >= self.pose_keypoint_conf
            torso_visible = int(visible[list(TORSO_INDEXES)].sum())
            # 전체 관절 또는 몸통 관절이 부족하면 Pose 자세를 신뢰하지 않고
            # SVM/bbox fallback을 사용한다.
            if (
                int(visible.sum()) < self.pose_min_keypoints
                or torso_visible < self.pose_min_torso_keypoints
            ):
                keep_bbox_fallback()
                continue
            # 각 관절을 [x, y, confidence] 한 행으로 묶어 자세 판정기와
            # 디버그 렌더러가 공통으로 사용할 수 있게 한다.
            keypoints = np.column_stack((xy, confidence))
            # Pose bbox가 아닌 1단계 mannequin bbox로 종횡비를 계산한다.
            posture, metrics = classify_posture(
                keypoints, xyxy, keypoint_conf=self.pose_keypoint_conf
            )
            # 사람용 Pose가 목각인형 관절을 세로로 잘못 찍더라도 detector
            # bbox가 명확히 가로형이면 안전 우선으로 낙상 후보를 유지한다.
            if classifier_fallen is not None:
                # 전용 crop 분류기가 연결되면 사람용 Pose의 잘못된 FALLEN
                # 결과로 정상 판정을 뒤집지 않는다. Pose는 골격 시각화 근거다.
                posture = "FALLEN" if classifier_fallen else "STANDING"
            elif bbox_fallen:
                # 전용 SVM이 없을 때만 가로형 detector bbox가 Pose 결과를
                # FALLEN으로 보완한다.
                posture = "FALLEN"
            # 최종 자세가 FALLEN인 bbox만 시간 누적 판정의 입력으로 보낸다.
            if posture == "FALLEN":
                fallen.append(box)
            # 낙상 여부와 관계없이 품질을 통과한 Pose 근거는 모두 남긴다.
            # status JSON과 디버그 영상에서 STANDING/SITTING/UNKNOWN도 확인 가능하다.
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
        # 모든 mannequin을 처리한 뒤 낙상 후보와 자세 근거를 함께 반환한다.
        return fallen, evidence

    def _detect_rescue_targets(self, frame):
        """선택한 backend로 환자·조력자 후보와 자세 근거를 검출한다."""
        # backend는 카메라 종류가 아니라 "낙상 후보를 만드는 방법"이다.
        # mannequin_detect: 전용 detector -> mannequin crop 자세 판정
        # person_pose: 전체 영상 Pose -> 관절 기반 실제 사람 자세 판정
        if self.detection_backend == "mannequin_detect":
            rescue_result = self.rescue_model.predict(
                frame, conf=self.rescue_conf, **self.options
            )[0]
            fallen, evidence = self._mannequin_pose_detections(
                frame, rescue_result
            )
            helpers = _boxes(rescue_result, 1)
            return rescue_result, [], fallen, helpers, evidence

        rescue_result = self.pose_model.predict(
            frame, conf=self.person_conf, **self.options
        )[0]
        people, fallen, evidence = self._pose_detections(
            rescue_result, frame.shape
        )
        return rescue_result, people, fallen, [], evidence

    def _detect_people(self, frame, pose_people, fallen, helpers):
        """사람 모델 또는 Pose 결과로 인원수·혼잡도·조력자를 계산한다."""
        # 구조 대상 검출과 주변 사람 계산은 목적이 다르다.
        # 골목에서는 관절이 잘 안 보이는 먼 사람도 세기 위해 별도 COCO person
        # 모델을 쓰고, ROI 밖 사람과 fallen bbox에 중복된 사람을 제외한다.
        person_result = None
        person_count = 0
        crowd_level = None
        time_multiplier = None
        crowd_traversable = True
        height, width = frame.shape[:2]
        frame_size = (width, height)

        if self.person_model is not None:
            person_result = self.person_model.predict(
                frame,
                conf=self.person_conf,
                classes=[self.person_class_id],
                **self.options,
            )[0]
            people = filter_nonfallen_people(
                _boxes(person_result, self.person_class_id),
                fallen,
                frame_size,
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
        elif (
            self.detection_backend == "person_pose"
            and self.detect_people_as_helpers
        ):
            people = filter_nonfallen_people(
                pose_people,
                fallen,
                frame_size,
                self.crowd_roi,
                self.overlap_threshold,
            )
            person_count = len(people)
            helpers = people

        helpers = filter_helpers_near_fallen(
            helpers,
            fallen,
            frame_size,
            self.helper_max_distance_ratio,
        )
        return (
            person_result,
            helpers,
            person_count,
            crowd_level,
            time_multiplier,
            crowd_traversable,
        )

    def predict(self, frame) -> InferenceOutput:
        """한 프레임에서 자세·구조·조력자·혼잡도를 추론하고 결과를 묶어 반환한다."""
        # 1단계는 "누가 환자인가", 2단계는 "주변에 누가 있고 길이 막혔는가"를
        # 계산한다. 시간 누적 확정과 ROS 발행은 상위 VisionDetector의 책임이다.
        started = perf_counter()
        (
            rescue_result,
            pose_people,
            fallen,
            helpers,
            pose_evidence,
        ) = self._detect_rescue_targets(frame)
        (
            person_result,
            helpers,
            person_count,
            crowd_level,
            time_multiplier,
            crowd_traversable,
        ) = self._detect_people(
            frame, pose_people, fallen, helpers
        )

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
        """추론 결과의 bbox·골격·ROI·상태 정보를 입력 영상 위에 그린다."""
        if output.detection_backend == "mannequin_detect":
            # 1차 검출에서 mannequin은 bbox만 표시하고 helping_person만
            # 이름을 표시한다. confidence는 둘 다 숨기며, Pose를 통과한
            # 자세 라벨과 골격은 아래에서 별도로 덮어 그린다.
            image = output.rescue_result.plot(labels=False, conf=False)
            rescue_boxes = getattr(output.rescue_result, "boxes", None)
            if rescue_boxes is not None:
                coordinates = rescue_boxes.xyxy.int().cpu().tolist()
                class_ids = rescue_boxes.cls.int().cpu().tolist()
                for (x1, y1, _x2, _y2), class_id in zip(
                    coordinates, class_ids
                ):
                    if class_id != 1:
                        continue
                    cv2.putText(
                        image,
                        "helping_person",
                        (x1, max(y1 - 8, 22)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
        else:
            image = output.rescue_result.orig_img.copy()
        # 두 backend 모두 Pose 자세·골격을 같은 방식으로 덧그린다.
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
                        color, 2, cv2.LINE_AA,
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
