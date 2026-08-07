"""YOLO 모델 로딩, 구조·혼잡 추론과 디버그 영상 렌더링."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import cv2

from .detection_logic import (
    Box,
    classify_crowd,
    crowd_time_multiplier,
    filter_nonfallen_people,
)


RESCUE_CLASS_NAMES = ["fallen_person", "helper_rc_car"]
DISPLAY_NAMES = {0: "fallen_person", 1: "helper"}


def _model_path(value: str, description: str) -> Path:
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
        path = Path(os.path.expandvars(value)).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f"{description} not found: {path}")
    return path


def _names(model) -> list[str]:
    names = model.names
    if isinstance(names, dict):
        return [str(names[key]) for key in sorted(names, key=int)]
    if isinstance(names, (list, tuple)):
        return [str(name) for name in names]
    raise RuntimeError(f"Unsupported model names: {names}")


def _boxes(result, class_id: int) -> list[Box]:
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


class InferencePipeline:
    """카메라 모드에 맞는 YOLO 모델과 후처리를 캡슐화한다."""

    def __init__(
        self,
        *,
        rescue_weights: str,
        person_weights: str,
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
    ) -> None:
        from ultralytics import YOLO

        self.enable_crowd = enable_crowd
        self.detect_people_as_helpers = detect_people_as_helpers
        self.rescue_conf = rescue_conf
        self.person_conf = person_conf
        self.crowd_roi = crowd_roi
        # 하위 호환을 위해 파라미터는 받지만 혼잡 등급은 이제 사람 수 1/2/3을
        # 직접 사용한다.
        self.crowded_threshold = crowded_threshold
        self.overlap_threshold = overlap_threshold
        self.options = {"iou": iou, "imgsz": imgsz, "verbose": False}
        if device:
            self.options["device"] = device

        self.rescue_model = YOLO(
            str(_model_path(rescue_weights, "rescue weights"))
        )
        rescue_names = _names(self.rescue_model)
        if rescue_names != RESCUE_CLASS_NAMES:
            raise RuntimeError(
                f"Rescue classes must be {RESCUE_CLASS_NAMES}, "
                f"got {rescue_names}"
            )

        self.person_model = None
        self.person_class_id = -1
        if enable_crowd or detect_people_as_helpers:
            self.person_model = YOLO(
                str(_model_path(person_weights, "COCO person weights"))
            )
            person_names = _names(self.person_model)
            if "person" not in person_names:
                raise RuntimeError(
                    "The person model does not contain COCO person"
                )
            self.person_class_id = person_names.index("person")

    def predict(self, frame) -> InferenceOutput:
        started = perf_counter()
        rescue_result = self.rescue_model.predict(
            frame, conf=self.rescue_conf, **self.options
        )[0]
        fallen = _boxes(rescue_result, 0)
        helpers = _boxes(rescue_result, 1)
        person_result = None
        person_count = 0
        crowd_level = None
        time_multiplier = None
        crowd_traversable = True

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
        )

    def render_debug(self, output: InferenceOutput, camera_id: str):
        output.rescue_result.names = DISPLAY_NAMES
        image = output.rescue_result.plot()
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
            text = f"{camera_id} | rescue detection"
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
