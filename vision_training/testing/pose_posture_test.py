#!/usr/bin/env python3
"""YOLO11 Pose 관절 검출과 자세 판정을 이미지·영상·웹캠에서 테스트한다."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import perf_counter

from pose_posture import PostureHistory, classify_posture


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = ROOT / "models" / "yolo11n-pose.pt"
DEFAULT_OUTPUT = ROOT / "runs" / "pose_test"
COLORS = {
    "STANDING": (0, 200, 0),
    "SITTING": (0, 200, 255),
    "FALLEN": (0, 0, 255),
    "UNKNOWN": (160, 160, 160),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="2", help="카메라 번호 또는 이미지/영상")
    parser.add_argument(
        "--stage",
        choices=("keypoints", "posture"),
        default="posture",
    )
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument(
        "--detector-weights", type=Path, default=None,
        help="선택 사항: 사람 전용 1단계 검출 모델. 기본값은 Pose 모델 자체",
    )
    parser.add_argument("--conf", type=float, default=0.15)
    parser.add_argument("--kpt-conf", type=float, default=0.15)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--pose-imgsz", type=int, default=640)
    parser.add_argument("--roi-margin", type=float, default=0.15)
    parser.add_argument(
        "--single-stage", action="store_true",
        help="사람 ROI 재추론 없이 Pose 모델을 전체 화면에 한 번만 실행",
    )
    parser.add_argument("--device", default="0")
    parser.add_argument("--confirmation-window", type=int, default=10)
    parser.add_argument("--confirmation-hits", type=int, default=6)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def resolve_source(value: str) -> tuple[int | str, bool]:
    if value.isdigit():
        return int(value), False
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"입력 파일이 없습니다: {path}")
    image = path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    return str(path), image


def annotate_postures(
    result, canvas, keypoint_conf: float,
    history: PostureHistory, frame_index: int,
) -> list[str]:
    import cv2

    labels = []
    if result.boxes is None or result.keypoints is None:
        return labels
    boxes = result.boxes.xyxy.cpu().tolist()
    keypoints = result.keypoints.data.cpu().tolist()
    track_ids = (
        result.boxes.id.int().cpu().tolist()
        if result.boxes.id is not None
        else list(range(len(boxes)))
    )
    for box, person_keypoints, track_id in zip(boxes, keypoints, track_ids):
        posture, metrics = classify_posture(
            person_keypoints, box, keypoint_conf
        )
        confirmed = history.update(track_id, posture, frame_index)
        labels.append(confirmed)
        x1, y1 = int(box[0]), max(int(box[1]) - 8, 58)
        text = (
            f"ID {track_id} {confirmed} ({posture}) | "
            f"ratio={metrics['aspect_ratio']:.2f} "
            f"torso={metrics['torso_angle']:.0f}deg"
        )
        text_size, baseline = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2
        )
        cv2.rectangle(
            canvas,
            (x1, y1 - text_size[1] - baseline - 4),
            (x1 + text_size[0] + 6, y1 + baseline),
            (0, 0, 0),
            -1,
        )
        cv2.putText(
            canvas, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, COLORS[confirmed], 2, cv2.LINE_AA,
        )
    return labels


def expanded_roi(box, frame_shape, margin: float) -> tuple[int, int, int, int]:
    """사람 bbox에 여백을 더하고 프레임 범위로 제한한다."""
    height, width = frame_shape[:2]
    x1, y1, x2, y2 = (float(value) for value in box)
    pad_x = (x2 - x1) * margin
    pad_y = (y2 - y1) * margin
    return (
        max(0, int(x1 - pad_x)),
        max(0, int(y1 - pad_y)),
        min(width, int(x2 + pad_x)),
        min(height, int(y2 + pad_y)),
    )


def infer_pose_rois(
    frame, detector_result, pose_model, args,
    history: PostureHistory, frame_index: int,
) -> tuple[object, list[str]]:
    """검출된 사람을 확대 재추론하고 결과를 원본 프레임에 합성한다."""
    import cv2

    canvas = detector_result.plot(labels=False, conf=False)
    labels = []
    if detector_result.boxes is None:
        return canvas, labels
    boxes = detector_result.boxes.xyxy.cpu().tolist()
    track_ids = (
        detector_result.boxes.id.int().cpu().tolist()
        if detector_result.boxes.id is not None
        else list(range(len(boxes)))
    )
    for detector_box, track_id in zip(boxes, track_ids):
        left, top, right, bottom = expanded_roi(
            detector_box, frame.shape, args.roi_margin
        )
        if right <= left or bottom <= top:
            continue
        roi = frame[top:bottom, left:right]
        pose_result = pose_model.predict(
            roi, conf=args.conf, imgsz=args.pose_imgsz,
            device=args.device, verbose=False,
        )[0]
        canvas[top:bottom, left:right] = pose_result.plot()
        if (
            pose_result.boxes is None or len(pose_result.boxes) == 0
            or pose_result.keypoints is None
        ):
            confirmed, posture = history.update(
                track_id, "UNKNOWN", frame_index
            ), "UNKNOWN"
            metrics = {"aspect_ratio": 0.0, "torso_angle": -1.0}
        else:
            confidences = pose_result.boxes.conf.cpu().tolist()
            best = max(range(len(confidences)), key=confidences.__getitem__)
            pose_box = pose_result.boxes.xyxy[best].cpu().tolist()
            keypoints = pose_result.keypoints.data[best].cpu().tolist()
            posture, metrics = classify_posture(
                keypoints, pose_box, args.kpt_conf
            )
            confirmed = history.update(track_id, posture, frame_index)
        labels.append(confirmed)
        text = (
            f"ID {track_id} {confirmed} ({posture}) | "
            f"ratio={metrics['aspect_ratio']:.2f} "
            f"torso={metrics['torso_angle']:.0f}deg"
        )
        text_y = max(top - 8, 58)
        cv2.putText(
            canvas, text, (left, text_y), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, COLORS[confirmed], 2, cv2.LINE_AA,
        )
    return canvas, labels


def main() -> int:
    args = parse_args()
    weights = args.weights.expanduser().resolve()
    detector_weights = (
        args.detector_weights.expanduser().resolve()
        if args.detector_weights is not None else None
    )
    if not weights.is_file():
        raise SystemExit(f"Pose 모델이 없습니다: {weights}")
    if detector_weights is not None and not detector_weights.is_file():
        raise SystemExit(f"사람 검출 모델이 없습니다: {detector_weights}")
    if not 0.0 <= args.conf <= 1.0 or not 0.0 <= args.kpt_conf <= 1.0:
        raise SystemExit("conf와 kpt-conf는 0 이상 1 이하여야 합니다.")
    if not 1 <= args.confirmation_hits <= args.confirmation_window:
        raise SystemExit("confirmation-hits는 1 이상 confirmation-window 이하여야 합니다.")
    if args.roi_margin < 0.0:
        raise SystemExit("roi-margin은 0 이상이어야 합니다.")
    source, is_image = resolve_source(args.source)

    import cv2
    from ultralytics import YOLO

    pose_model = YOLO(str(weights))
    if pose_model.task != "pose":
        raise SystemExit(f"Pose 모델이 아닙니다: task={pose_model.task}")
    detector_model = (
        YOLO(str(detector_weights)) if detector_weights is not None else None
    )
    if detector_model is not None and detector_model.names.get(0) != "person":
        raise SystemExit(
            f"COCO 사람 검출 모델이 아닙니다: names[0]={detector_model.names.get(0)!r}"
        )
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"입력을 열 수 없습니다: {source}")
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    window = f"YOLO11 Pose | {args.stage}"
    if not args.no_show:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    history = PostureHistory(
        args.confirmation_window, args.confirmation_hits
    )
    frame_index = 0

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            started = perf_counter()
            # 목각인형은 COCO detect 모델이 사람으로 인식하지 못할 수 있다.
            # 별도 검출 모델을 명시하지 않으면 Pose 모델 자체의 bbox를 ROI로 쓴다.
            inference_model = detector_model or pose_model
            result = inference_model.track(
                frame, conf=args.conf, imgsz=args.imgsz, classes=[0],
                device=args.device, persist=True, verbose=False,
            )[0]
            elapsed_ms = (perf_counter() - started) * 1000.0
            if args.single_stage:
                canvas = result.plot()
                labels = (
                    annotate_postures(
                        result, canvas, args.kpt_conf, history, frame_index
                    )
                    if args.stage == "posture" else []
                )
            else:
                canvas, labels = infer_pose_rois(
                    frame, result, pose_model, args, history, frame_index
                )
                elapsed_ms = (perf_counter() - started) * 1000.0
            history.discard_stale(frame_index)
            frame_index += 1
            people = 0 if result.boxes is None else len(result.boxes)
            if args.stage == "posture":
                posture_summary = ", ".join(labels) if labels else "NO PERSON"
            else:
                posture_summary = "KEYPOINTS ONLY"
            status = (
                f"{posture_summary} | people={people} | {elapsed_ms:.1f} ms"
            )
            cv2.rectangle(
                canvas, (0, 0), (canvas.shape[1], 44), (0, 0, 0), -1
            )
            cv2.putText(
                canvas, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 255, 0), 2, cv2.LINE_AA,
            )
            if is_image:
                output = output_dir / f"{Path(str(source)).stem}_{args.stage}.jpg"
                cv2.imwrite(str(output), canvas)
                print(f"people={people}, postures={labels}, output={output}")
                break
            if not args.no_show:
                cv2.imshow(window, canvas)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
