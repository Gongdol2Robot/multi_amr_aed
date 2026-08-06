#!/usr/bin/env python3
"""YOLO11 Pose 관절 검출과 자세 판정을 이미지·영상·웹캠에서 테스트한다."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import perf_counter

from pose_posture import classify_posture


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
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--kpt-conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
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


def annotate_postures(result, canvas, keypoint_conf: float) -> list[str]:
    import cv2

    labels = []
    if result.boxes is None or result.keypoints is None:
        return labels
    boxes = result.boxes.xyxy.cpu().tolist()
    keypoints = result.keypoints.data.cpu().tolist()
    for box, person_keypoints in zip(boxes, keypoints):
        posture, metrics = classify_posture(
            person_keypoints, box, keypoint_conf
        )
        labels.append(posture)
        x1, y1 = int(box[0]), max(int(box[1]) - 8, 58)
        text = (
            f"{posture} | ratio={metrics['aspect_ratio']:.2f} "
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
            0.65, COLORS[posture], 2, cv2.LINE_AA,
        )
    return labels


def main() -> int:
    args = parse_args()
    weights = args.weights.expanduser().resolve()
    if not weights.is_file():
        raise SystemExit(f"Pose 모델이 없습니다: {weights}")
    if not 0.0 <= args.conf <= 1.0 or not 0.0 <= args.kpt_conf <= 1.0:
        raise SystemExit("conf와 kpt-conf는 0 이상 1 이하여야 합니다.")
    source, is_image = resolve_source(args.source)

    import cv2
    from ultralytics import YOLO

    model = YOLO(str(weights))
    if model.task != "pose":
        raise SystemExit(f"Pose 모델이 아닙니다: task={model.task}")
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"입력을 열 수 없습니다: {source}")
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    window = f"YOLO11 Pose | {args.stage}"
    if not args.no_show:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            started = perf_counter()
            result = model.predict(
                frame, conf=args.conf, imgsz=args.imgsz,
                device=args.device, verbose=False,
            )[0]
            elapsed_ms = (perf_counter() - started) * 1000.0
            canvas = result.plot()
            labels = (
                annotate_postures(result, canvas, args.kpt_conf)
                if args.stage == "posture"
                else []
            )
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
