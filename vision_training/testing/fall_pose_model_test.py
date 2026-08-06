#!/usr/bin/env python3
"""학습한 12관절 Fall/Non-Fall YOLO11 Pose 모델을 테스트한다."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import perf_counter


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUNS = ROOT / "runs" / "pose"
DEFAULT_OUTPUT = DEFAULT_RUNS / "fall_pose_test"
EXPECTED_CLASSES = {0: "Fall", 1: "Non-Fall"}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def latest_weights() -> Path:
    preferred = DEFAULT_RUNS / "caterpillar_fall_pose" / "weights" / "best.pt"
    if preferred.is_file():
        return preferred
    smoke = DEFAULT_RUNS / "caterpillar_pose_smoke" / "weights" / "best.pt"
    if smoke.is_file():
        return smoke
    candidates = list(DEFAULT_RUNS.glob("*/weights/best.pt"))
    if not candidates:
        raise SystemExit(f"Pose best.pt가 없습니다: {DEFAULT_RUNS}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=None)
    parser.add_argument("--source", default="2", help="카메라 번호 또는 이미지/영상")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.5)
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
    return str(path), path.suffix.lower() in IMAGE_SUFFIXES


def normalize_names(names) -> dict[int, str]:
    if isinstance(names, dict):
        return {int(index): str(name) for index, name in names.items()}
    return {index: str(name) for index, name in enumerate(names)}


def main() -> int:
    args = parse_args()
    weights = (
        args.weights.expanduser().resolve()
        if args.weights is not None
        else latest_weights()
    )
    if not weights.is_file():
        raise SystemExit(f"가중치가 없습니다: {weights}")
    if not 0.0 <= args.conf <= 1.0 or not 0.0 <= args.iou <= 1.0:
        raise SystemExit("conf와 iou는 0 이상 1 이하여야 합니다.")
    source, is_image = resolve_source(args.source)

    import cv2
    from ultralytics import YOLO

    model = YOLO(str(weights))
    names = normalize_names(model.names)
    if model.task != "pose" or names != EXPECTED_CLASSES:
        raise SystemExit(
            f"Fall/Non-Fall Pose 모델이 아닙니다: task={model.task}, names={names}"
        )
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"입력을 열 수 없습니다: {source}")
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    window = "YOLO11 Fall Pose | Fall vs Non-Fall"
    if not args.no_show:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"모델: {weights}")
    print("Q/ESC: 종료 | S: 현재 화면 저장")

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            started = perf_counter()
            result = model.predict(
                frame,
                conf=args.conf,
                iou=args.iou,
                imgsz=args.imgsz,
                device=args.device,
                verbose=False,
            )[0]
            elapsed_ms = (perf_counter() - started) * 1000.0
            canvas = result.plot()
            counts = {name: 0 for name in EXPECTED_CLASSES.values()}
            if result.boxes is not None and result.boxes.cls is not None:
                for class_id in result.boxes.cls.int().cpu().tolist():
                    counts[names[class_id]] += 1
            summary = (
                f"Fall={counts['Fall']} | Non-Fall={counts['Non-Fall']} "
                f"| {elapsed_ms:.1f} ms"
            )
            cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 44), (0, 0, 0), -1)
            cv2.putText(
                canvas, summary, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2, cv2.LINE_AA,
            )

            if is_image:
                output = output_dir / f"{Path(str(source)).stem}_fall_pose.jpg"
                cv2.imwrite(str(output), canvas)
                print(f"{summary} | 저장: {output}")
                break
            if args.no_show:
                continue
            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                output = output_dir / "fall_pose_capture.jpg"
                cv2.imwrite(str(output), canvas)
                print(f"저장: {output}")
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
