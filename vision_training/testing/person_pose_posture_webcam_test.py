#!/usr/bin/env python3
"""기본 YOLO11 Pose 관절로 실제 사람의 서기/앉기/누움을 판정한다."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
from ultralytics import YOLO

from pose_posture import PostureHistory, classify_posture


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = ROOT / "models" / "yolo11n-pose.pt"
COLORS = {
    "STANDING": (0, 200, 0),
    "SITTING": (0, 200, 255),
    "FALLEN": (0, 0, 255),
    "UNKNOWN": (160, 160, 160),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="2", help="카메라 번호/장치/영상")
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--device", default="0")
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--keypoint-conf", type=float, default=0.3)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def resolve_source(value: str) -> int | str:
    if value.isdigit():
        return int(value)
    path = Path(value).expanduser().resolve()
    if not path.exists():
        raise SystemExit(f"입력이 없습니다: {path}")
    return str(path)


def main() -> int:
    args = parse_args()
    weights = args.weights.expanduser().resolve()
    if not weights.is_file():
        raise SystemExit(f"Pose 가중치가 없습니다: {weights}")
    model = YOLO(str(weights))
    if model.task != "pose":
        raise SystemExit(f"Pose 모델이 아닙니다: task={model.task}")
    capture = cv2.VideoCapture(resolve_source(args.source))
    if not capture.isOpened():
        raise SystemExit(f"입력을 열 수 없습니다: {args.source}")

    history = PostureHistory(window=10, fallen_hits=6)
    previous: dict[int, str] = {}
    frame_index = 0
    window = "Base YOLO11 Pose | posture"
    if not args.no_show:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"Pose 모델: {weights}")
    print("STANDING/SITTING/FALLEN | q/ESC: 종료")

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            started = perf_counter()
            result = model.track(
                frame, persist=True, tracker="bytetrack.yaml",
                conf=args.conf, imgsz=args.imgsz, device=args.device,
                verbose=False,
            )[0]
            canvas = frame.copy()
            active = 0
            if (
                result.boxes is not None and result.boxes.id is not None
                and result.keypoints is not None
                and result.keypoints.conf is not None
            ):
                boxes = result.boxes.xyxy.cpu().numpy()
                ids = result.boxes.id.int().cpu().tolist()
                points = result.keypoints.xy.cpu().numpy()
                scores = result.keypoints.conf.cpu().numpy()
                for box, track_id, xy, confidence in zip(boxes, ids, points, scores):
                    visible = confidence >= args.keypoint_conf
                    if int(visible.sum()) < 8 or int(visible[[5, 6, 11, 12]].sum()) < 3:
                        continue
                    active += 1
                    keypoints = np.column_stack((xy, confidence))
                    raw, metrics = classify_posture(
                        keypoints, box, keypoint_conf=args.keypoint_conf
                    )
                    posture = history.update(track_id, raw, frame_index)
                    if posture != previous.get(track_id):
                        print(
                            f"frame={frame_index} ID={track_id} "
                            f"{previous.get(track_id, 'START')}->{posture} "
                            f"raw={raw} aspect={metrics['aspect_ratio']:.2f} "
                            f"torso={metrics['torso_angle']:.1f}"
                        )
                        previous[track_id] = posture
                    color = COLORS[posture]
                    cv2.rectangle(
                        canvas, tuple(box[:2].astype(int)),
                        tuple(box[2:].astype(int)), color, 2,
                    )
                    for x, y, shown in zip(xy[:, 0], xy[:, 1], visible):
                        if shown:
                            cv2.circle(canvas, (int(x), int(y)), 3, color, -1)
                    text = (
                        f"ID {track_id} {posture} raw={raw} "
                        f"aspect={metrics['aspect_ratio']:.2f} "
                        f"torso={metrics['torso_angle']:.0f}"
                    )
                    cv2.putText(
                        canvas, text, (int(box[0]), max(int(box[1]) - 8, 24)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2,
                    )
            history.discard_stale(frame_index, max_age=60)
            elapsed = (perf_counter() - started) * 1000
            cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 38), (0, 0, 0), -1)
            cv2.putText(
                canvas, f"people={active} | {elapsed:.1f} ms | frame={frame_index}",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (255, 255, 0), 2,
            )
            frame_index += 1
            if args.no_show:
                continue
            cv2.imshow(window, canvas)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
