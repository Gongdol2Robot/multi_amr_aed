#!/usr/bin/env python3
"""회전 없이 vs 박스를 세워서 — 누운 대상의 관절이 잡히는지 비교한다.

같은 프레임에 두 방법을 나란히 돌려 검출률과 자세 판정을 함께 본다.
판정 규칙은 pose_posture.classify_posture 를 그대로 쓴다.

사용:
  python3 rotated_pose_test.py                      # 웹캠, 화면 표시
  python3 rotated_pose_test.py --frames 100 --no-show   # 숫자만
  python3 rotated_pose_test.py --source shot.jpg    # 이미지 한 장

--detector-weights 를 주면 그 모델이 찾은 박스를 잘라 쓴다. 목각인형처럼
Pose 모델이 아예 못 보는 대상은 이 1단계가 있어야 위치를 알 수 있다.
"""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

import rotated_pose
from pose_posture import classify_posture


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POSE = ROOT / "models" / "yolo11n-pose.pt"
DEFAULT_DETECTOR = (
    ROOT.parent / "src" / "aed_vision" / "models" / "rescue_yolo11n.pt"
)
COLORS = {
    "STANDING": (0, 200, 0),
    "SITTING": (0, 200, 255),
    "FALLEN": (0, 0, 255),
    "UNKNOWN": (160, 160, 160),
}
SKELETON = (
    (5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16), (0, 1), (0, 2), (1, 3), (2, 4),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="2", help="카메라 번호 또는 이미지 경로")
    parser.add_argument("--pose-weights", type=Path, default=DEFAULT_POSE)
    parser.add_argument(
        "--detector-weights", type=Path, default=DEFAULT_DETECTOR,
        help="박스를 찾는 1단계 모델. none 이면 Pose 가 화면 전체를 본다",
    )
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--pose-imgsz", type=int, default=960)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def draw(image, keypoints, label, box=None):
    color = COLORS.get(label, (160, 160, 160))
    if box is not None:
        cv2.rectangle(
            image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])),
            (255, 120, 0), 2,
        )
    if keypoints is not None:
        for first, second in SKELETON:
            if keypoints[first][2] > 0.5 and keypoints[second][2] > 0.5:
                cv2.line(
                    image,
                    (int(keypoints[first][0]), int(keypoints[first][1])),
                    (int(keypoints[second][0]), int(keypoints[second][1])),
                    (0, 255, 0), 2,
                )
        for x, y, score in keypoints:
            if score > 0.5:
                cv2.circle(image, (int(x), int(y)), 3, (0, 0, 255), -1)
    cv2.putText(
        image, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
    )
    return image


def main() -> int:
    from ultralytics import YOLO

    args = parse_args()
    pose = YOLO(str(args.pose_weights))
    detector = None
    if str(args.detector_weights).lower() != "none":
        detector = YOLO(str(args.detector_weights))

    source = args.source
    single_image = not str(source).isdigit()
    if single_image:
        frames = [cv2.imread(str(source))]
        if frames[0] is None:
            print(f"이미지를 못 읽었습니다: {source}")
            return 1
    else:
        capture = cv2.VideoCapture(int(source))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        for _ in range(10):     # 자동 노출이 안정될 때까지 버린다
            capture.read()

    options = {"imgsz": args.pose_imgsz, "verbose": False}
    if args.device:
        options["device"] = args.device

    plain_hits = rotated_hits = 0
    plain_labels: Counter[str] = Counter()
    rotated_labels: Counter[str] = Counter()
    angles: Counter[int] = Counter()
    elapsed = []
    total = 0

    while total < (1 if single_image else args.frames):
        if single_image:
            frame = frames[0]
        else:
            ok, frame = capture.read()
            if not ok:
                break
        total += 1

        box = None
        if detector is not None:
            found = detector.predict(
                frame, conf=args.conf, imgsz=640, verbose=False,
                device=args.device,
            )[0]
            if found.boxes is not None and len(found.boxes):
                index = int(np.argmax(found.boxes.conf.cpu().numpy()))
                box = [float(v) for v in found.boxes.xyxy[index]]

        # 1) 회전 없이 화면 그대로
        started = perf_counter()
        plain = pose.predict(frame, conf=args.conf, **options)[0]
        plain_keypoints = None
        plain_label = "UNKNOWN"
        if plain.boxes is not None and len(plain.boxes):
            index = int(np.argmax(plain.boxes.conf.cpu().numpy()))
            points = plain.keypoints.xy[index].cpu().numpy().astype(float)
            scores = plain.keypoints.conf[index].cpu().numpy().astype(float)
            plain_keypoints = np.column_stack([points, scores])
            plain_label, _ = classify_posture(
                plain_keypoints, plain.boxes.xyxy[index].cpu().numpy()
            )
            plain_hits += 1
        plain_labels[plain_label] += 1

        # 2) 박스를 세워서, 좌표는 원본으로 되돌려서
        rotated_keypoints = None
        rotated_label = "UNKNOWN"
        if box is not None:
            result = rotated_pose.estimate(
                pose, frame, box, conf=args.conf,
                imgsz=args.pose_imgsz, device=args.device,
            )
            if result is not None:
                rotated_keypoints, _, angle = result
                angles[angle] += 1
                rotated_label, _ = classify_posture(rotated_keypoints, box)
                rotated_hits += 1
        rotated_labels[rotated_label] += 1
        elapsed.append((perf_counter() - started) * 1000.0)

        if not args.no_show:
            left = draw(frame.copy(), plain_keypoints, plain_label)
            right = draw(frame.copy(), rotated_keypoints, rotated_label, box)
            cv2.putText(left, "no rotation", (12, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(right, "rotated (mapped back)", (12, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.imshow("no rotation  vs  rotated   -  q to quit",
                       np.hstack([left, right]))
            if cv2.waitKey(0 if single_image else 1) & 0xFF == ord("q"):
                break

    if not single_image:
        capture.release()
    cv2.destroyAllWindows()

    print(f"\n프레임 {total}개")
    print(f"  회전 없이 : 검출 {plain_hits}/{total}  {dict(plain_labels)}")
    print(f"  세워서    : 검출 {rotated_hits}/{total}  {dict(rotated_labels)}")
    if angles:
        print(f"  쓰인 회전각: {dict(angles)}")
    if elapsed:
        print(f"  두 방법 합쳐 프레임당 {np.mean(elapsed):.0f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
