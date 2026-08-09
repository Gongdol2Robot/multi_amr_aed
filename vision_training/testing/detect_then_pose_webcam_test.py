#!/usr/bin/env python3
"""객체를 먼저 검출하고, 검출 bbox 내부에만 Pose를 적용해 자세를 판정한다.

COCO 사람 검출 모델과 커스텀 목각인형 검출 모델을 모두 1단계 검출기로
사용할 수 있다. Pose 결과의 crop 좌표는 원본 프레임 좌표로 복원한다.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
from time import perf_counter, sleep

import cv2
import numpy as np
import torch
from ultralytics import YOLO

from posture_utils import PostureHistory, classify_posture


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT.parent
WORKSPACE_ROOT = PACKAGE_ROOT.parent
DEFAULT_POSE = PACKAGE_ROOT / "src" / "aed_vision" / "models" / "yolo11n-pose.pt"
DEFAULT_PERSON_DETECTOR = WORKSPACE_ROOT / "yolo_training" / "models" / "yolo11n.pt"
DEFAULT_MANNEQUIN_DETECTOR = (
    PACKAGE_ROOT / "src" / "aed_vision" / "models" / "rescue2_yolo11n.pt"
)
COLORS = {
    "STANDING": (0, 200, 0),
    "SITTING": (0, 200, 255),
    "FALLEN": (0, 0, 255),
    "UNKNOWN": (160, 160, 160),
}

# YOLO Pose가 사용하는 COCO 17개 관절의 연결 관계.
# 얼굴은 눈·귀까지, 몸통은 어깨-엉덩이, 팔과 다리는 각 관절 순서로 잇는다.
COCO_SKELETON = (
    (0, 1), (0, 2), (1, 3), (2, 4),
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", default="33",
        help="카메라 번호/장치 또는 영상 파일 (기본값: 0). libcamera도 사용 가능",
    )
    parser.add_argument(
        "--camera", default="0",
        help="--source libcamera일 때 cam 카메라 번호/식별자",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument(
        "--target", choices=("person", "mannequin"), default="mannequin",
        help="person=COCO 사람 검출, mannequin=커스텀 목각인형 검출",
    )
    parser.add_argument(
        "--detector-weights", type=Path, default=None,
        help="미지정 시 --target에 맞는 기본 검출 모델 사용",
    )
    parser.add_argument("--pose-weights", type=Path, default=DEFAULT_POSE)
    parser.add_argument(
        "--device", default="auto",
        help="auto, cpu 또는 CUDA 번호(예: 0). 기본값 auto",
    )
    parser.add_argument(
        "--det-conf", type=float, default=0.20,
        help="1단계 목각인형 검출 confidence 임계값 (기본값: 0.20)",
    )
    parser.add_argument("--pose-conf", type=float, default=0.25)
    parser.add_argument("--keypoint-conf", type=float, default=0.3)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--crop-padding", type=float, default=0.15,
        help="검출 bbox 각 방향에 추가할 비율",
    )
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def resolve_source(value: str) -> int | str:
    if value == "libcamera":
        return value
    if value.isdigit():
        return int(value)
    path = Path(value).expanduser().resolve()
    if not path.exists():
        raise SystemExit(f"입력이 없습니다: {path}")
    return str(path)


class LibcameraCapture:
    """libcamera의 cam 출력을 relay 없이 OpenCV 프레임으로 변환한다."""

    def __init__(self, camera: str, width: int, height: int):
        self.width = width
        self.height = height
        self.frame_bytes = width * height * 4
        command = [
            "cam", "-c", camera,
            "--stream",
            f"role=viewfinder,width={width},height={height},pixelformat=XRGB8888",
            "--capture", "--file=-",
        ]
        try:
            self.process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=None, bufsize=0,
            )
        except FileNotFoundError as exc:
            raise SystemExit("libcamera cam 명령을 찾을 수 없습니다.") from exc

    def isOpened(self) -> bool:
        return self.process.poll() is None and self.process.stdout is not None

    def read(self):
        if self.process.stdout is None:
            return False, None
        data = bytearray()
        while len(data) < self.frame_bytes:
            chunk = self.process.stdout.read(self.frame_bytes - len(data))
            if not chunk:
                return False, None
            data.extend(chunk)
        xrgb = np.frombuffer(data, dtype=np.uint8).reshape(
            self.height, self.width, 4
        )
        # XRGB8888은 little-endian 메모리에서 B, G, R, X 순서다.
        return True, xrgb[:, :, :3].copy()

    def release(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def resolve_weights(args: argparse.Namespace) -> tuple[Path, Path]:
    detector = args.detector_weights
    if detector is None:
        detector = (
            DEFAULT_PERSON_DETECTOR
            if args.target == "person"
            else DEFAULT_MANNEQUIN_DETECTOR
        )
    detector = detector.expanduser().resolve()
    pose = args.pose_weights.expanduser().resolve()
    for label, path in (("검출", detector), ("Pose", pose)):
        if not path.is_file():
            raise SystemExit(f"{label} 가중치가 없습니다: {path}")
    return detector, pose


def padded_crop(frame: np.ndarray, box: np.ndarray, padding: float):
    """bbox에 여백을 더한 crop과 원본 기준 좌상단 좌표를 반환한다."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = (float(value) for value in box)
    pad_x = (x2 - x1) * padding
    pad_y = (y2 - y1) * padding
    left = max(0, int(x1 - pad_x))
    top = max(0, int(y1 - pad_y))
    right = min(width, int(x2 + pad_x))
    bottom = min(height, int(y2 + pad_y))
    if right <= left or bottom <= top:
        return None, (left, top)
    return frame[top:bottom, left:right], (left, top)


def best_pose(result):
    """한 crop에서 confidence가 가장 높은 Pose 한 명을 선택한다."""
    if (
        result.boxes is None or len(result.boxes) == 0
        or result.keypoints is None or result.keypoints.conf is None
    ):
        return None
    confidences = result.boxes.conf.cpu().numpy()
    index = int(np.argmax(confidences))
    box = result.boxes.xyxy[index].cpu().numpy()
    points = result.keypoints.xy[index].cpu().numpy()
    scores = result.keypoints.conf[index].cpu().numpy()
    return box, points, scores, float(confidences[index])


def main() -> int:
    args = parse_args()
    if args.crop_padding < 0:
        raise SystemExit("--crop-padding은 0 이상이어야 합니다.")
    if args.camera_width <= 0 or args.camera_height <= 0:
        raise SystemExit("카메라 너비와 높이는 1 이상이어야 합니다.")
    detector_weights, pose_weights = resolve_weights(args)
    device = args.device
    if device == "auto":
        device = "0" if torch.cuda.is_available() else "cpu"
    detector = YOLO(str(detector_weights))
    pose_model = YOLO(str(pose_weights))
    if pose_model.task != "pose":
        raise SystemExit(f"Pose 모델이 아닙니다: task={pose_model.task}")

    # 사람 모드는 person(class 0), 목각인형 모드는 mannequin(class 0)과
    # RC카(class 1)를 함께 검출한다. Pose는 class 0에만 적용한다.
    pose_target_class = 0
    detector_classes = [0] if args.target == "person" else [0, 1]
    source = resolve_source(args.source)
    if source == "libcamera":
        capture = LibcameraCapture(
            args.camera, args.camera_width, args.camera_height
        )
    else:
        capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"입력을 열 수 없습니다: {args.source}")

    # 누운 자세는 연속된 몇 프레임만 확인되면 빠르게 고정한다.
    history = PostureHistory(window=12, fallen_hits=4)
    previous: dict[int, str] = {}
    frame_index = 0
    window = f"Detect then Pose | {args.target}"
    if not args.no_show:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"1단계 검출 모델: {detector_weights}")
    print(f"2단계 Pose 모델: {pose_weights}")
    print(f"추론 장치: {device}")
    if source == "libcamera":
        print(
            f"입력: libcamera camera={args.camera} "
            f"{args.camera_width}x{args.camera_height} (relay 미사용)"
        )
    print(
        f"검출 대상: {args.target} classes={detector_classes} "
        f"(Pose class={pose_target_class}) | q/ESC: 종료"
    )

    # USB 웹캠은 장치를 연 직후 몇 번의 read가 실패할 수 있다.
    first_frame = None
    for attempt in range(30):
        ok, candidate = capture.read()
        if ok and candidate is not None and candidate.size > 0:
            first_frame = candidate
            break
        if attempt == 0:
            print(f"입력 {args.source} 첫 프레임 대기 중...")
        sleep(0.1)
    if first_frame is None:
        capture.release()
        raise SystemExit(
            f"입력 {args.source}을 열었지만 프레임을 읽지 못했습니다. "
            "카메라 선택과 다른 프로그램의 카메라 사용 여부를 확인하세요."
        )

    try:
        while True:
            if first_frame is not None:
                frame = first_frame
                first_frame = None
            else:
                ok, frame = capture.read()
                if not ok or frame is None:
                    print(
                        f"frame={frame_index}: 입력 프레임 읽기 실패, "
                        "카메라 연결이 끊겼습니다."
                    )
                    break
            started = perf_counter()
            detection = detector.track(
                frame, persist=True, tracker="bytetrack.yaml",
                classes=detector_classes, conf=args.det_conf,
                imgsz=args.imgsz, device=device, verbose=False,
            )[0]
            canvas = frame.copy()
            detected = 0
            posed = 0
            rc_cars = 0

            if detection.boxes is not None and len(detection.boxes) > 0:
                boxes = detection.boxes.xyxy.cpu().numpy()
                if detection.boxes.id is None:
                    ids = list(range(len(boxes)))
                else:
                    ids = detection.boxes.id.int().cpu().tolist()
                det_scores = detection.boxes.conf.cpu().numpy()
                det_classes = detection.boxes.cls.int().cpu().tolist()

                for det_box, track_id, det_score, class_id in zip(
                    boxes, ids, det_scores, det_classes
                ):
                    detected += 1
                    if args.target == "mannequin" and class_id == 1:
                        rc_cars += 1
                        rc_color = (255, 0, 255)
                        cv2.rectangle(
                            canvas, tuple(det_box[:2].astype(int)),
                            tuple(det_box[2:].astype(int)), rc_color, 2,
                        )
                        cv2.putText(
                            canvas,
                            f"ID {track_id} helper_rc_car det={det_score:.2f}",
                            (int(det_box[0]), max(int(det_box[1]) - 8, 24)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, rc_color, 2,
                        )
                        continue
                    if class_id != pose_target_class:
                        continue
                    crop, (offset_x, offset_y) = padded_crop(
                        frame, det_box, args.crop_padding
                    )
                    if crop is None or crop.size == 0:
                        continue
                    pose_result = pose_model.predict(
                        crop, conf=args.pose_conf, imgsz=args.imgsz,
                        device=device, verbose=False,
                    )[0]
                    selected = best_pose(pose_result)
                    if selected is None:
                        cv2.rectangle(
                            canvas, tuple(det_box[:2].astype(int)),
                            tuple(det_box[2:].astype(int)), (255, 120, 0), 2,
                        )
                        cv2.putText(
                            canvas, f"ID {track_id} detected / no pose",
                            (int(det_box[0]), max(int(det_box[1]) - 8, 24)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 120, 0), 2,
                        )
                        continue

                    pose_box, xy, scores, pose_score = selected
                    pose_box[[0, 2]] += offset_x
                    pose_box[[1, 3]] += offset_y
                    xy[:, 0] += offset_x
                    xy[:, 1] += offset_y
                    visible = scores >= args.keypoint_conf
                    if int(visible.sum()) < 8 or int(visible[[5, 6, 11, 12]].sum()) < 3:
                        continue

                    posed += 1
                    keypoints = np.column_stack((xy, scores))
                    raw, metrics = classify_posture(
                        # 사람용 Pose bbox는 목각인형에서 크게 흔들릴 수 있다.
                        # 자세의 종횡비는 1단계 mannequin 검출 bbox를 사용한다.
                        keypoints, det_box, keypoint_conf=args.keypoint_conf
                    )
                    posture = history.update(track_id, raw, frame_index)
                    if posture != previous.get(track_id):
                        print(
                            f"frame={frame_index} ID={track_id} "
                            f"{previous.get(track_id, 'START')}->{posture} "
                            f"raw={raw} det={det_score:.2f} pose={pose_score:.2f} "
                            f"aspect={metrics['aspect_ratio']:.2f} "
                            f"torso={metrics['torso_angle']:.1f}"
                        )
                        previous[track_id] = posture

                    color = COLORS[posture]
                    cv2.rectangle(
                        canvas, tuple(det_box[:2].astype(int)),
                        tuple(det_box[2:].astype(int)), color, 2,
                    )
                    for start, end in COCO_SKELETON:
                        if visible[start] and visible[end]:
                            cv2.line(
                                canvas,
                                tuple(xy[start].astype(int)),
                                tuple(xy[end].astype(int)),
                                color, 2, cv2.LINE_AA,
                            )
                    for x, y, shown in zip(xy[:, 0], xy[:, 1], visible):
                        if shown:
                            cv2.circle(
                                canvas, (int(x), int(y)), 4, color, -1,
                                cv2.LINE_AA,
                            )
                    label = (
                        f"ID {track_id} {posture} raw={raw} "
                        f"det={det_score:.2f} pose={pose_score:.2f}"
                    )
                    cv2.putText(
                        canvas, label,
                        (int(det_box[0]), max(int(det_box[1]) - 8, 24)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                    )

            history.discard_stale(frame_index, max_age=60)
            elapsed = (perf_counter() - started) * 1000
            cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 38), (0, 0, 0), -1)
            cv2.putText(
                canvas,
                f"detected={detected} rc_car={rc_cars} pose={posed} | "
                f"{elapsed:.1f} ms | frame={frame_index}",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
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
