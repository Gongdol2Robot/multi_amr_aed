#!/usr/bin/env python3
"""YOLO11n-Pose와 학습한 BiLSTM으로 웹캠 낙상 상태를 테스트한다."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from pathlib import Path
from time import monotonic, perf_counter

import numpy as np
import torch
from torch import nn


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CLASSIFIER = ROOT / "omnifall_yolo11n_pose_lstm_best.pt"
DEFAULT_POSE = ROOT / "models" / "yolo11n-pose.pt"
DEFAULT_OUTPUT = ROOT / "runs" / "pose_lstm_webcam"
COLORS = {
    "NORMAL": (0, 200, 0),
    "FALLING": (0, 180, 255),
    "FALLEN": (0, 0, 255),
    "COLLECTING": (180, 180, 180),
}


class FallPoseLSTM(nn.Module):
    """Colab 학습 노트북과 동일한 Pose 시계열 분류기."""

    def __init__(
        self, input_size: int = 56, hidden_size: int = 128, classes: int = 3
    ) -> None:
        super().__init__()
        self.norm = nn.LayerNorm(input_size * 2)
        self.lstm = nn.LSTM(
            input_size * 2,
            hidden_size,
            num_layers=2,
            batch_first=True,
            bidirectional=True,
            dropout=0.25,
        )
        self.head = nn.Sequential(
            nn.Linear(hidden_size * 2, 128),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(128, classes),
        )

    def forward(self, sequence: torch.Tensor) -> torch.Tensor:
        delta = torch.zeros_like(sequence)
        delta[:, 1:] = sequence[:, 1:] - sequence[:, :-1]
        features = self.norm(torch.cat([sequence, delta], dim=-1))
        _, (hidden, _) = self.lstm(features)
        summary = torch.cat([hidden[-2], hidden[-1]], dim=1)
        return self.head(summary)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="0", help="카메라 번호 또는 영상 파일")
    parser.add_argument("--classifier", type=Path, default=DEFAULT_CLASSIFIER)
    parser.add_argument("--pose-weights", type=Path, default=DEFAULT_POSE)
    parser.add_argument("--device", default="0", help="CUDA 번호 또는 cpu")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--sample-fps",
        type=float,
        default=8.0,
        help="ID별 Pose 시퀀스 수집 속도. 16개 기준 기본 약 2초",
    )
    parser.add_argument(
        "--smooth-window", type=int, default=5,
        help="최근 분류 확률 평균에 사용할 예측 개수",
    )
    parser.add_argument(
        "--fallen-hits", type=int, default=3,
        help="FALLEN 경보까지 필요한 연속 판정 횟수",
    )
    parser.add_argument(
        "--max-track-age", type=float, default=2.0,
        help="사라진 ID 기록을 제거할 때까지의 초",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def resolve_source(value: str) -> int | str:
    if value.isdigit():
        return int(value)
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"입력 영상이 없습니다: {path}")
    return str(path)


def resolve_devices(value: str) -> tuple[str, torch.device]:
    if value.lower() == "cpu":
        return "cpu", torch.device("cpu")
    if not torch.cuda.is_available():
        print("CUDA를 사용할 수 없어 CPU로 실행합니다.")
        return "cpu", torch.device("cpu")
    index = int(value)
    return value, torch.device(f"cuda:{index}")


def pose_feature(
    box: np.ndarray,
    keypoint_xy: np.ndarray,
    keypoint_conf: np.ndarray,
    frame_shape,
) -> np.ndarray:
    """노트북과 동일한 56차원 정규화 Pose 특징을 만든다."""
    height, width = frame_shape[:2]
    x1, y1, x2, y2 = (float(value) for value in box)
    box_width = max(x2 - x1, 1.0)
    box_height = max(y2 - y1, 1.0)
    normalized_xy = np.column_stack(
        (
            (keypoint_xy[:, 0] - x1) / box_width,
            (keypoint_xy[:, 1] - y1) / box_height,
        )
    )
    box_features = np.asarray(
        [
            ((x1 + x2) / 2.0) / width,
            ((y1 + y2) / 2.0) / height,
            box_width / width,
            box_height / height,
            box_width / box_height,
        ],
        dtype=np.float32,
    )
    return np.concatenate(
        [normalized_xy.reshape(-1), keypoint_conf, box_features]
    ).astype(np.float32)


def load_classifier(path: Path, device: torch.device):
    path = path.expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"BiLSTM 체크포인트가 없습니다: {path}")
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    required = {"model", "classes", "num_frames", "pose_features"}
    missing = required.difference(checkpoint)
    if missing:
        raise SystemExit(f"체크포인트 필드가 없습니다: {sorted(missing)}")
    classes = [str(name) for name in checkpoint["classes"]]
    if classes != ["NORMAL", "FALLING", "FALLEN"]:
        raise SystemExit(f"예상하지 못한 클래스입니다: {classes}")
    feature_count = int(checkpoint["pose_features"])
    if feature_count != 56:
        raise SystemExit(f"예상하지 못한 Pose 특징 수입니다: {feature_count}")
    model = FallPoseLSTM(feature_count, classes=len(classes)).to(device)
    model.load_state_dict(checkpoint["model"], strict=True)
    model.eval()
    return model, classes, int(checkpoint["num_frames"]), checkpoint


def put_label(canvas, box, text: str, color) -> None:
    import cv2

    x1, y1 = int(box[0]), max(int(box[1]) - 8, 24)
    size, baseline = cv2.getTextSize(
        text, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2
    )
    cv2.rectangle(
        canvas,
        (x1, y1 - size[1] - baseline - 5),
        (x1 + size[0] + 6, y1 + baseline),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        canvas,
        text,
        (x1 + 3, y1),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        color,
        2,
        cv2.LINE_AA,
    )


def main() -> int:
    args = parse_args()
    if not 0.0 <= args.conf <= 1.0:
        raise SystemExit("conf는 0 이상 1 이하여야 합니다.")
    if args.sample_fps <= 0 or args.smooth_window < 1 or args.fallen_hits < 1:
        raise SystemExit("sample-fps, smooth-window, fallen-hits는 양수여야 합니다.")
    source = resolve_source(args.source)
    yolo_device, torch_device = resolve_devices(args.device)
    classifier, classes, sequence_length, checkpoint = load_classifier(
        args.classifier, torch_device
    )
    pose_weights = args.pose_weights.expanduser().resolve()
    if not pose_weights.is_file():
        raise SystemExit(f"Pose 모델이 없습니다: {pose_weights}")

    import cv2
    from ultralytics import YOLO

    pose_model = YOLO(str(pose_weights))
    if pose_model.task != "pose":
        raise SystemExit(f"Pose 모델이 아닙니다: task={pose_model.task}")
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"카메라/영상을 열 수 없습니다: {source}")
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    window_name = "OmniFall YOLO11n-Pose + BiLSTM"
    if not args.no_show:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    sequences: dict[int, deque[np.ndarray]] = defaultdict(
        lambda: deque(maxlen=sequence_length)
    )
    probabilities: dict[int, deque[np.ndarray]] = defaultdict(
        lambda: deque(maxlen=args.smooth_window)
    )
    last_seen: dict[int, float] = {}
    fallen_streak: dict[int, int] = defaultdict(int)
    last_prediction: dict[int, tuple[str, np.ndarray]] = {}
    next_sample_at = monotonic()
    frame_index = 0

    print(f"분류기: {args.classifier.expanduser().resolve()}")
    print(f"Pose 모델: {pose_weights}")
    print(f"학습 당시 validation macro-F1: {checkpoint.get('val_macro_f1', 'unknown')}")
    print(
        f"시퀀스: {sequence_length} frames @ {args.sample_fps:g} Hz "
        f"({sequence_length / args.sample_fps:.2f}s)"
    )
    print("Q/ESC: 종료 | S: 현재 화면 저장")

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            started = perf_counter()
            result = pose_model.track(
                frame,
                persist=True,
                tracker="bytetrack.yaml",
                conf=args.conf,
                imgsz=args.imgsz,
                device=yolo_device,
                verbose=False,
            )[0]
            canvas = result.plot(labels=False, conf=False)
            now = monotonic()
            collect_sample = now >= next_sample_at
            if collect_sample:
                next_sample_at = now + 1.0 / args.sample_fps

            active_ids: list[int] = []
            if (
                result.boxes is not None
                and result.keypoints is not None
                and result.boxes.id is not None
                and result.keypoints.conf is not None
            ):
                boxes = result.boxes.xyxy.detach().cpu().numpy()
                track_ids = result.boxes.id.int().detach().cpu().tolist()
                all_xy = result.keypoints.xy.detach().cpu().numpy()
                all_conf = result.keypoints.conf.detach().cpu().numpy()
                for box, track_id, xy, confidence in zip(
                    boxes, track_ids, all_xy, all_conf
                ):
                    active_ids.append(track_id)
                    last_seen[track_id] = now
                    if collect_sample:
                        sequences[track_id].append(
                            pose_feature(box, xy, confidence, frame.shape)
                        )
                        if len(sequences[track_id]) == sequence_length:
                            tensor = torch.from_numpy(
                                np.stack(sequences[track_id])
                            ).unsqueeze(0).to(torch_device)
                            with torch.inference_mode():
                                prob = classifier(tensor).softmax(1)[0].cpu().numpy()
                            probabilities[track_id].append(prob)
                            mean_prob = np.mean(probabilities[track_id], axis=0)
                            class_index = int(np.argmax(mean_prob))
                            label = classes[class_index]
                            fallen_streak[track_id] = (
                                fallen_streak[track_id] + 1
                                if label == "FALLEN" else 0
                            )
                            last_prediction[track_id] = (label, mean_prob)

                    if track_id in last_prediction:
                        label, mean_prob = last_prediction[track_id]
                        alarm = (
                            label == "FALLEN"
                            and fallen_streak[track_id] >= args.fallen_hits
                        )
                        shown_label = "FALLEN ALERT" if alarm else label
                        confidence_value = float(np.max(mean_prob))
                        text = (
                            f"ID {track_id} {shown_label} "
                            f"{confidence_value:.2f} "
                            f"N/Fg/Fn={mean_prob[0]:.2f}/{mean_prob[1]:.2f}/{mean_prob[2]:.2f}"
                        )
                        color = COLORS[label]
                    else:
                        collected = len(sequences[track_id])
                        text = f"ID {track_id} COLLECTING {collected}/{sequence_length}"
                        color = COLORS["COLLECTING"]
                    put_label(canvas, box, text, color)

            stale_ids = [
                track_id for track_id, seen in last_seen.items()
                if now - seen > args.max_track_age
            ]
            for track_id in stale_ids:
                sequences.pop(track_id, None)
                probabilities.pop(track_id, None)
                last_seen.pop(track_id, None)
                fallen_streak.pop(track_id, None)
                last_prediction.pop(track_id, None)

            elapsed_ms = (perf_counter() - started) * 1000.0
            status = (
                f"people={len(active_ids)} | tracks={len(sequences)} | "
                f"{elapsed_ms:.1f} ms | frame={frame_index}"
            )
            cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 42), (0, 0, 0), -1)
            cv2.putText(
                canvas, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 255, 0), 2, cv2.LINE_AA,
            )
            frame_index += 1

            if args.no_show:
                continue
            cv2.imshow(window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                output = output_dir / f"pose_lstm_{frame_index:06d}.jpg"
                cv2.imwrite(str(output), canvas)
                print(f"저장: {output}")
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
