#!/usr/bin/env python3
"""YOLO Keypoint Detection 데이터로 YOLO11 Pose 모델을 파인튜닝한다."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / "models" / "yolo11n-pose.pt"
DEFAULT_RUNS = ROOT / "runs" / "pose"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data",
        type=Path,
        required=True,
        help="kpt_shape가 포함된 Pose data.yaml",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--batch",
        type=int,
        default=8,
        help="6GB GPU 권장값. 메모리 부족 시 4 또는 2로 낮추세요.",
    )
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS)
    parser.add_argument("--name", default=None)
    parser.add_argument(
        "--resume",
        action="store_true",
        help="--model에 지정한 last.pt에서 중단 학습 재개",
    )
    return parser.parse_args()


def resolve_dataset_path(
    data_yaml: Path, dataset_root: str | None, value: str
) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    root = Path(dataset_root).expanduser() if dataset_root else Path()
    if not root.is_absolute():
        root = data_yaml.parent / root
    return root / path


def validate_pose_dataset(data_yaml: Path) -> Path:
    """Detection 라벨을 Pose 학습에 넣는 실수를 학습 전에 차단한다."""
    data_yaml = data_yaml.expanduser().resolve()
    if not data_yaml.is_file():
        raise SystemExit(f"Pose data.yaml을 찾지 못했습니다: {data_yaml}")

    with data_yaml.open(encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}
    keypoint_shape = config.get("kpt_shape")
    if (
        not isinstance(keypoint_shape, list)
        or len(keypoint_shape) != 2
        or not all(isinstance(value, int) for value in keypoint_shape)
        or keypoint_shape[0] < 1
        or keypoint_shape[1] not in (2, 3)
    ):
        raise SystemExit(
            "data.yaml에 유효한 'kpt_shape: [관절 수, 2 또는 3]'이 필요합니다."
        )
    flip_idx = config.get("flip_idx")
    if flip_idx is not None and len(flip_idx) != keypoint_shape[0]:
        raise SystemExit("flip_idx 길이는 kpt_shape의 관절 수와 같아야 합니다.")
    pose_label_fields = 5 + keypoint_shape[0] * keypoint_shape[1]

    checked = 0
    for split in ("train", "val"):
        split_value = config.get(split)
        if not isinstance(split_value, str):
            raise SystemExit(f"data.yaml에 {split} 이미지 경로가 필요합니다.")
        image_dir = resolve_dataset_path(
            data_yaml, config.get("path"), split_value
        )
        label_dir = (
            image_dir.parent / "labels"
            if image_dir.name == "images"
            else Path(str(image_dir).replace("/images/", "/labels/"))
        )
        if not label_dir.is_dir():
            raise SystemExit(f"{split} Pose 라벨 폴더가 없습니다: {label_dir}")
        for label_path in label_dir.rglob("*.txt"):
            for line_number, line in enumerate(
                label_path.read_text(encoding="utf-8").splitlines(), 1
            ):
                if not line.strip():
                    continue
                fields = line.split()
                if len(fields) != pose_label_fields:
                    raise SystemExit(
                        f"{label_path}:{line_number}: Pose 라벨은 "
                        f"{pose_label_fields}필드여야 하지만 {len(fields)}필드입니다."
                    )
                checked += 1
    if checked == 0:
        raise SystemExit("학습·검증 Pose 라벨이 한 개도 없습니다.")
    print(
        f"Pose 라벨 검사 완료: {checked}개 객체, "
        f"관절 {keypoint_shape[0]}개 × {keypoint_shape[1]}차원"
    )
    return data_yaml


def main() -> int:
    args = parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise SystemExit(f"Pose 초기 가중치를 찾지 못했습니다: {model_path}")
    if args.epochs < 1 or args.batch < 1 or args.patience < 0:
        raise SystemExit("epochs/batch는 1 이상, patience는 0 이상이어야 합니다.")
    data_yaml = validate_pose_dataset(args.data)

    from ultralytics import YOLO

    model = YOLO(str(model_path))
    if model.task != "pose":
        raise SystemExit(f"Pose 모델이 아닙니다: task={model.task}")

    run_name = args.name or datetime.now().strftime("yolo11n_pose_%Y%m%d_%H%M%S")
    options = {
        "data": str(data_yaml),
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "batch": args.batch,
        "device": args.device,
        "workers": args.workers,
        "patience": args.patience,
        "project": str(args.runs_dir.expanduser().resolve()),
        "name": run_name,
        "seed": 42,
        "deterministic": True,
        "amp": True,
        "plots": True,
        "resume": args.resume,
    }
    model.train(**options)
    best = Path(options["project"]) / run_name / "weights" / "best.pt"
    print(f"학습 완료: {best}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
