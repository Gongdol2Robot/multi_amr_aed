#!/usr/bin/env python3
"""서기·앉기 등 음성 사진의 빈 라벨을 만들고 학습 데이터에 추가한다."""

from __future__ import annotations

import argparse
from pathlib import Path
import random
import re
import shutil


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "hard_negatives" / "images"
DEFAULT_SOURCE_LABELS = ROOT / "hard_negatives" / "labels"
DEFAULT_DATASET = ROOT / "datasets" / "rescue"
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
SPLIT_RATIOS = {"train": 0.8, "val": 0.1, "test": 0.1}
OUTPUT_PREFIX = "hard_negative_"
TIMESTAMP_PATTERN = re.compile(r"_(\d{8})_(\d{2})(\d{2})(\d{2})_")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=DEFAULT_SOURCE,
        help="서기·앉기·숙이기·빈 바닥 원본 사진 폴더",
    )
    parser.add_argument(
        "--source-labels",
        type=Path,
        default=DEFAULT_SOURCE_LABELS,
        help="원본 사진과 같은 이름의 빈 라벨을 만들 폴더",
    )
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def capture_group(path: Path) -> tuple:
    match = TIMESTAMP_PATTERN.search(path.name)
    if match is None:
        return (path.stem,)
    date, hour, minute, second = match.groups()
    return (date, hour, minute, int(second) // 10)


def split_images(images: list[Path], seed: int) -> dict[str, list[Path]]:
    groups: dict[tuple, list[Path]] = {}
    for image in images:
        groups.setdefault(capture_group(image), []).append(image)
    grouped_images = list(groups.values())
    random.Random(seed).shuffle(grouped_images)
    grouped_images.sort(key=len, reverse=True)

    targets = {
        split: len(images) * ratio for split, ratio in SPLIT_RATIOS.items()
    }
    result = {split: [] for split in SPLIT_RATIOS}
    for group in grouped_images:
        split = max(
            SPLIT_RATIOS,
            key=lambda name: targets[name] - len(result[name]),
        )
        result[split].extend(group)
    return result


def main() -> int:
    args = parse_args()
    source = args.source.expanduser().resolve()
    source_labels = args.source_labels.expanduser().resolve()
    dataset = args.dataset.expanduser().resolve()
    source.mkdir(parents=True, exist_ok=True)
    source_labels.mkdir(parents=True, exist_ok=True)

    images = sorted(
        path for path in source.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )
    if not images:
        raise SystemExit(
            f"음성 사진이 없습니다: {source}\n"
            "서기·앉기·숙이기·빈 바닥 사진을 이 폴더에 넣으세요."
        )

    for image in images:
        label = source_labels / f"{image.stem}.txt"
        if label.exists() and label.stat().st_size > 0:
            raise SystemExit(
                f"음성 데이터 라벨은 비어 있어야 합니다: {label}"
            )
        label.touch(exist_ok=True)

    for split in SPLIT_RATIOS:
        image_dir = dataset / "images" / split
        label_dir = dataset / "labels" / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        for directory in (image_dir, label_dir):
            for path in directory.glob(f"{OUTPUT_PREFIX}*"):
                if path.is_file():
                    path.unlink()

    splits = split_images(images, args.seed)
    for split, split_images_list in splits.items():
        for image in split_images_list:
            stem = f"{OUTPUT_PREFIX}{image.stem}"
            shutil.copy2(
                image,
                dataset / "images" / split / f"{stem}{image.suffix.lower()}",
            )
            (dataset / "labels" / split / f"{stem}.txt").touch()

    print(f"음성 데이터 준비 완료: 총 {len(images)}장")
    for split, split_images_list in splits.items():
        print(f"- {split}: {len(split_images_list)}장, 빈 라벨 생성")
    print(f"- 원본 빈 라벨: {source_labels}")
    print(f"- 파인튜닝 데이터: {dataset}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
