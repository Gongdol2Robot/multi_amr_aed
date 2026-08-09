#!/usr/bin/env python3
"""COCO 목각인형 데이터를 Colab용 YOLO Detection ZIP으로 변환한다."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import random
import re
import shutil


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    ROOT / "training" / "dataset" / "final_proj-Folder- raw.coco"
)
DEFAULT_OUTPUT = ROOT / "training" / "mannequin_dataset.zip"
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
SPLIT_RATIOS = {"train": 0.8, "val": 0.1, "test": 0.1}
TIMESTAMP = re.compile(r"_(\d{8})_(\d{6})_")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def capture_group(file_name: str) -> str:
    """인접 연속 프레임을 10초 묶음으로 만들어 split 누수를 줄인다."""
    match = TIMESTAMP.search(file_name)
    if match is None:
        return file_name
    date, hhmmss = match.groups()
    return f"{date}_{hhmmss[:4]}_{int(hhmmss[4:]) // 10}"


def assign_splits(images: list[dict], seed: int) -> dict[int, str]:
    groups: dict[str, list[dict]] = {}
    for image in images:
        groups.setdefault(capture_group(image["file_name"]), []).append(image)
    grouped = list(groups.values())
    random.Random(seed).shuffle(grouped)
    grouped.sort(key=len, reverse=True)
    targets = {
        split: len(images) * ratio for split, ratio in SPLIT_RATIOS.items()
    }
    assigned = {split: [] for split in SPLIT_RATIOS}
    for group in grouped:
        split = max(
            SPLIT_RATIOS,
            key=lambda name: targets[name] - len(assigned[name]),
        )
        assigned[split].extend(group)
    return {
        image["id"]: split
        for split, split_images in assigned.items()
        for image in split_images
    }


def yolo_line(annotation: dict, image: dict, class_id: int) -> str:
    x, y, width, height = (float(value) for value in annotation["bbox"])
    image_width = float(image["width"])
    image_height = float(image["height"])
    cx = (x + width / 2.0) / image_width
    cy = (y + height / 2.0) / image_height
    normalized_width = width / image_width
    normalized_height = height / image_height
    values = (cx, cy, normalized_width, normalized_height)
    if any(value < 0.0 or value > 1.0 for value in values):
        raise ValueError(
            f"이미지 {image['file_name']}의 bbox가 범위를 벗어납니다: "
            f"{annotation['bbox']}"
        )
    return f"{class_id} " + " ".join(f"{value:.6f}" for value in values)


def main() -> int:
    args = parse_args()
    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    annotation_files = sorted(source.glob("*/_annotations.coco.json"))
    if len(annotation_files) != 1:
        raise SystemExit(
            "현재 변환기는 COCO annotation JSON 하나를 기대합니다: "
            f"{annotation_files}"
        )
    annotation_file = annotation_files[0]
    image_dir = annotation_file.parent
    data = json.loads(annotation_file.read_text(encoding="utf-8"))
    categories = {item["name"]: item["id"] for item in data["categories"]}
    if "fallen_person" in categories:
        raise SystemExit(
            "fallen_person 카테고리가 남아 있습니다. 먼저 mannequin으로 "
            "병합하세요."
        )
    if "mannequin" not in categories:
        raise SystemExit("mannequin 카테고리가 없습니다.")

    # helping_person은 현재 데이터의 두 번째 검출 클래스로 보존한다.
    class_mapping = {categories["mannequin"]: 0}
    names = {0: "mannequin"}
    if "helping_person" in categories:
        class_mapping[categories["helping_person"]] = 1
        names[1] = "helping_person"

    images = data["images"]
    images_by_id = {item["id"]: item for item in images}
    split_by_id = assign_splits(images, args.seed)
    annotations_by_image: dict[int, list[dict]] = {}
    for annotation in data["annotations"]:
        if annotation["category_id"] in class_mapping:
            annotations_by_image.setdefault(
                annotation["image_id"], []
            ).append(annotation)

    work_dir = output.parent / f".{output.stem}_build"
    if work_dir.exists():
        shutil.rmtree(work_dir)
    dataset_dir = work_dir / "mannequin_dataset"
    counts = Counter()
    for image in images:
        source_image = image_dir / image["file_name"]
        if not source_image.is_file():
            raise FileNotFoundError(source_image)
        if source_image.suffix.lower() not in IMAGE_EXTENSIONS:
            raise ValueError(f"지원하지 않는 이미지 형식: {source_image}")
        split = split_by_id[image["id"]]
        destination_images = dataset_dir / "images" / split
        destination_labels = dataset_dir / "labels" / split
        destination_images.mkdir(parents=True, exist_ok=True)
        destination_labels.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_image, destination_images / source_image.name)
        lines = []
        for annotation in annotations_by_image.get(image["id"], []):
            class_id = class_mapping[annotation["category_id"]]
            lines.append(yolo_line(annotation, image, class_id))
            counts[f"class_{class_id}"] += 1
        (destination_labels / f"{source_image.stem}.txt").write_text(
            ("\n".join(lines) + "\n") if lines else "", encoding="utf-8"
        )
        counts[f"{split}_images"] += 1

    data_yaml = (
        "path: .\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        "names:\n"
        + "".join(f"  {class_id}: {name}\n" for class_id, name in names.items())
    )
    (dataset_dir / "data.yaml").write_text(data_yaml, encoding="utf-8")
    (dataset_dir / "DATASET_INFO.txt").write_text(
        "COCO 원본을 YOLO Detection으로 변환한 전 자세 목각인형 데이터셋\n"
        + "\n".join(f"{key}: {value}" for key, value in sorted(counts.items()))
        + "\n",
        encoding="utf-8",
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    archive_base = output.with_suffix("")
    created = Path(
        shutil.make_archive(
            str(archive_base), "zip", root_dir=work_dir,
            base_dir=dataset_dir.name,
        )
    )
    shutil.rmtree(work_dir)
    print(f"Colab 데이터 ZIP 생성: {created}")
    print(f"크기: {created.stat().st_size / 1024 / 1024:.1f} MiB")
    for key, value in sorted(counts.items()):
        print(f"- {key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
