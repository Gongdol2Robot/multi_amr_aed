#!/usr/bin/env python3
"""COCO 목각인형 데이터를 Colab용 YOLO Detection ZIP으로 변환한다."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import shutil


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    ROOT / "training" / "dataset" / "mannequin_combined.coco"
)
DEFAULT_OUTPUT = ROOT / "training" / "mannequin_dataset.zip"
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
SOURCE_SPLITS = {"train": "train", "valid": "val", "val": "val", "test": "test"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


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
    annotation_files = {
        path.parent.name: path
        for path in source.glob("*/_annotations.coco.json")
        if path.parent.name in SOURCE_SPLITS
    }
    required = {"train", "valid", "test"}
    if not required.issubset(annotation_files):
        raise SystemExit(
            "COCO train/valid/test annotation JSON이 모두 필요합니다: "
            f"{sorted(annotation_files)}"
        )
    split_data = {
        SOURCE_SPLITS[name]: (
            path.parent,
            json.loads(path.read_text(encoding="utf-8")),
        )
        for name, path in annotation_files.items()
    }
    categories = {
        item["name"]: item["id"]
        for item in split_data["train"][1]["categories"]
    }
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

    work_dir = output.parent / f".{output.stem}_build"
    if work_dir.exists():
        shutil.rmtree(work_dir)
    dataset_dir = work_dir / "mannequin_dataset"
    counts = Counter()
    for split in ("train", "val", "test"):
        image_dir, data = split_data[split]
        current_categories = {
            item["name"]: item["id"] for item in data["categories"]
        }
        current_mapping = {
            current_categories[name]: class_id
            for name, class_id in (("mannequin", 0), ("helping_person", 1))
            if name in current_categories
        }
        annotations_by_image: dict[int, list[dict]] = {}
        for annotation in data["annotations"]:
            if annotation["category_id"] in current_mapping:
                annotations_by_image.setdefault(
                    annotation["image_id"], []
                ).append(annotation)
        for image in data["images"]:
            source_image = image_dir / image["file_name"]
            if not source_image.is_file():
                raise FileNotFoundError(source_image)
            if source_image.suffix.lower() not in IMAGE_EXTENSIONS:
                raise ValueError(f"지원하지 않는 이미지 형식: {source_image}")
            destination_images = dataset_dir / "images" / split
            destination_labels = dataset_dir / "labels" / split
            destination_images.mkdir(parents=True, exist_ok=True)
            destination_labels.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_image, destination_images / source_image.name)
            lines = []
            for annotation in annotations_by_image.get(image["id"], []):
                class_id = current_mapping[annotation["category_id"]]
                lines.append(yolo_line(annotation, image, class_id))
                counts[f"class_{class_id}"] += 1
            (destination_labels / f"{source_image.stem}.txt").write_text(
                ("\n".join(lines) + "\n") if lines else "",
                encoding="utf-8",
            )
            counts[f"{split}_images"] += 1
            counts["positive_images" if lines else "empty_images"] += 1

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
