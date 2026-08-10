#!/usr/bin/env python3
"""기존 Detection 데이터의 mannequin crop으로 FALLEN/NON_FALLEN SVM을 학습한다."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
import sys
sys.path.insert(0, str(ROOT / "src/aed_vision"))

from aed_vision.posture_classifier import crop_with_padding, hog_features


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--padding", type=float, default=0.35)
    parser.add_argument("--c", type=float, default=2.0)
    parser.add_argument("--gamma", type=float, default=0.0005669)
    return parser.parse_args()


def weak_label(name: str) -> int:
    return int(name.startswith(("fallen_person", "helper_rc_car")))


def load_box(path: Path, width: int, height: int):
    for line in path.read_text().splitlines():
        cls, cx, cy, bw, bh = map(float, line.split())
        if int(cls) == 0:
            return np.array([
                (cx - bw / 2) * width, (cy - bh / 2) * height,
                (cx + bw / 2) * width, (cy + bh / 2) * height,
            ])
    return None


def rotate(image, degrees):
    height, width = image.shape[:2]
    matrix = cv2.getRotationMatrix2D((width / 2, height / 2), degrees, 1.0)
    return cv2.warpAffine(image, matrix, (width, height), borderMode=cv2.BORDER_REFLECT_101)


def load_split(dataset: Path, split: str, augment: bool, padding: float):
    features, labels, names = [], [], []
    for image_path in sorted((dataset / f"images/{split}").iterdir()):
        if image_path.suffix.lower() not in {".jpg", ".jpeg", ".png"}:
            continue
        frame = cv2.imread(str(image_path))
        if frame is None:
            continue
        height, width = frame.shape[:2]
        box = load_box(dataset / f"labels/{split}/{image_path.stem}.txt", width, height)
        if box is None:
            continue
        crop = crop_with_padding(frame, box, padding)
        variants = [crop]
        if augment:
            variants += [rotate(crop, -25), rotate(crop, 25), cv2.flip(crop, 1)]
        for variant in variants:
            features.append(hog_features(variant)[0])
            labels.append(weak_label(image_path.name))
            names.append(image_path.name)
    return np.asarray(features, np.float32), np.asarray(labels, np.int32), names


def metrics(truth, predicted):
    truth, predicted = np.asarray(truth), np.asarray(predicted)
    tp = int(((truth == 1) & (predicted == 1)).sum())
    fn = int(((truth == 1) & (predicted == 0)).sum())
    fp = int(((truth == 0) & (predicted == 1)).sum())
    tn = int(((truth == 0) & (predicted == 0)).sum())
    recall = tp / max(tp + fn, 1)
    specificity = tn / max(tn + fp, 1)
    precision = tp / max(tp + fp, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-9)
    return {"tp": tp, "fn": fn, "fp": fp, "tn": tn, "fallen_recall": recall,
            "normal_specificity": specificity, "fallen_precision": precision,
            "fallen_f1": f1, "balanced_accuracy": (recall + specificity) / 2}


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    train_x, train_y, _ = load_split(
        args.dataset, "train", True, args.padding
    )
    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_RBF)
    svm.setC(args.c)
    svm.setGamma(args.gamma)
    counts = np.bincount(train_y, minlength=2).astype(np.float32)
    svm.setClassWeights((counts.sum() / np.maximum(counts, 1)).reshape(-1, 1))
    svm.train(train_x, cv2.ml.ROW_SAMPLE, train_y)
    svm.save(str(args.output))
    report = {"train_samples_with_augmentation": int(len(train_y)), "class_counts": counts.astype(int).tolist(), "c": args.c, "gamma": args.gamma}
    for split in ("val", "test"):
        x, y, _ = load_split(args.dataset, split, False, args.padding)
        _, prediction = svm.predict(x)
        report[split] = metrics(y, prediction.ravel().astype(int))
        report[f"{split}_samples"] = int(len(y))
    report_path = args.output.with_suffix(".metrics.json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
