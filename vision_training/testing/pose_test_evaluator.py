#!/usr/bin/env python3
"""기존 YOLO Detection test split으로 Detect→Crop→Pose 파이프라인을 평가한다."""

from __future__ import annotations

import argparse
import csv
import json
import random
from collections import Counter
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
from ultralytics import YOLO

from detect_then_pose_webcam_test import COCO_SKELETON, best_pose, padded_crop
from posture_utils import classify_posture


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DETECTOR = ROOT / "src/aed_vision/models/rescue2_yolo11n.pt"
DEFAULT_POSE = ROOT / "src/aed_vision/models/yolo11n-pose.pt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--detector", type=Path, default=DEFAULT_DETECTOR)
    parser.add_argument("--pose", type=Path, default=DEFAULT_POSE)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--det-conf", type=float, default=0.20)
    parser.add_argument("--pose-conf", type=float, default=0.25)
    parser.add_argument("--keypoint-conf", type=float, default=0.30)
    parser.add_argument("--iou", type=float, default=0.50)
    parser.add_argument("--crop-padding", type=float, default=0.15)
    parser.add_argument("--samples", type=int, default=12)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def load_labels(path: Path, width: int, height: int) -> list[tuple[int, np.ndarray]]:
    labels = []
    if not path.exists():
        return labels
    for line in path.read_text().splitlines():
        values = line.split()
        if len(values) != 5:
            continue
        cls, cx, cy, bw, bh = map(float, values)
        labels.append((int(cls), np.array([
            (cx - bw / 2) * width, (cy - bh / 2) * height,
            (cx + bw / 2) * width, (cy + bh / 2) * height,
        ])))
    return labels


def iou(first: np.ndarray, second: np.ndarray) -> float:
    left, top = np.maximum(first[:2], second[:2])
    right, bottom = np.minimum(first[2:], second[2:])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    return intersection / max(first_area + second_area - intersection, 1e-9)


def draw_pose(canvas, points, scores, offset, threshold, color):
    ox, oy = offset
    visible = scores >= threshold
    for first, second in COCO_SKELETON:
        if visible[first] and visible[second]:
            p1 = tuple(np.round(points[first] + (ox, oy)).astype(int))
            p2 = tuple(np.round(points[second] + (ox, oy)).astype(int))
            cv2.line(canvas, p1, p2, color, 2)
    for index, point in enumerate(points):
        if visible[index]:
            center = tuple(np.round(point + (ox, oy)).astype(int))
            cv2.circle(canvas, center, 3, color, -1)


def main() -> int:
    args = parse_args()
    images_dir = args.dataset / "images/test"
    labels_dir = args.dataset / "labels/test"
    images = sorted(path for path in images_dir.iterdir() if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        raise SystemExit(f"test 이미지가 없습니다: {images_dir}")
    args.output.mkdir(parents=True, exist_ok=True)
    sample_dir = args.output / "samples"
    sample_dir.mkdir(exist_ok=True)
    sample_names = {path.name for path in random.Random(args.seed).sample(images, min(args.samples, len(images)))}

    detector = YOLO(str(args.detector))
    pose_model = YOLO(str(args.pose))
    rows = []
    totals = Counter()
    det_times, pose_times, total_times = [], [], []

    for image_path in images:
        frame = cv2.imread(str(image_path))
        if frame is None:
            continue
        height, width = frame.shape[:2]
        ground_truth = [box for cls, box in load_labels(labels_dir / f"{image_path.stem}.txt", width, height) if cls == 0]
        started = perf_counter()
        det_started = perf_counter()
        detection = detector.predict(frame, classes=[0, 1], conf=args.det_conf, imgsz=args.imgsz, device=args.device, verbose=False)[0]
        det_ms = (perf_counter() - det_started) * 1000
        predictions = []
        if detection.boxes is not None:
            for box, cls, conf in zip(detection.boxes.xyxy.cpu().numpy(), detection.boxes.cls.cpu().numpy(), detection.boxes.conf.cpu().numpy()):
                if int(cls) == 0:
                    predictions.append((box, float(conf)))

        used = set()
        matched = []
        for gt in ground_truth:
            candidates = [(iou(gt, pred[0]), index) for index, pred in enumerate(predictions) if index not in used]
            score, index = max(candidates, default=(0.0, -1))
            if score >= args.iou:
                used.add(index)
                matched.append((gt, predictions[index][0], predictions[index][1], score))

        canvas = frame.copy()
        pose_ms = 0.0
        posed = 0
        quality_pass = 0
        postures = Counter()
        visible_counts = []
        for gt, box, det_conf, match_iou in matched:
            crop, offset = padded_crop(frame, box, args.crop_padding)
            if crop is None:
                continue
            pose_started = perf_counter()
            result = pose_model.predict(crop, conf=args.pose_conf, imgsz=args.imgsz, device=args.device, verbose=False)[0]
            pose_ms += (perf_counter() - pose_started) * 1000
            selected = best_pose(result)
            if selected is None:
                continue
            pose_box, points, scores, pose_conf = selected
            posed += 1
            visible = int((scores >= args.keypoint_conf).sum())
            visible_counts.append(visible)
            if visible >= 6:
                quality_pass += 1
            points_with_conf = np.column_stack((points, scores))
            # 운영 파이프라인과 동일하게 불안정한 사람용 Pose bbox가 아니라
            # 1단계 mannequin detector bbox의 종횡비로 자세를 판단한다.
            posture, metrics = classify_posture(points_with_conf, box, args.keypoint_conf)
            postures[posture] += 1
            color = {"FALLEN": (0, 0, 255), "STANDING": (0, 190, 0), "SITTING": (0, 170, 255)}.get(posture, (160, 160, 160))
            draw_pose(canvas, points, scores, offset, args.keypoint_conf, color)
            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            cv2.putText(canvas, f"{posture} det={det_conf:.2f} pose={pose_conf:.2f} kp={visible}", (x1, max(18, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        total_ms = (perf_counter() - started) * 1000
        if image_path.name in sample_names:
            cv2.imwrite(str(sample_dir / image_path.name), canvas)
        totals.update(images=1, gt=len(ground_truth), detected=len(matched), posed=posed, quality_pass=quality_pass)
        totals.update(postures)
        det_times.append(det_ms)
        pose_times.append(pose_ms)
        total_times.append(total_ms)
        rows.append({
            "image": image_path.name, "gt_mannequin": len(ground_truth), "matched_detection": len(matched),
            "pose_generated": posed, "quality_pass_kp_ge_6": quality_pass,
            "standing": postures["STANDING"], "sitting": postures["SITTING"], "fallen": postures["FALLEN"], "unknown": postures["UNKNOWN"],
            "visible_keypoints_mean": round(float(np.mean(visible_counts)), 3) if visible_counts else 0.0,
            "det_ms": round(det_ms, 3), "pose_ms": round(pose_ms, 3), "total_ms": round(total_ms, 3),
        })

    with (args.output / "per_image.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    summary = {
        "dataset": str(args.dataset), "test_images": totals["images"], "gt_mannequin": totals["gt"],
        "matched_detection": totals["detected"], "detection_recall_iou_0_5": totals["detected"] / max(totals["gt"], 1),
        "pose_generated": totals["posed"], "pose_generation_rate_on_detected": totals["posed"] / max(totals["detected"], 1),
        "quality_pass_kp_ge_6": totals["quality_pass"], "quality_pass_rate_on_pose": totals["quality_pass"] / max(totals["posed"], 1),
        "posture_distribution": {name: totals[name] for name in ("STANDING", "SITTING", "FALLEN", "UNKNOWN")},
        "mean_det_ms_per_image": float(np.mean(det_times)), "mean_pose_ms_per_image": float(np.mean(pose_times)),
        "mean_pipeline_ms_per_image": float(np.mean(total_times)), "pipeline_fps": 1000.0 / max(float(np.mean(total_times)), 1e-9),
        "settings": {"device": args.device, "imgsz": args.imgsz, "det_conf": args.det_conf, "pose_conf": args.pose_conf, "keypoint_conf": args.keypoint_conf, "iou": args.iou, "crop_padding": args.crop_padding, "seed": args.seed, "sample_count": len(sample_names)},
        "limitation": "Detection 라벨만 있으므로 Pose keypoint mAP 및 자세 분류 정확도는 산출하지 않음",
    }
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
