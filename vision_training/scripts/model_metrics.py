"""YOLO 모델별 핵심 학습·검증·추론 지표를 수집하고 비교 자료를 만든다."""

from __future__ import annotations

import csv
import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

import yaml


@dataclass
class ModelMetrics:
    model: str
    initial_weights: str
    data: str
    eval_split: str
    epochs_requested: int
    epochs_completed: int
    early_stopped: bool
    training_time_seconds: float
    imgsz: int
    batch: int
    patience: int
    device: str
    precision: float
    correct_predictions_percent: float
    recall: float
    map50: float
    map50_95: float
    best_f1: float
    best_f1_confidence: float
    inference_ms_per_image: float
    inference_fps: float
    average_confidence: float
    predicted_boxes: int
    evaluated_images: int
    best_weights: str
    train_run_dir: str
    val_run_dir: str


def completed_epochs(results_csv: Path) -> int:
    with results_csv.open(encoding="utf-8", newline="") as file:
        return sum(1 for _ in csv.DictReader(file))


def resolve_eval_source(data_yaml: Path, split: str) -> tuple[str | list[str], int]:
    content = yaml.safe_load(data_yaml.read_text(encoding="utf-8")) or {}
    if split not in content:
        raise SystemExit(
            f"data.yaml에 '{split}' split이 없습니다: {data_yaml}\n"
            "현재 데이터셋은 --eval-split val을 사용하거나 data.yaml에 "
            f"{split}: 경로를 추가하세요."
        )

    dataset_root_value = content.get("path", data_yaml.parent)
    dataset_root = Path(str(dataset_root_value)).expanduser()
    if not dataset_root.is_absolute():
        dataset_root = (data_yaml.parent / dataset_root).resolve()

    raw_sources = content[split]
    values = raw_sources if isinstance(raw_sources, list) else [raw_sources]
    sources: list[str] = []
    image_count = 0
    image_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    for value in values:
        source = Path(str(value)).expanduser()
        if not source.is_absolute():
            source = dataset_root / source
        source = source.resolve()
        if not source.exists():
            raise SystemExit(f"{split} 이미지 경로를 찾지 못했습니다: {source}")
        sources.append(str(source))
        if source.is_dir():
            image_count += sum(
                1
                for path in source.rglob("*")
                if path.is_file() and path.suffix.lower() in image_extensions
            )
        elif source.suffix.lower() == ".txt":
            image_count += sum(
                1
                for line in source.read_text(encoding="utf-8").splitlines()
                if line.strip()
            )
        else:
            image_count += 1
    return (sources[0] if len(sources) == 1 else sources), image_count


def average_prediction_confidence(
    model: Any,
    source: str | list[str],
    imgsz: int,
    device: str | None,
) -> tuple[float, int]:
    options: dict[str, Any] = {
        "imgsz": imgsz,
        "stream": True,
        "verbose": False,
        "save": False,
    }
    if device is not None:
        options["device"] = device

    confidence_sum = 0.0
    predicted_boxes = 0
    # Ultralytics interprets a list as a list of individual images. Passing a
    # list of dataset directories therefore makes PIL try to open each
    # directory as an image. Run each directory/list file as its own source.
    sources = source if isinstance(source, list) else [source]
    for prediction_source in sources:
        options["source"] = prediction_source
        for result in model.predict(**options):
            if result.boxes is None or result.boxes.conf is None:
                continue
            confidences = result.boxes.conf.detach().cpu()
            confidence_sum += float(confidences.sum())
            predicted_boxes += int(confidences.numel())
    average = confidence_sum / predicted_boxes if predicted_boxes else 0.0
    return average, predicted_boxes


def best_f1_values(box_metrics: Any) -> tuple[float, float]:
    import numpy as np

    f1_curve = np.asarray(box_metrics.f1_curve, dtype=float)
    confidence_axis = np.asarray(box_metrics.px, dtype=float)
    if f1_curve.size == 0 or confidence_axis.size == 0:
        return 0.0, 0.0
    if f1_curve.ndim == 1:
        mean_f1 = f1_curve
    else:
        mean_f1 = f1_curve.mean(axis=0)
    best_index = int(mean_f1.argmax())
    return float(mean_f1[best_index]), float(confidence_axis[best_index])


def build_model_metrics(
    *,
    model_name: str,
    initial_weights: Path,
    data_yaml: Path,
    eval_split: str,
    epochs_requested: int,
    training_time_seconds: float,
    imgsz: int,
    batch: int,
    patience: int,
    device: str | None,
    best_weights: Path,
    train_run_dir: Path,
    val_run_dir: Path,
    val_results: Any,
    average_confidence: float,
    predicted_boxes: int,
    evaluated_images: int,
) -> ModelMetrics:
    epochs_done = completed_epochs(train_run_dir / "results.csv")
    precision = float(val_results.box.mp)
    recall = float(val_results.box.mr)
    map50 = float(val_results.box.map50)
    map50_95 = float(val_results.box.map)
    best_f1, best_f1_confidence = best_f1_values(val_results.box)
    inference_ms = float(val_results.speed.get("inference", 0.0))
    inference_fps = 1000.0 / inference_ms if inference_ms > 0 else 0.0
    return ModelMetrics(
        model=model_name,
        initial_weights=str(initial_weights),
        data=str(data_yaml),
        eval_split=eval_split,
        epochs_requested=epochs_requested,
        epochs_completed=epochs_done,
        early_stopped=epochs_done < epochs_requested,
        training_time_seconds=training_time_seconds,
        imgsz=imgsz,
        batch=batch,
        patience=patience,
        device=device or "auto",
        precision=precision,
        correct_predictions_percent=precision * 100.0,
        recall=recall,
        map50=map50,
        map50_95=map50_95,
        best_f1=best_f1,
        best_f1_confidence=best_f1_confidence,
        inference_ms_per_image=inference_ms,
        inference_fps=inference_fps,
        average_confidence=average_confidence,
        predicted_boxes=predicted_boxes,
        evaluated_images=evaluated_images,
        best_weights=str(best_weights),
        train_run_dir=str(train_run_dir),
        val_run_dir=str(val_run_dir),
    )


def write_metrics_reports(
    metrics: Sequence[ModelMetrics],
    output_dir: Path,
) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = [asdict(item) for item in metrics]
    json_path = output_dir / "comparison_metrics.json"
    csv_path = output_dir / "comparison_metrics.csv"
    json_path.write_text(
        json.dumps(
            {
                "definitions": {
                    "correct_predictions_percent": (
                        "Detection precision × 100: 전체 예측 박스 중 "
                        "평가 기준을 만족한 올바른 탐지의 비율"
                    ),
                    "average_confidence": (
                        "평가 이미지에서 생성된 모든 예측 박스 confidence의 평균"
                    ),
                    "best_f1_confidence": (
                        "클래스 평균 F1이 최대가 되는 confidence threshold"
                    ),
                    "inference_fps": (
                        "Ultralytics 검증 inference 시간만 사용한 1000/ms 값; "
                        "전처리·후처리·영상 입출력 제외"
                    ),
                },
                "models": rows,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return csv_path, json_path


def generate_key_metrics_plots(
    metrics: Sequence[ModelMetrics],
    output_dir: Path,
) -> list[Path]:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/yolo_training_matplotlib")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    names = [item.model for item in metrics]
    created: list[Path] = []

    plot_specs = (
        (
            "comparison_validation_metrics.png",
            "Validation Metrics",
            ("precision", "recall", "map50", "map50_95", "best_f1"),
            ("Precision", "Recall", "mAP50", "mAP50-95", "Best F1"),
            "Score",
            (0, 1.08),
        ),
        (
            "comparison_training.png",
            "Training Cost",
            ("epochs_completed", "training_time_seconds"),
            ("Completed Epochs", "Training Time (s)"),
            "Value",
            None,
        ),
        (
            "comparison_test_inference.png",
            "Evaluation Inference",
            (
                "correct_predictions_percent",
                "inference_fps",
                "average_confidence",
            ),
            ("% Correct Predictions", "Inference FPS", "Avg Confidence"),
            "Value (different units)",
            None,
        ),
        (
            "comparison_f1_confidence.png",
            "Best F1 and Confidence Threshold",
            ("best_f1", "best_f1_confidence"),
            ("Best F1", "Confidence at Best F1"),
            "Score / Confidence",
            (0, 1.08),
        ),
    )

    for filename, title, fields, labels, ylabel, ylim in plot_specs:
        figure, axes = plt.subplots(
            len(fields),
            1,
            figsize=(12, max(4, 3.4 * len(fields))),
            squeeze=False,
        )
        for axis, field, label in zip(axes.flat, fields, labels):
            values = [float(getattr(item, field)) for item in metrics]
            bars = axis.bar(names, values)
            axis.bar_label(bars, fmt="%.3f", padding=3)
            axis.set_title(label)
            axis.set_ylabel(ylabel)
            if ylim is not None:
                axis.set_ylim(*ylim)
            axis.tick_params(axis="x", rotation=20)
            axis.grid(True, axis="y", alpha=0.3)
        figure.suptitle(title, fontsize=16)
        figure.tight_layout(rect=(0, 0, 1, 0.97))
        path = output_dir / filename
        figure.savefig(path, dpi=160, bbox_inches="tight")
        plt.close(figure)
        created.append(path)
    return created
