#!/usr/bin/env python3
"""여러 Ultralytics 학습 결과(results.csv)를 지표별 PNG로 비교한다."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Sequence


METRICS = {
    "precision": ("metrics/precision(B)", "Precision", "Precision"),
    "recall": ("metrics/recall(B)", "Recall", "Recall"),
    "map50": ("metrics/mAP50(B)", "mAP@0.5", "mAP@0.5"),
    "map50_95": (
        "metrics/mAP50-95(B)",
        "mAP@0.5:0.95",
        "mAP@0.5:0.95",
    ),
    "train_box_loss": ("train/box_loss", "Train Box Loss", "Loss"),
    "train_cls_loss": ("train/cls_loss", "Train Class Loss", "Loss"),
    "val_box_loss": ("val/box_loss", "Validation Box Loss", "Loss"),
    "val_cls_loss": ("val/cls_loss", "Validation Class Loss", "Loss"),
}
DASHBOARD_METRICS = (
    "precision",
    "recall",
    "map50",
    "map50_95",
    "train_box_loss",
    "val_box_loss",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "run_dirs",
        type=Path,
        nargs="+",
        help="각 모델의 results.csv가 들어 있는 학습 결과 폴더",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="비교 그래프를 저장할 폴더",
    )
    return parser.parse_args()


def read_results(run_dir: Path) -> dict[str, list[float]]:
    csv_path = run_dir / "results.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(f"results.csv를 찾지 못했습니다: {csv_path}")

    columns: dict[str, list[float]] = {}
    with csv_path.open(encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError(f"CSV 헤더가 없습니다: {csv_path}")
        normalized_names = {
            field_name: field_name.strip() for field_name in reader.fieldnames
        }
        for row in reader:
            for original_name, normalized_name in normalized_names.items():
                value = (row.get(original_name) or "").strip()
                if not value:
                    continue
                try:
                    columns.setdefault(normalized_name, []).append(float(value))
                except ValueError:
                    continue
    return columns


def add_metric_plot(
    axis: object,
    results: Sequence[tuple[str, dict[str, list[float]]]],
    metric_key: str,
) -> None:
    column, title, ylabel = METRICS[metric_key]
    for model_name, columns in results:
        values = columns.get(column, [])
        if values:
            epochs = columns.get("epoch", list(range(len(values))))
            axis.plot(
                epochs[: len(values)],
                values,
                linewidth=1.8,
                label=model_name,
            )
    axis.set_title(title)
    axis.set_xlabel("Epoch")
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)


def generate_comparison_plots(
    named_run_dirs: Sequence[tuple[str, Path]],
    output_dir: Path,
) -> list[Path]:
    # 쓰기 가능한 임시 캐시를 사용해 headless 환경에서도 안정적으로 저장한다.
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/yolo_training_matplotlib")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    results = [
        (model_name, read_results(run_dir.expanduser().resolve()))
        for model_name, run_dir in named_run_dirs
    ]
    created: list[Path] = []

    for metric_key, (_, title, _) in METRICS.items():
        figure, axis = plt.subplots(figsize=(10, 6))
        add_metric_plot(axis, results, metric_key)
        axis.legend(loc="best")
        figure.tight_layout()
        output_path = output_dir / f"comparison_{metric_key}.png"
        figure.savefig(output_path, dpi=160, bbox_inches="tight")
        plt.close(figure)
        created.append(output_path)

    figure, axes = plt.subplots(2, 3, figsize=(18, 10))
    for axis, metric_key in zip(axes.flat, DASHBOARD_METRICS):
        add_metric_plot(axis, results, metric_key)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    if handles:
        figure.legend(handles, labels, loc="lower center", ncol=3)
    figure.suptitle("YOLO Model Training Comparison", fontsize=16)
    figure.tight_layout(rect=(0, 0.06, 1, 0.96))
    dashboard_path = output_dir / "comparison_dashboard.png"
    figure.savefig(dashboard_path, dpi=160, bbox_inches="tight")
    plt.close(figure)
    created.append(dashboard_path)

    score_keys = ("precision", "recall", "map50", "map50_95")
    figure, axes = plt.subplots(2, 2, figsize=(14, 10))
    for axis, metric_key in zip(axes.flat, score_keys):
        column, title, ylabel = METRICS[metric_key]
        model_names = []
        best_values = []
        for model_name, columns in results:
            values = columns.get(column, [])
            if values:
                model_names.append(model_name)
                best_values.append(max(values))
        bars = axis.bar(model_names, best_values)
        axis.bar_label(bars, fmt="%.3f", padding=3)
        axis.set_title(f"Best {title}")
        axis.set_ylabel(ylabel)
        axis.set_ylim(0, 1.08)
        axis.tick_params(axis="x", rotation=25)
        axis.grid(True, axis="y", alpha=0.3)
    figure.suptitle("Best Validation Metrics by Model", fontsize=16)
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    best_metrics_path = output_dir / "comparison_best_metrics.png"
    figure.savefig(best_metrics_path, dpi=160, bbox_inches="tight")
    plt.close(figure)
    created.append(best_metrics_path)

    return created


def main() -> int:
    args = parse_args()
    named_run_dirs = [
        (run_dir.name.rsplit("_", 2)[0], run_dir) for run_dir in args.run_dirs
    ]
    created = generate_comparison_plots(named_run_dirs, args.output_dir)
    print("모델 비교 그래프 생성 완료")
    for path in created:
        print(f"- {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
