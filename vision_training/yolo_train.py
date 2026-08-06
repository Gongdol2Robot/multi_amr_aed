#!/usr/bin/env python3
"""구조 상황 데이터로 YOLOv8, YOLO11, YOLO26의 n/s 모델을 비교한다."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
from time import perf_counter

import yaml


TRAINING_ROOT = Path(__file__).resolve().parent
DEFAULT_MODELS = (
    TRAINING_ROOT / "models" / "yolov8n.pt",
    TRAINING_ROOT / "models" / "yolov8s.pt",
    TRAINING_ROOT / "models" / "yolo11n.pt",
    TRAINING_ROOT / "models" / "yolo11s.pt",
    TRAINING_ROOT / "models" / "yolo26n.pt",
    TRAINING_ROOT / "models" / "yolo26s.pt",
)
DEFAULT_DATA = TRAINING_ROOT / "data.yaml"
DEFAULT_RUNS = TRAINING_ROOT / "runs"
REQUIRED_NAMES = ["fallen_person", "helper_rc_car"]


def parse_args() -> argparse.Namespace:
    """1차 학습에 사용할 명령행 인자를 정의하고 파싱해 반환한다.

    반환된 값에는 데이터 경로, 초기 가중치 목록, 학습 횟수와 배치 크기,
    데이터 증강 범위, 결과 저장 위치 등이 들어 있다.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data",
        type=Path,
        default=DEFAULT_DATA,
        help="fallen_person(0), helper_rc_car(1) YOLO Detection data.yaml",
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--batch",
        type=int,
        default=8,
        help="배치 크기 (기본: 8, 6GB GPU의 멀티스케일 학습 기준)",
    )
    parser.add_argument("--device", default=None, help="예: 0 또는 cpu (기본: 자동)")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="객체/장면 랜덤 확대·축소 폭 (기본: 1.0, 최대 약 2배)",
    )
    parser.add_argument(
        "--multi-scale",
        type=float,
        default=0.25,
        help="배치별 입력 해상도 변화 폭 (기본: 0.25, imgsz의 ±25%%)",
    )
    parser.add_argument(
        "--patience",
        type=int,
        default=20,
        help=(
            "검증 지표가 개선되지 않아도 기다릴 epoch 수 "
            "(기본: 20, 0이면 조기 종료 비활성화)"
        ),
    )
    parser.add_argument(
        "--amp",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="자동 혼합정밀(AMP). CUDA 오류가 나면 --no-amp로 끄세요.",
    )
    parser.add_argument(
        "--deterministic",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="결정론적 학습. CUDA 오류가 나면 --no-deterministic로 끄세요.",
    )
    parser.add_argument(
        "--plots",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="학습·검증 플롯 저장 (기본: 활성화)",
    )
    parser.add_argument(
        "--eval-split",
        choices=("val", "test"),
        default="val",
        help="최종 모델 비교에 사용할 데이터 split (기본: val)",
    )
    parser.add_argument(
        "--models",
        type=Path,
        nargs="+",
        default=list(DEFAULT_MODELS),
        help="비교할 초기 가중치 목록 (기본: YOLOv8/11/26의 n, s)",
    )
    parser.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS)
    return parser.parse_args()


def require_file(path: Path, description: str) -> Path:
    """파일의 절대 경로를 반환하고, 파일이 없으면 설명과 함께 종료한다.

    Args:
        path: 존재 여부를 확인할 파일 경로.
        description: 오류 메시지에서 파일의 용도를 나타내는 이름.

    Returns:
        사용자 홈 표시와 상대 경로를 정리한 절대 경로.
    """
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise SystemExit(f"{description}을 찾지 못했습니다: {resolved}")
    return resolved


def validate_data_yaml(data_yaml: Path) -> Path:
    """data.yaml이 존재하며 클래스 번호와 이름이 정확한지 확인한다.

    YOLO 라벨의 첫 숫자 0과 1이 각각 fallen_person과 helper_rc_car를
    뜻해야 하므로, 클래스 순서가 다르면 잘못 학습하기 전에 실행을 중단한다.

    Returns:
        검증을 통과한 data.yaml의 절대 경로.
    """
    data_yaml = require_file(data_yaml, "학습 data.yaml")
    with data_yaml.open(encoding="utf-8") as file:
        names = (yaml.safe_load(file) or {}).get("names")
    if isinstance(names, dict):
        names = [names[key] for key in sorted(names, key=lambda key: int(key))]
    if names != REQUIRED_NAMES:
        raise SystemExit(
            f"data.yaml의 클래스 순서는 반드시 {REQUIRED_NAMES}여야 합니다.\n"
            f"현재 names: {names}"
        )
    return data_yaml


def main() -> int:
    """6개 초기 모델의 학습·검증·성능 비교 과정을 순서대로 실행한다.

    각 모델을 같은 조건으로 학습한 뒤 best.pt로 지정 split을 평가하고,
    모델별 지표와 비교 그래프를 저장한다. 정상 완료 시 0을 반환한다.
    """
    args = parse_args()

    # Ultralytics에 잘못된 범위가 전달되기 전에 사용자 입력을 검사한다.
    if args.patience < 0:
        raise SystemExit("--patience는 0 이상이어야 합니다.")
    if not 0.0 <= args.scale <= 1.0:
        raise SystemExit("--scale은 0.0 이상 1.0 이하여야 합니다.")
    if not 0.0 <= args.multi_scale <= 0.9:
        raise SystemExit("--multi-scale은 0.0 이상 0.9 이하여야 합니다.")
    data_yaml = validate_data_yaml(args.data)
    model_paths = [
        require_file(model_path, "초기 모델") for model_path in args.models
    ]
    runs_dir = args.runs_dir.expanduser().resolve()
    runs_dir.mkdir(parents=True, exist_ok=True)

    # --help처럼 학습을 실행하지 않는 경우 불필요한 의존성 로드를 피하고,
    # 실제 실행 시에는 설치 방법이 포함된 오류를 보여준다.
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "ultralytics가 설치되지 않았습니다. 가상환경에서 "
            "'python3 -m pip install -r requirements.txt'를 실행하세요."
        ) from exc

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results: list[tuple[str, Path, Path]] = []
    model_metrics = []

    from scripts.model_metrics import (
        average_prediction_confidence,
        build_model_metrics,
        generate_key_metrics_plots,
        resolve_eval_source,
        write_metrics_reports,
    )

    # 모든 모델의 평균 confidence를 동일한 이미지 집합에서 계산한다.
    eval_source, evaluated_images = resolve_eval_source(
        data_yaml, args.eval_split
    )

    for model_path in model_paths:
        # 모델마다 독립된 폴더를 사용해 가중치와 그래프가 섞이지 않게 한다.
        model_name = model_path.stem
        train_name = f"{model_name}_{timestamp}"
        print(f"\n{'=' * 64}")
        print(f"[학습 시작] {model_name}")
        print(f"학습 데이터: {data_yaml}")
        print(f"초기 모델:   {model_path}")
        print(
            "조기 종료:   "
            + (
                f"{args.patience} epoch 동안 개선이 없으면 종료"
                if args.patience > 0
                else "비활성화"
            )
        )

        model = YOLO(str(model_path))
        train_options = {
            "data": str(data_yaml),
            "epochs": args.epochs,
            "imgsz": args.imgsz,
            "batch": args.batch,
            "workers": args.workers,
            "patience": args.patience,
            "amp": args.amp,
            "project": str(runs_dir),
            "name": train_name,
            "seed": 42,
            "deterministic": args.deterministic,
            "plots": args.plots,
            "scale": args.scale,
            "multi_scale": args.multi_scale,
        }
        if args.device is not None:
            train_options["device"] = args.device
        # 순수 학습 호출 시간을 측정해 모델 크기별 소요 시간을 비교한다.
        training_started = perf_counter()
        model.train(**train_options)
        training_time_seconds = perf_counter() - training_started

        run_dir = runs_dir / train_name
        best_weights = require_file(
            run_dir / "weights" / "best.pt", f"{model_name} best.pt"
        )
        val_name = f"{train_name}_val"
        val_options = {
            "data": str(data_yaml),
            "imgsz": args.imgsz,
            "batch": args.batch,
            "project": str(runs_dir),
            "name": val_name,
            "plots": args.plots,
        }
        if args.device is not None:
            val_options["device"] = args.device
        # 마지막 epoch가 아니라 검증 성능이 가장 좋았던 best.pt를 평가한다.
        best_model = YOLO(str(best_weights))
        val_options["split"] = args.eval_split
        val_results = best_model.val(**val_options)
        average_confidence, predicted_boxes = average_prediction_confidence(
            best_model,
            eval_source,
            args.imgsz,
            args.device,
        )
        model_metrics.append(
            build_model_metrics(
                model_name=model_name,
                initial_weights=model_path,
                data_yaml=data_yaml,
                eval_split=args.eval_split,
                epochs_requested=args.epochs,
                training_time_seconds=training_time_seconds,
                imgsz=args.imgsz,
                batch=args.batch,
                patience=args.patience,
                device=args.device,
                best_weights=best_weights,
                train_run_dir=run_dir,
                val_run_dir=runs_dir / val_name,
                val_results=val_results,
                average_confidence=average_confidence,
                predicted_boxes=predicted_boxes,
                evaluated_images=evaluated_images,
            )
        )
        results.append((model_name, best_weights, runs_dir / val_name))

    print("\n학습 및 검증 완료")
    for model_name, best_weights, val_dir in results:
        print(f"- {model_name}: {best_weights}")
        print(f"  검증 결과: {val_dir}")
        if args.plots:
            train_dir = best_weights.parent.parent
            print("  주요 학습 플롯:")
            for plot_name in (
                "results.png",
                "confusion_matrix.png",
                "confusion_matrix_normalized.png",
                "train_batch0.jpg",
                "val_batch0_labels.jpg",
                "val_batch0_pred.jpg",
            ):
                plot_path = train_dir / plot_name
                if plot_path.is_file():
                    print(f"    - {plot_path}")
            print("  주요 검증 플롯:")
            for plot_name in (
                "confusion_matrix.png",
                "confusion_matrix_normalized.png",
                "val_batch0_labels.jpg",
                "val_batch0_pred.jpg",
            ):
                plot_path = val_dir / plot_name
                if plot_path.is_file():
                    print(f"    - {plot_path}")

    if args.plots:
        from scripts.plot_model_comparison import generate_comparison_plots

        comparison_dir = runs_dir / f"comparison_{timestamp}"
        comparison_plots = generate_comparison_plots(
            [
                (model_name, best_weights.parent.parent)
                for model_name, best_weights, _ in results
            ],
            comparison_dir,
        )
        print("\n모델 간 비교 그래프:")
        for plot_path in comparison_plots:
            print(f"- {plot_path}")

    metrics_dir = runs_dir / f"metrics_{timestamp}"
    metrics_csv, metrics_json = write_metrics_reports(
        model_metrics, metrics_dir
    )
    key_metrics_plots = (
        generate_key_metrics_plots(model_metrics, metrics_dir)
        if args.plots
        else []
    )
    print("\n핵심 비교 지표:")
    print(f"- CSV:  {metrics_csv}")
    print(f"- JSON: {metrics_json}")
    for item in model_metrics:
        print(
            f"- {item.model}: "
            f"mAP50={item.map50:.4f}, "
            f"P={item.precision:.4f}, R={item.recall:.4f}, "
            f"F1={item.best_f1:.4f}@conf={item.best_f1_confidence:.3f}, "
            f"학습={item.epochs_completed} epoch/"
            f"{item.training_time_seconds:.1f}s, "
            f"추론={item.inference_ms_per_image:.2f}ms/"
            f"{item.inference_fps:.1f}FPS, "
            f"평균 conf={item.average_confidence:.4f}"
        )
    for plot_path in key_metrics_plots:
        print(f"- 그래프: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
