#!/usr/bin/env python3
"""1차 YOLO 6종 best.pt를 양성+hard negative 데이터로 파인튜닝한다."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

from yolo_train import REQUIRED_NAMES, validate_data_yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA = ROOT / "data.yaml"
DEFAULT_BASE_RUNS = ROOT / "runs"
DEFAULT_RUNS = ROOT / "finetune_runs"
MODEL_NAMES = ("yolov8n", "yolov8s", "yolo11n", "yolo11s", "yolo26n", "yolo26s")


def parse_args() -> argparse.Namespace:
    """파인튜닝에 사용할 명령행 인자를 정의하고 파싱해 반환한다.

    --weights를 생략하면 1차 학습 모델 6종의 최신 best.pt를 자동으로 찾는다.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--weights",
        type=Path,
        nargs="+",
        default=None,
        help="1차 학습 best.pt 목록. 생략하면 1차 모델 6종의 최신 파일 사용",
    )
    parser.add_argument(
        "--base-runs-dir",
        type=Path,
        default=DEFAULT_BASE_RUNS,
        help="--weights 생략 시 best.pt를 검색할 1차 학습 결과 폴더",
    )
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--epochs", type=int, default=25)
    parser.add_argument("--lr0", type=float, default=0.0001)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default=None, help="예: 0 또는 cpu (기본: 자동)")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--patience", type=int, default=10)
    parser.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS)
    parser.add_argument(
        "--plots",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser.parse_args()


def find_latest_best_by_model(runs_dir: Path) -> list[Path]:
    """1차 학습 결과에서 모델 종류별 최신 best.pt를 한 개씩 찾는다.

    Args:
        runs_dir: 1차 학습 결과 폴더. 각 실행 폴더는 모델명으로 시작해야 한다.

    Returns:
        ``MODEL_NAMES`` 순서로 정렬된 6개 best.pt 절대 경로.

    Raises:
        SystemExit: 한 종류라도 학습 가중치가 발견되지 않은 경우.
    """
    runs_dir = runs_dir.expanduser().resolve()
    result = []
    missing = []
    for model_name in MODEL_NAMES:
        candidates = list(runs_dir.glob(f"{model_name}_*/weights/best.pt"))
        if not candidates:
            missing.append(model_name)
            continue
        result.append(max(candidates, key=lambda path: path.stat().st_mtime).resolve())
    if missing:
        raise SystemExit(
            f"1차 학습 best.pt가 없는 모델: {', '.join(missing)}\n"
            f"검색 위치: {runs_dir}/<모델명>_*/weights/best.pt\n"
            "6종 1차 학습을 완료하거나 --weights로 사용할 파일만 지정하세요."
        )
    return result


def normalize_names(names: object) -> list[str]:
    """Ultralytics 모델의 클래스 이름을 순서가 보존된 문자열 목록으로 바꾼다.

    Ultralytics 버전에 따라 names가 딕셔너리 또는 리스트로 제공될 수 있어
    두 형식을 하나로 통일한다. 지원하지 않는 형식이면 실행을 중단한다.
    """
    if isinstance(names, dict):
        return [str(names[key]) for key in sorted(names, key=lambda key: int(key))]
    if isinstance(names, (list, tuple)):
        return [str(name) for name in names]
    raise SystemExit(f"모델 클래스 이름 형식이 올바르지 않습니다: {names}")


def main() -> int:
    """1차 best.pt들을 양성+hard negative 데이터로 파인튜닝하고 평가한다.

    각 모델의 클래스 구성을 확인한 뒤 낮은 학습률로 추가 학습하며, 새로 생성된
    best.pt를 독립된 test split으로 평가한다. 정상 완료 시 0을 반환한다.
    """
    args = parse_args()

    # 잘못된 수치 설정으로 긴 학습이 시작되는 것을 사전에 방지한다.
    if args.epochs < 1 or args.batch < 1 or args.workers < 0 or args.patience < 0:
        raise SystemExit("epochs/batch는 1 이상, workers/patience는 0 이상이어야 합니다.")
    if args.lr0 <= 0:
        raise SystemExit("--lr0는 0보다 커야 합니다.")

    # data.yaml에는 기존 양성 이미지와 새 hard negative 이미지가 함께 연결된다.
    data_yaml = validate_data_yaml(args.data)
    # 명시한 가중치가 있으면 그대로 쓰고, 없으면 모델별 최신 1차 best.pt를 찾는다.
    weights_list = (
        [path.expanduser().resolve() for path in args.weights]
        if args.weights is not None
        else find_latest_best_by_model(args.base_runs_dir)
    )
    for weights in weights_list:
        if not weights.is_file():
            raise SystemExit(f"1차 학습 가중치를 찾지 못했습니다: {weights}")
    runs_dir = args.runs_dir.expanduser().resolve()
    runs_dir.mkdir(parents=True, exist_ok=True)

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "ultralytics가 없습니다. "
            "'python3 -m pip install -r vision_training/requirements.txt'를 실행하세요."
        ) from exc

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"파인튜닝 데이터: {data_yaml}")
    print(f"클래스: {REQUIRED_NAMES}")
    print(f"학습률: {args.lr0}, epoch: {args.epochs}")
    print(f"대상 모델: {len(weights_list)}개")
    completed = []
    for index, weights in enumerate(weights_list, start=1):
        model = YOLO(str(weights))

        # 클래스 순서가 다르면 기존 검출기의 의미가 뒤바뀌므로 학습을 중단한다.
        model_names = normalize_names(model.names)
        if model_names != REQUIRED_NAMES:
            raise SystemExit(
                f"1차 모델과 데이터 클래스가 다릅니다: {weights}\n"
                f"모델: {model_names}\n데이터: {REQUIRED_NAMES}"
            )

        run_name = f"{weights.parents[1].name}_hard_negative_{timestamp}"
        print(f"\n[{index}/{len(weights_list)}] {weights.parents[1].name}")
        print(f"1차 가중치: {weights}")
        # 1차 학습 결과를 보존하도록 낮은 lr0를 사용하고, 마지막 5 epoch에는
        # mosaic 증강을 꺼 실제 카메라 영상에 가까운 이미지로 마무리한다.
        options = {
            "data": str(data_yaml),
            "epochs": args.epochs,
            "lr0": args.lr0,
            "imgsz": args.imgsz,
            "batch": args.batch,
            "workers": args.workers,
            "patience": args.patience,
            "project": str(runs_dir),
            "name": run_name,
            "seed": 42,
            "deterministic": True,
            "plots": args.plots,
            "close_mosaic": 5,
        }
        if args.device is not None:
            options["device"] = args.device
        model.train(**options)

        best_weights = runs_dir / run_name / "weights" / "best.pt"
        if not best_weights.is_file():
            raise SystemExit(f"파인튜닝 best.pt가 없습니다: {best_weights}")
        test_name = f"{run_name}_test"
        val_options = {
            "data": str(data_yaml),
            "imgsz": args.imgsz,
            "batch": args.batch,
            "project": str(runs_dir),
            "name": test_name,
            "split": "test",
            "plots": args.plots,
        }
        if args.device is not None:
            val_options["device"] = args.device
        # 파인튜닝 중 사용한 val이 아닌 test split으로 최종 일반화 성능을 확인한다.
        YOLO(str(best_weights)).val(**val_options)
        completed.append((weights.parents[1].name, best_weights, runs_dir / test_name))

    print("\n6종 파인튜닝 및 test 평가 완료")
    for source_name, best_weights, test_dir in completed:
        print(f"- {source_name}: {best_weights}")
        print(f"  test 결과: {test_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
