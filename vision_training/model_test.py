#!/usr/bin/env python3
"""같은 영상에서 파인튜닝 구조 모델과 COCO YOLO11n person을 비교한다."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
from time import perf_counter

from yolo_train import REQUIRED_NAMES


ROOT = Path(__file__).resolve().parent
DEFAULT_FINETUNE_RUNS = ROOT / "finetune_runs"
DEFAULT_PERSON_WEIGHTS = ROOT / "models" / "yolo11n.pt"
DEFAULT_CAPTURE_DIR = ROOT / "test_captures"
RESCUE_WINDOW = "Fine-tuned Rescue Model"
PERSON_WINDOW = "YOLO11n COCO Person"
RESCUE_DISPLAY_NAMES = {0: "fallen_person", 1: "helper"}


def parse_args() -> argparse.Namespace:
    """모델 비교 테스트에 사용할 명령행 인자를 정의하고 반환한다.

    구조 모델과 COCO person 모델의 가중치, 영상 소스, 모델별 confidence,
    NMS IoU, 입력 크기와 캡처 저장 위치를 실행 시 변경할 수 있게 한다.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--weights",
        type=Path,
        default=None,
        help="파인튜닝 best.pt. 생략하면 최신 YOLO11n 파인튜닝 모델 사용",
    )
    parser.add_argument(
        "--person-weights",
        type=Path,
        default=DEFAULT_PERSON_WEIGHTS,
        help="인파 확인용 COCO YOLO11n 가중치",
    )
    parser.add_argument(
        "--source",
        default="2",
        help="카메라 번호, 영상 경로 또는 스트림 URL (기본: 2)",
    )
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--person-conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default=None, help="예: 0 또는 cpu (기본: 자동)")
    parser.add_argument(
        "--capture-dir",
        type=Path,
        default=DEFAULT_CAPTURE_DIR,
        help="S 키로 저장한 두 모델의 테스트 화면 폴더",
    )
    return parser.parse_args()


def find_finetuned_weights(weights: Path | None) -> Path:
    """지정된 구조 모델 또는 가장 최근 YOLO11n 파인튜닝 모델을 반환한다.

    Args:
        weights: 사용자가 ``--weights``로 지정한 best.pt 경로. None이면
            finetune_runs에서 최신 YOLO11n 결과를 자동으로 검색한다.

    Returns:
        존재 여부가 확인된 파인튜닝 가중치의 절대 경로.
    """
    if weights is not None:
        resolved = weights.expanduser().resolve()
        if not resolved.is_file():
            raise SystemExit(f"파인튜닝 모델을 찾지 못했습니다: {resolved}")
        return resolved
    candidates = list(
        DEFAULT_FINETUNE_RUNS.glob("yolo11n_*_hard_negative_*/weights/best.pt")
    )
    if not candidates:
        raise SystemExit(
            f"YOLO11n 파인튜닝 best.pt가 없습니다: {DEFAULT_FINETUNE_RUNS}\n"
            "먼저 yolo_finetune.py를 실행하거나 --weights를 지정하세요."
        )
    return max(candidates, key=lambda path: path.stat().st_mtime).resolve()


def require_file(path: Path, description: str) -> Path:
    """파일이 존재하는지 검사하고 정리된 절대 경로를 반환한다.

    파일이 없으면 ``description``을 포함한 메시지로 즉시 실행을 중단한다.
    """
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise SystemExit(f"{description}을 찾지 못했습니다: {resolved}")
    return resolved


def resolve_source(source: str) -> int | str:
    """입력 문자열을 OpenCV가 사용할 카메라 번호·영상 경로·URL로 변환한다.

    숫자 문자열은 카메라 장치 번호인 int로, URL은 원문 그대로, 나머지는
    존재 여부를 확인한 영상 파일의 절대 경로로 반환한다.
    """
    value = source.strip()
    if value.isdigit():
        return int(value)
    if "://" in value:
        return value
    path = Path(value).expanduser().resolve()
    if not path.exists():
        raise SystemExit(f"테스트 영상을 찾지 못했습니다: {path}")
    return str(path)


def normalize_names(names: object) -> list[str]:
    """모델 클래스 이름을 클래스 ID 순서의 문자열 목록으로 통일한다.

    Ultralytics 버전이나 모델에 따라 names가 dict 또는 list로 반환되는
    차이를 흡수하며, 알 수 없는 형식이면 클래스 검증을 위해 종료한다.
    """
    if isinstance(names, dict):
        return [str(names[key]) for key in sorted(names, key=lambda key: int(key))]
    if isinstance(names, (list, tuple)):
        return [str(name) for name in names]
    raise SystemExit(f"모델 클래스 형식이 올바르지 않습니다: {names}")


def put_status(frame, text: str) -> None:
    """추론 결과 영상 상단에 검출 개수와 처리시간 상태 표시줄을 그린다.

    Args:
        frame: OpenCV BGR 이미지. 이 함수가 해당 이미지를 직접 수정한다.
        text: 상태 표시줄에 출력할 문자열.
    """
    import cv2

    cv2.rectangle(frame, (0, 0), (frame.shape[1], 42), (0, 0, 0), -1)
    cv2.putText(
        frame,
        text,
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def main() -> int:
    """동일한 영상 프레임을 두 모델로 추론하여 별도 창에 실시간 표시한다.

    첫 창은 파인튜닝 구조 모델의 fallen_person/helper 검출 결과를,
    둘째 창은 기본 COCO YOLO11n의 person 검출 결과를 보여준다. S 키로 두
    화면을 동시에 저장할 수 있으며 Q 또는 ESC로 종료한다.
    """
    args = parse_args()

    # confidence와 IoU는 확률/비율 값이므로 유효 범위를 먼저 검사한다.
    for option, value in (("--conf", args.conf), ("--person-conf", args.person_conf), ("--iou", args.iou)):
        if not 0.0 <= value <= 1.0:
            raise SystemExit(f"{option}는 0 이상 1 이하여야 합니다.")
    if args.imgsz < 32:
        raise SystemExit("--imgsz는 32 이상이어야 합니다.")

    # 두 모델과 영상 소스를 추론 시작 전에 확인해 실행 중 오류를 줄인다.
    rescue_weights = find_finetuned_weights(args.weights)
    person_weights = require_file(args.person_weights, "COCO YOLO11n 모델")
    source = resolve_source(args.source)
    capture_dir = args.capture_dir.expanduser().resolve()
    capture_dir.mkdir(parents=True, exist_ok=True)

    try:
        import cv2
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "opencv-python 또는 ultralytics가 없습니다. "
            "'python3 -m pip install -r vision_training/requirements.txt'를 실행하세요."
        ) from exc

    # 구조 모델의 클래스 ID 의미가 학습 데이터와 같은지 확인한다.
    rescue_model = YOLO(str(rescue_weights))
    rescue_names = normalize_names(rescue_model.names)
    if rescue_names != REQUIRED_NAMES:
        raise SystemExit(
            f"구조 모델 클래스가 다릅니다. 모델={rescue_names}, 기대값={REQUIRED_NAMES}"
        )
    # COCO 모델에서는 다른 79개 클래스를 제외하고 person 클래스만 사용한다.
    person_model = YOLO(str(person_weights))
    person_names = normalize_names(person_model.names)
    if "person" not in person_names:
        raise SystemExit(f"COCO 모델에 person 클래스가 없습니다: {person_names}")
    person_class_id = person_names.index("person")

    # 하나의 VideoCapture에서 읽은 동일 프레임을 두 모델에 전달해야 두 결과를
    # 같은 시점과 구도로 공정하게 비교할 수 있다.
    camera = cv2.VideoCapture(source)
    if not camera.isOpened():
        raise SystemExit(f"카메라 또는 영상을 열 수 없습니다: {source}")
    cv2.namedWindow(RESCUE_WINDOW)
    cv2.namedWindow(PERSON_WINDOW)
    print(f"파인튜닝 구조 모델: {rescue_weights}")
    print(f"COCO person 모델: {person_weights}")
    print(f"소스: {source}")
    print("Q/ESC: 종료 | S: 두 모델의 현재 화면 저장")

    predict_options = {
        "iou": args.iou,
        "imgsz": args.imgsz,
        "verbose": False,
    }
    if args.device is not None:
        predict_options["device"] = args.device

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("[종료] 카메라 또는 영상 프레임을 더 읽을 수 없습니다.")
                break

            # 각 모델의 predict 호출 시간을 따로 측정해 화면에 밀리초로 표시한다.
            started = perf_counter()
            rescue_result = rescue_model.predict(
                frame, conf=args.conf, **predict_options
            )[0]
            rescue_ms = (perf_counter() - started) * 1000
            started = perf_counter()
            person_result = person_model.predict(
                frame,
                conf=args.person_conf,
                classes=[person_class_id],
                **predict_options,
            )[0]
            person_ms = (perf_counter() - started) * 1000

            # 학습 클래스명 helper_rc_car는 가중치 검증과 개수 집계에 유지하되,
            # 사용자 화면의 bbox에는 역할을 나타내는 helper로 짧게 표시한다.
            rescue_result.names = RESCUE_DISPLAY_NAMES

            # result.plot()은 원본 프레임을 보존한 채 bbox가 그려진 이미지를 만든다.
            rescue_frame = rescue_result.plot()
            person_frame = person_result.plot()
            rescue_counts = {name: 0 for name in REQUIRED_NAMES}
            if rescue_result.boxes is not None and rescue_result.boxes.cls is not None:
                for class_id in rescue_result.boxes.cls.int().cpu().tolist():
                    rescue_counts[rescue_names[class_id]] += 1
            person_count = (
                len(person_result.boxes)
                if person_result.boxes is not None
                else 0
            )
            put_status(
                rescue_frame,
                f"fallen: {rescue_counts['fallen_person']} | "
                f"helper: {rescue_counts['helper_rc_car']} | {rescue_ms:.1f} ms",
            )
            put_status(
                person_frame,
                f"COCO person: {person_count} | {person_ms:.1f} ms",
            )
            cv2.imshow(RESCUE_WINDOW, rescue_frame)
            cv2.imshow(PERSON_WINDOW, person_frame)
            # 두 창이 표시된 상태에서 공통 키 입력을 한 번만 처리한다.
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                rescue_output = capture_dir / f"rescue_{timestamp}.jpg"
                person_output = capture_dir / f"person_{timestamp}.jpg"
                cv2.imwrite(str(rescue_output), rescue_frame)
                cv2.imwrite(str(person_output), person_frame)
                print(f"[저장] {rescue_output}")
                print(f"[저장] {person_output}")
    finally:
        # 정상 종료와 예외 발생 모두에서 카메라 장치와 GUI 창을 해제한다.
        camera.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
