#!/usr/bin/env python3
"""구조 상황 학습용 이미지를 하나의 원본 폴더에 촬영한다."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = ROOT / "captures" / "raw"
CAPTURE_KEY = ord(" ")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
WINDOW_NAME = "Rescue Dataset Capture"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=2, help="카메라 번호 (기본: 2)")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="촬영한 원본 이미지를 저장할 폴더",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    image_count = sum(
        path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        for path in output_dir.iterdir()
    )

    try:
        import cv2
    except ImportError as exc:
        raise SystemExit(
            "opencv-python이 없습니다. "
            "'python3 -m pip install -r vision_training/requirements.txt'를 실행하세요."
        ) from exc

    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        raise SystemExit(
            f"카메라 {args.camera}을 열 수 없습니다. --camera 0처럼 바꿔 보세요."
        )

    print("마우스 왼쪽 클릭/Space: 현재 프레임 저장 | Q/ESC: 종료")
    print(f"저장 위치: {output_dir}")

    capture_requested = False

    def request_capture(event: int, _x: int, _y: int, _flags: int, _param: object) -> None:
        nonlocal capture_requested
        if event == cv2.EVENT_LBUTTONDOWN:
            capture_requested = True

    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, request_capture)

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("[오류] 카메라 프레임을 읽지 못했습니다.")
                return 1

            preview = frame.copy()
            cv2.putText(
                preview,
                "LEFT CLICK / SPACE: capture | Q/ESC: quit",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                preview,
                f"captured: {image_count}",
                (15, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key != CAPTURE_KEY and not capture_requested:
                continue

            capture_requested = False
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_path = output_dir / f"capture_{timestamp}.jpg"
            if cv2.imwrite(str(image_path), frame):
                image_count += 1
                print(f"[저장] {image_path}")
            else:
                print(f"[저장 실패] {image_path}")
    finally:
        camera.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
