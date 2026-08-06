#!/usr/bin/env python3
"""녹화한 영상에 YOLO 검출 상자를 그려 넣는다.

시연 화면에 띄울 영상은 검출 상자가 보여야 한다. 실시간으로 돌리면 시연
때마다 GPU 를 쓰고 결과도 매번 달라지므로, 한 번 돌려 파일로 떨궈 둔다.

aed_vision 의 파인튜닝 모델(rescue_yolo11n.pt)을 쓴다. 그 모델이 아는 것은
쓰러진 사람(fallen_person)과 helper(빨간 RC카)다. 화면 표기도 vision_detector
의 debug 영상과 같은 형태로 맞춘다. 시연에서 두 화면이 같은 것으로 보여야
한다.

사용:
  python3 tools/annotate_video.py docs/videos/robot1_oakd_demo.mp4
  python3 tools/annotate_video.py <입력> --out <출력> --conf 0.25
"""
import argparse
import os
import sys
import time

import cv2

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_WEIGHTS = os.path.join(
    REPO_ROOT, "src", "aed_vision", "models", "rescue_yolo11n.pt"
)

# 가중치 안의 클래스 이름은 helper_rc_car 인데, vision_detector 는 화면에
# helper 로 줄여 쓴다(DISPLAY_NAMES). 두 화면이 같아 보여야 하므로 여기서도
# 같게 줄인다.
DISPLAY_NAMES = {
    "fallen_person": "fallen_person",
    "helper_rc_car": "helper",
}

# vision_detector 의 debug 영상과 같은 색을 쓴다. BGR 이다.
COLOURS = {
    "fallen_person": (255, 100, 60),   # 파랑 계열
    "helper": (230, 220, 60),          # 청록 계열
}
DEFAULT_COLOUR = (0, 200, 255)


def draw(frame, boxes, names, fps_text: str) -> None:
    fallen = helper = 0
    for box in boxes:
        raw = names[int(box.cls[0])]
        cls = DISPLAY_NAMES.get(raw, raw)
        conf = float(box.conf[0])
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
        colour = COLOURS.get(cls, DEFAULT_COLOUR)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        label = f"{cls} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), colour, -1)
        cv2.putText(frame, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1)
        if cls == "fallen_person":
            fallen += 1
        elif cls == "helper":
            helper += 1

    # 상단 띠. vision_detector 의 debug 화면과 같은 정보를 같은 자리에 둔다.
    width = frame.shape[1]
    cv2.rectangle(frame, (0, 0), (width, 24), (0, 0, 0), -1)
    cv2.putText(frame, f"fallen: {fallen} | helper: {helper} | {fps_text}",
                (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 220, 255), 1)


def main() -> int:
    parser = argparse.ArgumentParser(description="영상에 YOLO 검출 상자 그리기")
    parser.add_argument("video")
    parser.add_argument("--out", default=None)
    parser.add_argument("--weights", default=DEFAULT_WEIGHTS)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()

    if not os.path.exists(args.video):
        print(f"실패: {args.video} 없음")
        return 1
    if not os.path.exists(args.weights):
        print(f"실패: 가중치 없음 {args.weights}")
        return 1
    out_path = args.out or args.video.replace(".mp4", "_yolo.mp4")

    from ultralytics import YOLO

    model = YOLO(args.weights)
    names = model.names
    print(f"모델 클래스: {names}")

    capture = cv2.VideoCapture(args.video)
    if not capture.isOpened():
        print(f"실패: {args.video} 를 못 엶")
        return 1
    fps = capture.get(cv2.CAP_PROP_FPS) or 15.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

    writer = cv2.VideoWriter(
        out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        print(f"실패: {out_path} 를 못 엶")
        return 1

    print(f"입력: {args.video}  {width}x{height} {fps:.0f}fps {total}프레임")
    processed = 0
    detected_frames = 0
    started = time.time()
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        t0 = time.time()
        result = model.predict(
            frame, conf=args.conf, device=args.device, verbose=False
        )[0]
        elapsed_ms = (time.time() - t0) * 1000
        boxes = result.boxes
        if len(boxes):
            detected_frames += 1
        draw(frame, boxes, names, f"{elapsed_ms:.1f} ms")
        writer.write(frame)
        processed += 1
        if processed % 300 == 0:
            print(f"  {processed}/{total} 프레임", flush=True)

    capture.release()
    writer.release()
    took = time.time() - started
    print(f"저장: {out_path}")
    print(f"      {processed}프레임, 검출된 프레임 {detected_frames}개 "
          f"({detected_frames/max(processed,1)*100:.0f}%), 처리 {took:.0f}초")
    return 0


if __name__ == "__main__":
    sys.exit(main())
