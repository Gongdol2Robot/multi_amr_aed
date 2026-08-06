#!/usr/bin/env python3
"""시연용 mp4 에서 프레임별 검출 수를 뽑아 사이드카 JSON 으로 떨군다.

목업 화면에서 영상만 틀면, 화면에는 검출 상자가 보이는데 관제 쪽 검출
표시는 0 인 상태가 된다. 시연에서 그 어긋남이 제일 먼저 눈에 띈다.
그렇다고 목업이 돌 때마다 YOLO 를 다시 돌리면 GPU 를 쓰고 결과도 매번
달라진다. 그래서 한 번 세어 파일로 남기고, 목업은 그 파일을 읽는다.

세는 방법이 둘인 이유
---------------------
영상마다 손에 쥔 것이 다르다.

* 로봇 시점(`--model`): 상자가 없는 원본 영상이 있다. 모델을 돌려 센다.
* 고정 웹캠(`--boxes`): rosbag 에 담긴 것이 vision_detector 의 debug
  영상뿐이라 원본이 없다. 이미 그려진 상자를 되읽는다.

debug 영상에 모델을 다시 돌리면 안 된다. 그려진 상자와 라벨이 그 자체로
검출을 만들어, 화면에 하나뿐인 장면이 서너 개로 세어진다.

`--boxes` 는 클래스 색으로 가른다. fallen_person 은 파랑, helper 는
청록이다. 색이 겹치는 것은 alley 카메라가 늘 그리는 혼잡 ROI 뿐인데,
그건 helper 와 같은 청록이라 색으로는 안 갈린다.

그래서 색 마스크를 한 겹 깎은 뒤(erode) 센다. 2px 짜리 테두리와 ROI
선은 사라지고 라벨 배경만 남는다. 깎지 않으면 조각난 ROI 선이 저마다
한 건으로 세어져, 검출 하나짜리 장면이 네 건이 된다.

사용:
  python3 tools/scan_detections.py --boxes docs/videos/camera_open_demo.mp4
  python3 tools/scan_detections.py --model docs/videos/robot1_oakd_demo.mp4
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_WEIGHTS = os.path.join(
    REPO_ROOT, "src", "aed_vision", "models", "rescue_yolo11n.pt"
)

CLASSES = ("fallen_person", "helper")

# ultralytics 기본 팔레트. 클래스 0 은 파랑, 클래스 1 은 청록이다. BGR.
BOX_COLOURS = {
    "fallen_person": lambda b, g, r: (b > 180) & (g < 120) & (r < 100),
    "helper": lambda b, g, r: (b > 180) & (g > 160) & (r < 120),
}

BOX_MIN_AREA = 400          # 이보다 작으면 압축 잡티
BOX_MAX_SPAN_RATIO = 0.55   # 화면의 이만큼을 넘게 걸치면 혼잡 ROI 다

# 상자를 세는 실마리는 라벨 배경이다. "fallen_person 0.82" 가 들어가는
# 속이 찬 칸이라 두껍다. 상자 테두리와 ROI 선은 2px 남짓이라 한 겹 깎으면
# 사라진다. 테두리만 세면 조각난 ROI 선이 여러 개로 잡힌다.
LABEL_ERODE = np.ones((3, 3), np.uint8)


def count_from_boxes(frame: np.ndarray) -> dict:
    height, width = frame.shape[:2]
    b = frame[:, :, 0].astype(np.int16)
    g = frame[:, :, 1].astype(np.int16)
    r = frame[:, :, 2].astype(np.int16)

    counts = {}
    for name, predicate in BOX_COLOURS.items():
        mask = predicate(b, g, r).astype(np.uint8)
        # 얇은 선을 지우고 라벨 배경만 남긴다.
        mask = cv2.erode(mask, LABEL_ERODE)
        found, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        boxes = 0
        for i in range(1, found):
            _, _, w, h, area = stats[i]
            if area < BOX_MIN_AREA:
                continue
            if (w > width * BOX_MAX_SPAN_RATIO
                    and h > height * BOX_MAX_SPAN_RATIO):
                continue
            boxes += 1
        counts[name] = boxes
    return counts


def count_from_model(model, frame, conf: float, device: str) -> dict:
    boxes = model.predict(frame, conf=conf, device=device, verbose=False)[0].boxes
    names = model.names
    counts = {name: 0 for name in CLASSES}
    for box in boxes:
        label = names[int(box.cls[0])]
        if label in counts:
            counts[label] += 1
    return counts


def scan(path: str, step: int, counter) -> dict:
    capture = cv2.VideoCapture(path)
    if not capture.isOpened():
        raise SystemExit(f"실패: {path} 를 못 엶")
    fps = capture.get(cv2.CAP_PROP_FPS) or 15.0

    samples = []
    index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if index % step == 0:
            counts = counter(frame)
            samples.append([
                round(index / fps, 3),
                counts["fallen_person"],
                counts["helper"],
            ])
        index += 1
    capture.release()

    detected = sum(1 for s in samples if s[1] or s[2])
    return {
        "video": os.path.basename(path),
        "fps": round(fps, 3),
        "frames": index,
        "duration_s": round(index / fps, 2),
        "step_frames": step,
        "columns": ["time_s", "fallen_person", "helper"],
        "samples": samples,
        "detected_ratio": round(detected / max(len(samples), 1), 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="영상에서 검출 수 뽑기")
    parser.add_argument("videos", nargs="+")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--boxes", action="store_true",
                      help="이미 그려진 상자를 되읽는다(debug 영상용)")
    mode.add_argument("--model", action="store_true",
                      help="모델을 돌려 센다(원본 영상용)")
    parser.add_argument("--weights", default=DEFAULT_WEIGHTS)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--out-name", default=None,
                        help="사이드카를 다른 이름으로 저장. 원본으로 세고 "
                             "상자 그린 영상에 붙일 때 쓴다")
    parser.add_argument(
        "--step", type=int, default=3,
        help="몇 프레임마다 셀지. 기본 3(15fps 에서 0.2초)"
    )
    args = parser.parse_args()

    if args.model:
        if not os.path.exists(args.weights):
            print(f"실패: 가중치 없음 {args.weights}")
            return 1
        from ultralytics import YOLO
        model = YOLO(args.weights)

        def counter(frame):
            return count_from_model(model, frame, args.conf, args.device)
    else:
        counter = count_from_boxes

    if args.out_name and len(args.videos) > 1:
        print("실패: --out-name 은 영상 하나에만 씁니다")
        return 1

    for path in args.videos:
        if not os.path.exists(path):
            print(f"건너뜀: {path} 없음")
            continue
        started = time.time()
        result = scan(path, args.step, counter)
        base = args.out_name or os.path.splitext(os.path.basename(path))[0]
        out_path = os.path.join(os.path.dirname(path),
                                base + ".detections.json")
        with open(out_path, "w", encoding="utf-8") as handle:
            json.dump(result, handle)
        print(f"{os.path.basename(path):<32} "
              f"{result['frames']:>5}프레임 {result['duration_s']:>6.1f}초  "
              f"검출 구간 {result['detected_ratio'] * 100:>3.0f}%  "
              f"({time.time() - started:.0f}초) -> {os.path.basename(out_path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
