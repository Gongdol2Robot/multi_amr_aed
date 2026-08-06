#!/usr/bin/env python3
"""호모그래피 대응점의 픽셀 좌표를 찍는 도구.

두 가지 방식을 지원한다.

A) 라이브 모드 — 바닥에 표시해 둔 기준점을 화면에서 직접 찍는다.
     python3 tools/pick_pixels.py
   SPACE로 프레임을 고정하고 기준점을 순서대로 클릭한 뒤 ENTER를 누른다.
   창이 닫히면 각 점의 map 좌표를 터미널에서 입력한다.

B) 채우기 모드 — survey_point.py로 모아 둔 이미지에서 로봇 바닥 중심을 찍어
   points.csv의 빈 pixel_u, pixel_v 열을 채운다.
     python3 tools/pick_pixels.py --fill

조작:
  클릭     점 추가          u        마지막 점 취소
  ENTER    확정             q / ESC  취소하고 종료
  s        (채우기 모드) 이 이미지 건너뛰기

기준점은 모두 바닥 평면 위에 있어야 하고, 한 직선 위에 몰리면 안 된다.
화면에서 넓게 벌어질수록 오차가 작다.

호모그래피는 해상도에 종속된다. 여기서 찍은 해상도와 webcam_publisher가
발행하는 해상도가 같아야 한다. 기본값은 둘 다 640x480이다.

환경변수: CAM_INDEX(기본 2), CAM_WIDTH(640), CAM_HEIGHT(480)
결과: tools/survey/points.csv
"""
import csv
import os
import sys

import cv2

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "tools", "survey")
CSV_PATH = os.path.join(OUT, "points.csv")
CAM_INDEX = int(os.environ.get("CAM_INDEX", "2"))
CAM_WIDTH = int(os.environ.get("CAM_WIDTH", "640"))
CAM_HEIGHT = int(os.environ.get("CAM_HEIGHT", "480"))

HEADER = ["label", "map_x", "map_y", "yaw_deg", "pixel_u", "pixel_v"]
ZOOM_HALF = 20  # 확대경이 보여주는 반경(픽셀)
ZOOM_SCALE = 6


def open_camera():
    capture = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(CAM_INDEX)
    if not capture.isOpened():
        return None
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def _zoom_inset(view, frame, cursor):
    """커서 주변을 확대해 구석에 붙인다. 테이프 모서리를 픽셀 단위로 찍기 위함."""
    u, v = cursor
    height, width = frame.shape[:2]
    x1, y1 = u - ZOOM_HALF, v - ZOOM_HALF
    patch = cv2.copyMakeBorder(
        frame, ZOOM_HALF, ZOOM_HALF, ZOOM_HALF, ZOOM_HALF, cv2.BORDER_REPLICATE
    )[y1 + ZOOM_HALF:y1 + 3 * ZOOM_HALF + 1,
      x1 + ZOOM_HALF:x1 + 3 * ZOOM_HALF + 1]
    if patch.size == 0:
        return
    side = (2 * ZOOM_HALF + 1) * ZOOM_SCALE
    zoom = cv2.resize(patch, (side, side), interpolation=cv2.INTER_NEAREST)
    center = side // 2
    cv2.line(zoom, (center, 0), (center, side), (0, 255, 255), 1)
    cv2.line(zoom, (0, center), (side, center), (0, 255, 255), 1)
    cv2.rectangle(zoom, (0, 0), (side - 1, side - 1), (255, 255, 255), 1)

    # 커서가 있는 쪽 반대편 구석에 둔다
    ox = 8 if u > width // 2 else width - side - 8
    oy = 8 if v > height // 2 else height - side - 8
    if ox < 0 or oy < 0 or ox + side > width or oy + side > height:
        return
    view[oy:oy + side, ox:ox + side] = zoom


def _render(frame, points, cursor, hint):
    view = frame.copy()
    for index, (u, v) in enumerate(points):
        cv2.drawMarker(view, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.circle(view, (u, v), 10, (0, 0, 255), 1)
        cv2.putText(
            view, str(index + 1), (u + 12, v - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA
        )
    if len(points) >= 2:
        for index in range(len(points) - 1):
            cv2.line(view, points[index], points[index + 1], (0, 180, 255), 1)
    if cursor is not None:
        _zoom_inset(view, frame, cursor)
    cv2.putText(
        view, hint, (8, view.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA
    )
    cv2.putText(
        view, hint, (8, view.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
    )
    return view


def pick_points(frame, title, limit=None, allow_skip=False):
    """클릭한 점들을 반환한다. 취소는 None, 건너뛰기는 빈 리스트."""
    state = {"points": [], "cursor": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            state["cursor"] = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            if limit is None or len(state["points"]) < limit:
                state["points"].append((x, y))

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, frame.shape[1], frame.shape[0])
    cv2.setMouseCallback(title, on_mouse)
    try:
        while True:
            need = "" if limit is None else f"/{limit}"
            hint = (
                f"{len(state['points'])}{need} points | "
                f"click=add  u=undo  ENTER=done"
                + ("  s=skip" if allow_skip else "")
                + "  q=quit"
            )
            cv2.imshow(title, _render(
                frame, state["points"], state["cursor"], hint
            ))
            key = cv2.waitKey(20) & 0xFF
            if key in (13, 10):  # ENTER
                return state["points"]
            if key == ord("u") and state["points"]:
                state["points"].pop()
            elif key == ord("s") and allow_skip:
                return []
            elif key in (ord("q"), 27):  # q, ESC
                return None
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                return None
    finally:
        cv2.destroyWindow(title)
        cv2.waitKey(1)


def read_rows():
    if not os.path.exists(CSV_PATH):
        return []
    with open(CSV_PATH, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_rows(rows):
    os.makedirs(OUT, exist_ok=True)
    with open(CSV_PATH, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=HEADER)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in HEADER})


def ask_map_xy(label):
    while True:
        raw = input(f"  {label} 의 map 좌표 (x y): ").replace(",", " ").split()
        if len(raw) != 2:
            print("    x와 y 두 개를 공백으로 구분해 입력하세요. 예: 1.25 -0.40")
            continue
        try:
            return float(raw[0]), float(raw[1])
        except ValueError:
            print("    숫자로 입력하세요.")


def live_mode():
    capture = open_camera()
    if capture is None:
        print(f"실패: /dev/video{CAM_INDEX} 를 못 엶")
        return 1

    title = f"live /dev/video{CAM_INDEX} - SPACE=freeze  q=quit"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    frozen = None
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                print("실패: 프레임을 못 읽음")
                return 1
            cv2.imshow(title, frame)
            key = cv2.waitKey(20) & 0xFF
            if key == 32:  # SPACE
                frozen = frame.copy()
                break
            if key in (ord("q"), 27):
                return 1
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                return 1
    finally:
        capture.release()
        cv2.destroyWindow(title)
        cv2.waitKey(1)

    height, width = frozen.shape[:2]
    if (width, height) != (CAM_WIDTH, CAM_HEIGHT):
        print(f"주의: 카메라가 {width}x{height} 로 잡혔습니다 "
              f"(요청 {CAM_WIDTH}x{CAM_HEIGHT}). "
              f"webcam_publisher도 같은 해상도로 띄워야 합니다.")

    os.makedirs(OUT, exist_ok=True)
    frame_path = os.path.join(OUT, "frame.jpg")
    cv2.imwrite(frame_path, frozen)

    points = pick_points(frozen, "pick reference points")
    if points is None:
        print("취소됨")
        return 1
    if len(points) < 4:
        print(f"점이 {len(points)}개뿐입니다. 호모그래피에는 최소 4점이 필요합니다.")
        return 1

    print()
    print(f"{len(points)}개 점을 찍었습니다. 각 점의 map 좌표를 입력하세요.")
    print("(로봇을 그 자리에 세우고 tools/survey_point.py로 읽거나, "
          "RViz에서 해당 지점을 읽습니다)")
    rows = read_rows()
    existing = {row["label"] for row in rows}
    for index, (u, v) in enumerate(points, start=1):
        label = f"p{index}"
        while label in existing:
            label += "_"
        map_x, map_y = ask_map_xy(f"{label} (pixel {u},{v})")
        rows.append({
            "label": label,
            "map_x": f"{map_x:.4f}",
            "map_y": f"{map_y:.4f}",
            "yaw_deg": "",
            "pixel_u": str(u),
            "pixel_v": str(v),
        })
        existing.add(label)

    write_rows(rows)
    print()
    print(f"저장: {CSV_PATH}")
    print(f"프레임: {frame_path}  ({width}x{height})")
    print("다음: python3 tools/fit_homography.py")
    return 0


def fill_mode():
    rows = read_rows()
    if not rows:
        print(f"실패: {CSV_PATH} 가 없습니다. 먼저 survey_point.py 를 실행하세요.")
        return 1

    pending = [row for row in rows if not row.get("pixel_u")]
    if not pending:
        print("빈 pixel 열이 없습니다. 바로 fit_homography.py 를 실행하면 됩니다.")
        return 0

    print(f"{len(pending)}개 점의 픽셀을 채웁니다. "
          f"각 이미지에서 로봇 바닥 원의 중심을 클릭하세요.")
    for row in pending:
        label = row["label"]
        image_path = os.path.join(OUT, f"{label}.jpg")
        if not os.path.exists(image_path):
            print(f"  {label}: 이미지 없음 ({image_path}) — 건너뜀")
            continue
        image = cv2.imread(image_path)
        if image is None:
            print(f"  {label}: 이미지를 못 읽음 — 건너뜀")
            continue
        picked = pick_points(
            image, f"{label}  (map {row['map_x']}, {row['map_y']})",
            limit=1, allow_skip=True
        )
        if picked is None:
            print("취소됨 — 여기까지 저장합니다.")
            break
        if not picked:
            print(f"  {label}: 건너뜀")
            continue
        u, v = picked[0]
        row["pixel_u"], row["pixel_v"] = str(u), str(v)
        print(f"  {label}: pixel=({u}, {v})")

    write_rows(rows)
    done = sum(1 for row in rows if row.get("pixel_u"))
    print()
    print(f"저장: {CSV_PATH}  (픽셀이 채워진 점 {done}개)")
    if done >= 4:
        print("다음: python3 tools/fit_homography.py")
    else:
        print(f"호모그래피에는 4점이 필요합니다. {4 - done}개 더 필요합니다.")
    return 0


def main():
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("실패: 화면이 없습니다. 데스크톱 터미널에서 실행하세요.")
        return 1
    if "--fill" in sys.argv[1:]:
        return fill_mode()
    return live_mode()


if __name__ == "__main__":
    sys.exit(main())
