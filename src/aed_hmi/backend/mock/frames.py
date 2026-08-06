"""ROS 없이 4분할 화면을 채울 가짜 영상.

네 갈래 모두 실제로 녹화한 영상을 재생한다. 정지 사진을 띄우면 화면이
멈춰 보여서 시연에서 "이거 진짜 도는 건가"라는 질문이 먼저 나온다.

  robot1 / robot2  — OAK-D 로 찍은 주행 영상. 한 파일을 두 타일이 쓰므로
                     시작 위치를 벌려 같은 그림이 나란히 뜨지 않게 한다.
  camera_open      — 고정 웹캠. 인형이 서 있다가 쓰러지고, vision_detector
                     가 fallen_person 을 잡는 데까지 담겨 있다.
  camera_alley     — 고정 웹캠. 혼잡도와 helper(빨간 RC카)까지 나온다.

영상에는 vision_detector 가 그린 검출 상자가 이미 들어 있다. 그런데 화면
쪽 검출 표시가 0 이면 그 어긋남이 바로 눈에 띈다. 그래서 영상 옆의
`*.detections.json`(tools/scan_detections.py 가 만든다)을 읽어, 재생 중인
그 프레임의 검출 수를 그대로 흘려보낸다. 화면의 상자와 관제의 숫자가
같은 것을 보게 된다.

영상이 없으면 사진으로, 사진도 없으면 단색으로 떨어진다. 저장소를 갓
받은 사람도 서버는 뜨는 편이 낫다.
"""

import json
import os
import threading
import time

import cv2
import numpy as np

REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)

VIDEO_DIR = "docs/videos"

# stream_id -> (영상 파일, 재생 시작 지점 비율)
VIDEO_SOURCES = {
    "robot1": ("robot1_oakd_yolo.mp4", 0.00),
    "robot2": ("robot1_oakd_yolo.mp4", 0.45),
    "camera_open": ("camera_open_demo.mp4", 0.00),
    "camera_alley": ("camera_alley_demo.mp4", 0.00),
}

# 영상이 없을 때 대신 띄울 사진. 호모그래피 측량 때 찍은 그 카메라 장면이다.
FALLBACK_IMAGES = {
    "camera_open": "tools/survey/cam1/camera_view.jpg",
    "camera_alley": "tools/survey/cam2/camera_view.jpg",
}

ROBOT_STREAMS = ("robot1", "robot2")
STREAM_ORDER = ("robot1", "robot2", "camera_open", "camera_alley")

FPS = 10.0

# 사진으로 떨어질 때만 쓰는 크기. 영상은 원본 해상도 그대로 내보낸다.
# OAK-D 는 704x704, 웹캠은 640x480 이라 한 크기로 맞추면 한쪽이 늘어난다.
# 화면 쪽은 타일에 맞춰 비율을 지켜 넣으므로 여기서 건드릴 이유가 없다.
STILL_SIZE = (640, 480)


class _DetectionTrack:
    """영상 시각 -> 그 시점의 검출 수.

    tools/scan_detections.py 가 남긴 사이드카를 읽는다. 표본은 몇 프레임에
    한 번이므로, 물어본 시각 이하의 가장 마지막 표본을 쓴다. 표본 사이에서
    검출 수가 튀는 것보다 직전 값을 유지하는 편이 화면에서 자연스럽다.
    """

    def __init__(self, path: str) -> None:
        self.times: list[float] = []
        self.counts: list[int] = []
        if not os.path.exists(path):
            return
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        for time_s, fallen, helper in data.get("samples", []):
            self.times.append(float(time_s))
            self.counts.append(int(fallen) + int(helper))

    def __bool__(self) -> bool:
        return bool(self.times)

    def at(self, seconds: float) -> int:
        if not self.times:
            return 0
        # 표본이 시각 순으로 정렬돼 있으므로 이분 탐색으로 찾는다.
        lo, hi = 0, len(self.times) - 1
        if seconds <= self.times[0]:
            return self.counts[0]
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.times[mid] <= seconds:
                lo = mid
            else:
                hi = mid - 1
        return self.counts[lo]


class _VideoLoop:
    """mp4 를 끝에서 처음으로 돌려가며 읽는다.

    지금 몇 초 지점을 재생 중인지 알아야 검출 사이드카를 맞춰 읽을 수
    있으므로, 프레임 번호를 직접 센다. CAP_PROP_POS_MSEC 을 매 프레임
    묻지 않는 것은 코덱에 따라 값이 튀기 때문이다.
    """

    def __init__(self, path: str, start_ratio: float = 0.0) -> None:
        self.capture = cv2.VideoCapture(path)
        self.ok = self.capture.isOpened()
        self.fps = 15.0
        self.total = 0
        self.index = 0
        if self.ok:
            self.fps = self.capture.get(cv2.CAP_PROP_FPS) or 15.0
            self.total = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
            if self.total > 0 and start_ratio > 0.0:
                self.index = int(self.total * start_ratio)
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, self.index)

    @property
    def position_s(self) -> float:
        return self.index / self.fps

    def read(self):
        if not self.ok:
            return None
        got, frame = self.capture.read()
        if got:
            self.index += 1
        else:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.index = 0
            got, frame = self.capture.read()
            if not got:
                return None
            self.index = 1
        return frame

    def release(self) -> None:
        if self.ok:
            self.capture.release()


def _placeholder(text: str) -> np.ndarray:
    image = np.full((STILL_SIZE[1], STILL_SIZE[0], 3), 40, np.uint8)
    cv2.putText(image, text, (150, 250), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (80, 80, 200), 2)
    return image


class MockFrameSource:
    """카메라 갈래마다 프레임을 만들어 context 로 밀어 넣는다."""

    def __init__(self, context) -> None:
        self._context = context
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self._videos: dict[str, _VideoLoop] = {}
        self._tracks: dict[str, _DetectionTrack] = {}
        self._stills: dict[str, np.ndarray] = {}

        for stream_id, (filename, start_ratio) in VIDEO_SOURCES.items():
            path = os.path.join(REPO_ROOT, VIDEO_DIR, filename)
            if not os.path.exists(path):
                continue
            video = _VideoLoop(path, start_ratio)
            if not video.ok:
                continue
            self._videos[stream_id] = video
            track = _DetectionTrack(
                os.path.join(REPO_ROOT, VIDEO_DIR,
                             os.path.splitext(filename)[0] + ".detections.json")
            )
            if track:
                self._tracks[stream_id] = track

        for stream_id in STREAM_ORDER:
            if stream_id in self._videos:
                continue
            relative = FALLBACK_IMAGES.get(stream_id)
            image = (cv2.imread(os.path.join(REPO_ROOT, relative))
                     if relative else None)
            self._stills[stream_id] = (
                cv2.resize(image, STILL_SIZE) if image is not None
                else _placeholder("NO VIDEO")
            )

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="mock-frames", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for video in self._videos.values():
            video.release()

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.time()
            for stream_id in STREAM_ORDER:
                frame, detections = self._next_frame(stream_id)
                if frame is None:
                    continue
                self._annotate(frame, stream_id, now)
                ok, buffer = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75]
                )
                if not ok:
                    continue
                self._context.on_frame(stream_id, buffer.tobytes())
                self._context.on_person_count(stream_id, detections)
            time.sleep(1.0 / FPS)

    def _next_frame(self, stream_id: str):
        """(프레임, 그 프레임의 검출 수) 를 준다."""
        video = self._videos.get(stream_id)
        if video is not None:
            # 검출 수는 프레임을 읽기 전 위치로 본다. read() 가 인덱스를
            # 올린 뒤 물으면 방금 준 그림보다 한 칸 앞선 값이 나간다.
            position = video.position_s
            frame = video.read()
            if frame is not None:
                track = self._tracks.get(stream_id)
                return frame, (track.at(position) if track else 0)
        still = self._stills.get(stream_id)
        return (still.copy() if still is not None else None), 0

    def _annotate(self, frame, stream_id: str, now: float) -> None:
        """녹화본이라는 표시만 남긴다.

        갈래 이름·시각·fps 는 화면이 타일 라벨로 이미 보여준다. 그림 위에
        또 쓰면 라벨과 겹쳐 둘 다 안 읽힌다. 여기서는 실시간이 아니라는
        것만 알리면 된다.

        검출 상자도 그리지 않는다. 영상에 vision_detector 가 그린 진짜
        상자가 이미 들어 있어서, 가짜를 겹치면 어느 쪽이 진짜인지 구분되지
        않는다.
        """
        width = frame.shape[1]
        cv2.rectangle(frame, (width - 62, 8), (width - 8, 30), (0, 0, 0), -1)
        cv2.putText(frame, "MOCK", (width - 56, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 255), 1)

        if stream_id in ROBOT_STREAMS:
            centre_x, centre_y = width // 2, frame.shape[0] // 2
            cv2.line(frame, (centre_x - 18, centre_y),
                     (centre_x + 18, centre_y), (120, 200, 120), 1)
            cv2.line(frame, (centre_x, centre_y - 18),
                     (centre_x, centre_y + 18), (120, 200, 120), 1)
