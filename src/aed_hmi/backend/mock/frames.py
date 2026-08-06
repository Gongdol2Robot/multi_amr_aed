"""ROS 없이 4분할 화면을 채울 가짜 영상.

합성 무늬 대신 실제로 측량할 때 찍은 사진을 쓴다. 화면 배치와 색감을
실물과 비슷하게 두어야, 시연에서 "이게 진짜 화면인가"를 판단할 수 있다.

사진이 없으면 단색 화면으로 떨어진다. 저장소를 갓 받은 사람도 서버가 뜨는
편이 낫다.
"""

import os
import threading
import time

import cv2
import numpy as np

REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)

# 실제 측량 사진. 없으면 단색으로 대체한다.
#
# 고정 웹캠 두 갈래는 실제 그 카메라가 보는 장면을 그대로 쓴다.
# 로봇 OAK-D 는 촬영본이 없어서, 같은 사진의 아래쪽을 크게 잘라 낮은
# 시점처럼 보이게 만든다. 천장 시점 넷이 나란히 있으면 어느 타일이 로봇
# 것인지 구분이 안 되기 때문이다. 실물이 붙으면 이 대체는 쓰이지 않는다.
SOURCE_IMAGES = {
    "camera_open": "tools/survey/cam1/camera_view.jpg",
    "camera_alley": "tools/survey/cam2/camera_view.jpg",
    "robot1": "tools/survey/cam1/p3.jpg",
    "robot2": "tools/survey/cam2/p4.jpg",
}

ROBOT_STREAMS = ("robot1", "robot2")

FPS = 10.0


class MockFrameSource:
    """카메라 갈래마다 시각과 가짜 검출 상자를 얹은 프레임을 만든다."""

    def __init__(self, context) -> None:
        self._context = context
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._bases = {
            stream_id: self._load(path, stream_id in ROBOT_STREAMS)
            for stream_id, path in SOURCE_IMAGES.items()
        }

    @staticmethod
    def _load(relative_path: str, low_angle: bool) -> np.ndarray:
        path = os.path.join(REPO_ROOT, relative_path)
        image = cv2.imread(path)
        if image is None:
            image = np.full((480, 640, 3), 40, np.uint8)
            cv2.putText(image, "NO SIGNAL", (180, 250),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (80, 80, 200), 3)
            return image
        if low_angle:
            # 아래쪽 절반을 잘라 확대하면 바닥이 가까이 보여, 로봇에 달린
            # 카메라의 낮은 시점과 비슷해진다.
            height = image.shape[0]
            image = image[int(height * 0.45):, :]
        return cv2.resize(image, (640, 480))

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="mock-frames", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.time()
            for stream_id, base in self._bases.items():
                frame = base.copy()
                self._annotate(frame, stream_id, now)
                ok, buffer = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75]
                )
                if ok:
                    self._context.on_frame(stream_id, buffer.tobytes())
            time.sleep(1.0 / FPS)

    def _annotate(self, frame, stream_id: str, now: float) -> None:
        height, width = frame.shape[:2]
        label = time.strftime("%H:%M:%S", time.localtime(now))
        source = "OAK-D" if stream_id in ROBOT_STREAMS else "WEBCAM"
        cv2.rectangle(frame, (0, 0), (width, 26), (0, 0, 0), -1)
        cv2.putText(frame, f"{stream_id}  {source}  {label}  MOCK", (8, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        if stream_id in ROBOT_STREAMS:
            # 로봇 카메라라는 것이 한눈에 보이도록 중앙 조준선을 얹는다.
            cx, cy = width // 2, height // 2
            cv2.line(frame, (cx - 18, cy), (cx + 18, cy), (120, 200, 120), 1)
            cv2.line(frame, (cx, cy - 18), (cx, cy + 18), (120, 200, 120), 1)
        # 몇 초에 한 번 검출 상자를 띄워, 화면의 검출 표시가 도는지 본다.
        if int(now) % 8 < 3:
            x = int(width * 0.35 + 20 * np.sin(now))
            y = int(height * 0.45)
            cv2.rectangle(frame, (x, y), (x + 120, y + 90), (0, 200, 255), 2)
            cv2.putText(frame, "person 0.91", (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
