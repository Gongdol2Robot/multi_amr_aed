"""영상 프레임을 받아 두었다가 브라우저로 흘려보낸다.

전송은 MJPEG 로 한다. 이미 CompressedImage(JPEG)로 오는 것을 그대로
multipart 로 이어 붙이면 되고, 브라우저는 <img src="..."> 만으로 받는다.
WebRTC 는 지연이 훨씬 낮지만 시그널링과 코덱 협상이 붙어 하루에 만들 게
아니다. 지연이 문제가 되면 그때 바꾼다.

프레임은 카메라마다 최신 한 장만 쥔다. 쌓아두면 메모리가 늘고, 관제 화면은
과거 프레임을 볼 이유가 없다.
"""

import threading
import time
from collections import deque
from typing import Optional

from ..domain.models import StreamHealth

# 이 시간 동안 프레임이 없으면 끊긴 것으로 본다.
STREAM_STALE_AFTER_S = 3.0
# FPS 를 낼 때 볼 최근 프레임 수. 짧으면 값이 튀고 길면 변화가 늦게 보인다.
FPS_WINDOW = 30


class FrameBuffer:
    """카메라 한 대의 최신 프레임과 수신율."""

    def __init__(self, stream_id: str, label: str, kind: str) -> None:
        self.stream_id = stream_id
        self.label = label
        self.kind = kind
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._arrivals: deque[float] = deque(maxlen=FPS_WINDOW)
        self._detections = 0
        # 대기 중인 MJPEG 응답들을 깨우는 신호
        self._updated = threading.Event()

    def put(self, jpeg: bytes, detections: int = 0) -> None:
        with self._lock:
            self._jpeg = jpeg
            self._arrivals.append(time.time())
            self._detections = detections
        self._updated.set()
        self._updated.clear()

    def latest(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg

    def wait_for_next(self, timeout: float = 1.0) -> Optional[bytes]:
        """새 프레임이 올 때까지 기다린다. 없으면 마지막 것을 준다.

        폴링으로 돌리면 프레임 속도와 무관하게 CPU 를 태운다.
        """
        self._updated.wait(timeout=timeout)
        return self.latest()

    def health(self) -> StreamHealth:
        with self._lock:
            arrivals = list(self._arrivals)
            detections = self._detections
        if len(arrivals) < 2:
            return StreamHealth(
                self.stream_id, self.label, self.kind,
                online=False, fps=0.0,
                last_frame_at=arrivals[-1] if arrivals else None,
                detections=detections,
            )
        span = arrivals[-1] - arrivals[0]
        fps = (len(arrivals) - 1) / span if span > 0 else 0.0
        online = (time.time() - arrivals[-1]) <= STREAM_STALE_AFTER_S
        return StreamHealth(
            self.stream_id, self.label, self.kind,
            online=online,
            # 끊긴 뒤에도 마지막 fps 를 보여주면 살아있는 것처럼 보인다.
            fps=round(fps, 1) if online else 0.0,
            last_frame_at=arrivals[-1],
            detections=detections,
        )


class FrameRegistry:
    """카메라별 버퍼를 모아 둔다."""

    def __init__(self, sources) -> None:
        self._buffers = {
            source.stream_id: FrameBuffer(
                source.stream_id, source.label, source.kind
            )
            for source in sources
        }

    def put(self, stream_id: str, jpeg: bytes, detections: int = 0) -> None:
        buffer = self._buffers.get(stream_id)
        if buffer is not None:
            buffer.put(jpeg, detections)

    def get(self, stream_id: str) -> Optional[FrameBuffer]:
        return self._buffers.get(stream_id)

    def health(self) -> list[StreamHealth]:
        return [buffer.health() for buffer in self._buffers.values()]

    def stream_ids(self) -> list[str]:
        return list(self._buffers)
