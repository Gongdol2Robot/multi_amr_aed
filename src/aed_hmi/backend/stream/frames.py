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
        self._condition = threading.Condition(self._lock)
        self._jpeg: Optional[bytes] = None
        self._version = 0
        self._arrivals: deque[float] = deque(maxlen=FPS_WINDOW)
        self._detections = 0

    def put(self, jpeg: bytes, detections: int = 0) -> None:
        with self._condition:
            self._jpeg = jpeg
            self._version += 1
            self._arrivals.append(time.time())
            self._detections = detections
            # 여러 브라우저가 같은 타일을 보고 있어도 새 프레임 한 장으로
            # 대기 중인 모든 MJPEG 응답을 깨운다.
            self._condition.notify_all()

    def latest(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg

    def wait_for_next(
        self, version: int, timeout: float = 1.0
    ) -> tuple[int, Optional[bytes]]:
        """구독자가 본 버전 이후의 새 프레임만 반환한다.

        타임아웃마다 마지막 JPEG를 다시 보내면 카메라가 끊긴 동안에도
        브라우저 수만큼 중복 트래픽이 생긴다. 새 프레임이 없으면 JPEG 대신
        ``None``을 반환해 연결만 유지한다.
        """
        with self._condition:
            self._condition.wait_for(
                lambda: self._version != version,
                timeout=timeout,
            )
            if self._version == version:
                return version, None
            return self._version, self._jpeg

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
