"""직접 카메라 입력의 오류 격리 테스트."""

from types import SimpleNamespace

import numpy as np

from aed_vision import camera_source
from aed_vision.camera_source import DirectCameraSource


class _Logger:
    def warning(self, _message) -> None:
        pass


class _Node:
    def get_logger(self) -> _Logger:
        return _Logger()


class _Capture:
    def __init__(self, frame) -> None:
        self.frame = frame

    def read(self):
        return True, self.frame


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def test_jpeg_failure_does_not_skip_inference(monkeypatch) -> None:
    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    received = []
    source = DirectCameraSource.__new__(DirectCameraSource)
    source.node = _Node()
    source.capture = _Capture(frame)
    source.publisher = _Publisher()
    source.on_frame = lambda image, message: received.append((image, message))
    source._param = lambda _name: 80
    source._compressed_message = lambda _encoded=None: SimpleNamespace()
    monkeypatch.setattr(
        camera_source.cv2, "imencode", lambda *_args: (False, None)
    )

    source._read()

    assert len(received) == 1
    assert received[0][0] is frame
    assert source.publisher.messages == []
