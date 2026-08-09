from backend.stream.frames import FrameBuffer


def test_frame_buffer_only_returns_new_jpeg_versions() -> None:
    buffer = FrameBuffer("robot1", "Robot 1", "robot")

    version, jpeg = buffer.wait_for_next(0, timeout=0.001)
    assert version == 0
    assert jpeg is None

    buffer.put(b"first")
    version, jpeg = buffer.wait_for_next(version, timeout=0.001)
    assert version == 1
    assert jpeg == b"first"

    same_version, duplicate = buffer.wait_for_next(version, timeout=0.001)
    assert same_version == version
    assert duplicate is None

    buffer.put(b"second")
    version, jpeg = buffer.wait_for_next(version, timeout=0.001)
    assert version == 2
    assert jpeg == b"second"
