import cv2
import numpy as np
import pytest

from sensor_recovery.depth_decode import decode_compressed_depth


def test_decode_compressed_depth_skips_transport_header():
    expected = np.array([[0, 100], [500, 4000]], dtype=np.uint16)
    success, encoded = cv2.imencode(".png", expected)
    assert success
    payload = b"transport123" + encoded.tobytes()
    actual = decode_compressed_depth(payload)
    assert actual.dtype == np.uint16
    assert np.array_equal(actual, expected)


def test_decode_compressed_depth_rejects_non_png_payload():
    with pytest.raises(ValueError, match="signature"):
        decode_compressed_depth(b"not a compressed depth image")
