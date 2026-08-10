"""Decode ROS compressedDepth payloads without depending on image_transport."""

import cv2
import numpy as np


def decode_compressed_depth(data: bytes) -> np.ndarray:
    """Return the 16-bit PNG carried after a compressedDepth header."""
    signature = b"\x89PNG\r\n\x1a\n"
    start = bytes(data).find(signature)
    if start < 0:
        raise ValueError("compressedDepth PNG signature missing")
    encoded = np.frombuffer(data[start:], dtype=np.uint8)
    depth = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if depth is None or depth.ndim != 2:
        raise ValueError("compressedDepth PNG decode failed")
    return depth
