"""로봇 비전 운영 입력이 OAK-D 압축 JPEG 스트림으로 유지되는지 검증한다.

비압축 preview(약 17Mbps)는 로봇 WiFi 실효 대역폭(~6Mbps)을 초과해 프레임
대부분이 유실되고 로봇 핑이 수백 ms로 튄다. 운영 입력은 로봇 oakd가
image_transport로 발행하는 image_raw/compressed를 사용해야 한다.
"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_operational_robot_vision_uses_compressed_stream() -> None:
    source = (PACKAGE_ROOT / "launch/robot_vision.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'f"/{robot_id}/oakd/rgb/image_raw/compressed"' in source
    assert '"image_is_compressed": True' in source
    assert "oakd/rgb/preview/image_raw" not in source


def test_robot_profile_declares_compressed_image_input() -> None:
    config = (PACKAGE_ROOT / "config/robot_camera.yaml").read_text(
        encoding="utf-8"
    )

    assert "image_is_compressed: true" in config


def test_webcam_and_robot_default_to_rescue2_helper_rc_backend() -> None:
    mannequin_config = (
        PACKAGE_ROOT / "config/mannequin_backend.yaml"
    ).read_text(encoding="utf-8")

    assert "rescue2_yolo11n.pt" in mannequin_config
    assert "detection_backend: mannequin_detect" in mannequin_config
    for launch_name in ("camera_vision.launch.py", "robot_vision.launch.py"):
        source = (PACKAGE_ROOT / f"launch/{launch_name}").read_text(
            encoding="utf-8"
        )
        assert 'default_value="mannequin"' in source
        assert 'LaunchConfiguration("backend")' in source
        assert 'f"{backend}_backend.yaml"' in source
