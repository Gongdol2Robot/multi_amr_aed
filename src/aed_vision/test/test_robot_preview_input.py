"""로봇 비전 운영 입력이 OAK-D preview로 유지되는지 검증한다."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_operational_robot_vision_uses_raw_preview() -> None:
    source = (PACKAGE_ROOT / "launch/robot_vision.launch.py").read_text(
        encoding="utf-8"
    )

    assert "oakd/rgb/preview/image_raw" in source
    assert '"image_is_compressed": False' in source
    assert 'f"/{robot_id}/oakd/rgb/image_raw/compressed"' not in source


def test_robot_profile_declares_raw_image_input() -> None:
    config = (PACKAGE_ROOT / "config/robot_camera.yaml").read_text(
        encoding="utf-8"
    )

    assert "image_is_compressed: false" in config
