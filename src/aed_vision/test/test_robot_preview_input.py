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
