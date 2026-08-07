"""TurtleBot4 한 대의 OAK-D 구조 인력 검출 노드를 실행한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_robot_vision(context):
    """robot_id를 namespace와 OAK-D 토픽에 적용해 Vision 노드를 생성한다."""
    robot_id = LaunchConfiguration("robot_id").perform(context).strip("/")
    if not robot_id:
        raise ValueError("robot_id must not be empty")
    share_dir = Path(get_package_share_directory("aed_vision"))
    config = share_dir / "config" / "robot_camera.yaml"
    return [
        Node(
            package="aed_vision",
            executable="vision_detector",
            namespace=robot_id,
            name="vision_detector",
            parameters=[
                str(config),
                {
                    "camera_id": robot_id,
                    "zone_id": f"{robot_id}_view",
                    "image_topic": (
                        f"/{robot_id}/oakd/rgb/image_raw/compressed"
                    ),
                    "frame_id": f"{robot_id}/oakd_rgb_camera_optical_frame",
                },
            ],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """로봇 ID를 받는 OAK-D Vision launch 인터페이스를 반환한다."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="robot1"),
            OpaqueFunction(function=_create_robot_vision),
        ]
    )
