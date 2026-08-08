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
    target = LaunchConfiguration("target").perform(context)
    detection_backend = {
        "person": "person_pose",
        "mannequin": "mannequin_detect",
    }[target]
    person_conf = float(LaunchConfiguration("person_conf").perform(context))
    rescue_conf = float(LaunchConfiguration("rescue_conf").perform(context))
    helper_max_distance_ratio = float(
        LaunchConfiguration("helper_max_distance_ratio").perform(context)
    )
    if not 0.0 <= person_conf <= 1.0 or not 0.0 <= rescue_conf <= 1.0:
        raise ValueError("person_conf and rescue_conf must be between 0 and 1")
    if not 0.0 < helper_max_distance_ratio <= 1.0:
        raise ValueError("helper_max_distance_ratio must be in (0, 1]")
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
                    # TurtleBot CDR 경로에 이미 들어오는 작은 preview를 재사용한다.
                    # 메인 compressed 토픽을 새로 원격 구독하면 로봇 WiFi 큐가
                    # 밀리므로, 추론 입력은 preview Image로 유지한다.
                    "image_topic": (
                        f"/{robot_id}/oakd/rgb/preview/image_raw"
                    ),
                    "image_is_compressed": False,
                    "frame_id": f"{robot_id}/oakd_rgb_camera_optical_frame",
                    "detection_backend": detection_backend,
                    "person_conf": person_conf,
                    "rescue_conf": rescue_conf,
                    "helper_max_distance_ratio": helper_max_distance_ratio,
                },
            ],
            output="screen",
            # The workspace is built with PYTHONNOUSERSITE=1 to isolate
            # colcon from user-installed setuptools.  Ultralytics has no
            # Humble rosdep package and is installed in the user site, so
            # enable that site only for the inference process.
            additional_env={"PYTHONNOUSERSITE": ""},
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """로봇 ID를 받는 OAK-D Vision launch 인터페이스를 반환한다."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="robot1"),
            DeclareLaunchArgument(
                "target", default_value="person",
                choices=("person", "mannequin"),
            ),
            DeclareLaunchArgument("person_conf", default_value="0.5"),
            DeclareLaunchArgument("rescue_conf", default_value="0.25"),
            DeclareLaunchArgument(
                "helper_max_distance_ratio", default_value="0.30"
            ),
            OpaqueFunction(function=_create_robot_vision),
        ]
    )
