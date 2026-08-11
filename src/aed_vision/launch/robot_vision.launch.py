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
    base_config = share_dir / "config" / "base_camera.yaml"
    backend = LaunchConfiguration("backend").perform(context)
    backend_config = share_dir / "config" / f"{backend}_backend.yaml"
    config = share_dir / "config" / "robot_camera.yaml"
    return [
        Node(
            package="aed_vision",
            executable="vision_detector",
            namespace=robot_id,
            name="vision_detector",
            parameters=[
                str(base_config),
                str(backend_config),
                str(config),
                {
                    "camera_id": robot_id,
                    "zone_id": f"{robot_id}_view",
                    # HMI에서도 잘리지 않은 전체 시야를 쓰도록 OAK-D의
                    # 저해상도 preview를 운영 입력으로 사용한다. 이 토픽은
                    # 비압축 Image라 네트워크 절감 수단으로 간주하지 않는다.
                    "image_topic": (
                        f"/{robot_id}/oakd/rgb/preview/image_raw"
                    ),
                    "image_is_compressed": False,
                    "frame_id": f"{robot_id}/oakd_rgb_camera_optical_frame",
                    # 운영 노드는 배정 전 OAK-D 영상 구독 자체를 만들지 않는다.
                    "wait_for_assignment": True,
                    "assignment_topic": f"/{robot_id}/mission_assignment",
                    # 낙상 후보가 없으면 조력자 후보도 성립하지 않으므로
                    # 별도 COCO person 전체 프레임 추론을 생략한다.
                    "skip_person_without_fallen": True,
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
            DeclareLaunchArgument("robot_id", default_value="robot2"),
            DeclareLaunchArgument(
                "backend",
                default_value="mannequin",
                choices=("mannequin", "person_pose"),
            ),
            OpaqueFunction(function=_create_robot_vision),
        ]
    )
