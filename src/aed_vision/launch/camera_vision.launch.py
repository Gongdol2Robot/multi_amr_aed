"""카메라 번호 1/2에 맞는 웹캠 발행기와 비전 검출기를 실행한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_PROFILES = {
    # 실행 인자 -> (ROS namespace, 설치된 파라미터 YAML)
    "1": ("camera_open", "open_camera.yaml"),
    "2": ("camera_alley", "alley_camera.yaml"),
}


def _create_camera_nodes(context):
    """실행 인자 1/2를 해석해 같은 설정을 쓰는 두 노드를 생성한다.

    OpaqueFunction을 쓰는 이유는 launch 실행 시점에 camera 인자를 실제 문자열로
    평가한 뒤 Python 딕셔너리에서 namespace와 YAML을 함께 선택하기 위해서다.
    vision_detector가 웹캠을 직접 읽고 추론하므로 카메라별 ROS 노드는 하나다.
    """
    camera = LaunchConfiguration("camera").perform(context)
    target = LaunchConfiguration("target").perform(context)
    detection_backend = {
        "person": "person_pose",
        "mannequin": "mannequin_detect",
    }[target]
    person_conf = float(LaunchConfiguration("person_conf").perform(context))
    rescue_conf = float(LaunchConfiguration("rescue_conf").perform(context))
    if not 0.0 <= person_conf <= 1.0 or not 0.0 <= rescue_conf <= 1.0:
        raise ValueError("person_conf and rescue_conf must be between 0 and 1")
    namespace, config_name = CAMERA_PROFILES[camera]
    share_dir = Path(get_package_share_directory("aed_vision"))
    # 소스 경로가 아니라 install/share 경로를 사용하므로 다른 노트북에 패키지를
    # 설치한 뒤에도 동일한 launch 명령을 사용할 수 있다.
    config = share_dir / "config" / config_name

    return [
        Node(
            package="aed_vision",
            executable="vision_detector",
            name="vision_detector",
            namespace=namespace,
            parameters=[
                str(config),
                {
                    "detection_backend": detection_backend,
                    "person_conf": person_conf,
                    "rescue_conf": rescue_conf,
                },
            ],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """camera:=1 또는 camera:=2만 허용하는 launch 인터페이스를 정의한다.

    choices 검증으로 잘못된 카메라 역할이 기본 설정으로 조용히 실행되는 것을
    방지한다. 번호별 실제 역할은 CAMERA_PROFILES와 각 YAML에 정의되어 있다.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera",
                choices=("1", "2"),
                description=(
                    "1: 탁 트인 공간(fallen 검출), "
                    "2: 골목(fallen+인파 검출)"
                ),
            ),
            DeclareLaunchArgument(
                "target",
                default_value="person",
                choices=("person", "mannequin"),
                description="person (default) or mannequin",
            ),
            DeclareLaunchArgument(
                "person_conf",
                default_value="0.5",
                description="YOLO Pose confidence for actual people (0~1)",
            ),
            DeclareLaunchArgument(
                "rescue_conf",
                default_value="0.25",
                description="Fine-tuned mannequin detector confidence (0~1)",
            ),
            OpaqueFunction(function=_create_camera_nodes),
        ]
    )
