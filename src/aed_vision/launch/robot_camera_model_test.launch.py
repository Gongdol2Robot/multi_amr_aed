"""로봇 OAK-D 시점에서 현재 비전 모델의 검출 결과를 눈으로 확인한다.

운영용 robot_vision.launch.py와 같은 robot_camera.yaml을 사용하지만 출력은
``/<robot_id>_test/vision``으로 분리한다. 따라서 운영 노드가 실행 중이어도
입력 영상만 공유하고 응급 이벤트나 검출 토픽은 서로 충돌하지 않는다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    """launch 문자열 인자를 ROS boolean 파라미터로 변환한다."""
    return value.strip().lower() in ("1", "true", "yes", "on")


def _create_test_node(context):
    """로봇 영상은 구독하고 검출 출력은 테스트 namespace로 분리한다."""
    robot_id = LaunchConfiguration("robot_id").perform(context).strip("/")
    if not robot_id:
        raise ValueError("robot_id must not be empty")

    requested_topic = LaunchConfiguration("image_topic").perform(context)
    image_topic = requested_topic.strip() or (
        f"/{robot_id}/oakd/rgb/image_raw/compressed"
    )
    # 운영 launch와 같은 이유로 압축 스트림이 기본값이다. 다른 raw Image
    # 토픽을 시험할 수 있도록 토픽 이름으로 입력 형식을 판별한다.
    image_is_compressed = image_topic.endswith("/compressed")
    show_window = _as_bool(
        LaunchConfiguration("show_window").perform(context)
    )
    share_dir = Path(get_package_share_directory("aed_vision"))
    base_config = share_dir / "config" / "base_camera.yaml"
    backend = LaunchConfiguration("backend").perform(context)
    backend_config = share_dir / "config" / f"{backend}_backend.yaml"
    robot_config = share_dir / "config" / "robot_camera.yaml"
    test_camera_id = f"{robot_id}_test"

    return [
        Node(
            package="aed_vision",
            executable="vision_detector",
            namespace=f"{robot_id}_vision_test",
            name="vision_detector",
            parameters=[
                str(base_config),
                str(backend_config),
                str(robot_config),
                {
                    "camera_id": test_camera_id,
                    "zone_id": f"{robot_id}_camera_test",
                    "image_topic": image_topic,
                    "image_is_compressed": image_is_compressed,
                    "direct_camera": False,
                    "frame_id": f"{robot_id}/oakd_rgb_camera_optical_frame",
                    "publish_debug_image": True,
                    "show_window": show_window,
                },
            ],
            output="screen",
            additional_env={"PYTHONNOUSERSITE": ""},
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """로봇 ID, 입력 토픽과 로컬 창 사용 여부를 받는다."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id",
                default_value="robot2",
                description="OAK-D 영상을 발행하는 로봇 namespace",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="mannequin",
                choices=("mannequin", "person_pose"),
                description="테스트할 낙상 판정 backend 프로필",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="",
                description=(
                    "테스트할 영상 토픽. 빈 값이면 "
                    "/<robot_id>/oakd/rgb/image_raw/compressed 사용. "
                    "/compressed로 끝나면 CompressedImage로 구독한다"
                ),
            ),
            DeclareLaunchArgument(
                "show_window",
                default_value="true",
                choices=("true", "false"),
                description="서버 화면에 OpenCV 검출 창을 표시할지 여부",
            ),
            OpaqueFunction(function=_create_test_node),
        ]
    )
