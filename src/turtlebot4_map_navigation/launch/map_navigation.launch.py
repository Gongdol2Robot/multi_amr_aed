"""Launch localization, Nav2, and RViz with a saved TurtleBot 4 map."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the saved-map navigation launch description."""
    turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation'
    )
    aed_bringup = get_package_share_directory('aed_bringup')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    use_saved_initial_pose = LaunchConfiguration('use_saved_initial_pose')
    use_dock_initial_pose = LaunchConfiguration('use_dock_initial_pose')
    dock_poses_yaml = LaunchConfiguration('dock_poses_yaml')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw_deg = LaunchConfiguration('initial_yaw_deg')
    nav2_params = LaunchConfiguration('nav2_params')

    localization_launch = PathJoinSubstitution(
        [turtlebot4_navigation, 'launch', 'localization.launch.py']
    )
    navigation_launch = PathJoinSubstitution(
        [turtlebot4_navigation, 'launch', 'nav2.launch.py']
    )
    rviz_launch = PathJoinSubstitution(
        [nav2_bringup, 'launch', 'rviz_launch.py']
    )
    rviz_config = PathJoinSubstitution(
        [nav2_bringup, 'rviz', 'nav2_namespaced_view.rviz']
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'map': map_yaml,
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            # 표준 lifecycle manager가 lifecycle bond까지 함께 소유해야 한다.
            'autostart': 'true',
        }.items(),
    )
    localization_initializer = Node(
        package='turtlebot4_map_navigation',
        executable='localization_initializer',
        namespace=namespace,
        name='localization_initializer',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[
            {
                'auto_initial_pose': auto_initial_pose,
                'use_saved_initial_pose': use_saved_initial_pose,
                'use_dock_initial_pose': use_dock_initial_pose,
                'dock_poses_yaml': dock_poses_yaml,
                'map_yaml': map_yaml,
                'initial_x': initial_x,
                'initial_y': initial_y,
                'initial_yaw_deg': initial_yaw_deg,
            }
        ],
    )
    rviz_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        condition=IfCondition(rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'true',
            'rviz_config': rviz_config,
        }.items(),
    )
    navigation_initializer = Node(
        package='turtlebot4_map_navigation',
        executable='navigation_initializer',
        namespace=namespace,
        name='navigation_initializer',
        output='screen',
    )
    nav_diagnostics = Node(
        package='turtlebot4_map_navigation',
        executable='nav_diagnostics',
        namespace=namespace,
        name='nav_diagnostics',
        output='screen',
    )
    start_navigation_when_localized = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_initializer,
            # TF가 준비되기 전에 RViz가 scan을 쌓지 않도록 Nav2와 함께 연다.
            on_exit=[
                navigation,
                navigation_initializer,
                nav_diagnostics,
                rviz_view,
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='robot1',
                description='TurtleBot 4 ROS namespace',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                choices=['true', 'false'],
                description='Use simulation time',
            ),
            DeclareLaunchArgument(
                'map',
                default_value=PathJoinSubstitution(
                    [aed_bringup, 'maps', 'map.yaml']
                ),
                description='Shared robot1/robot2 map YAML',
            ),
            DeclareLaunchArgument(
                'nav2_params',
                default_value=PathJoinSubstitution(
                    [aed_bringup, 'config', 'nav2_aed.yaml']
                ),
                description='Nav2 parameter file',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Open the namespaced Nav2 RViz view',
            ),
            DeclareLaunchArgument(
                'auto_initial_pose',
                default_value='true',
                choices=['true', 'false'],
                description='Automatically use the configured initial pose',
            ),
            DeclareLaunchArgument(
                'use_dock_initial_pose',
                default_value='true',
                choices=['true', 'false'],
                description='Load the namespace-specific Dock pose',
            ),
            DeclareLaunchArgument(
                'dock_poses_yaml',
                default_value=PathJoinSubstitution(
                    [aed_bringup, 'config', 'dock_poses.yaml']
                ),
                description='Shared-map initial poses for each robot',
            ),
            DeclareLaunchArgument(
                'use_saved_initial_pose',
                default_value='false',
                choices=['true', 'false'],
                description='Load the robot pose recorded with the map',
            ),
            DeclareLaunchArgument(
                'initial_x',
                default_value='0.0',
                description='Automatic initial map X coordinate',
            ),
            DeclareLaunchArgument(
                'initial_y',
                default_value='0.0',
                description='Automatic initial map Y coordinate',
            ),
            DeclareLaunchArgument(
                'initial_yaw_deg',
                default_value='0.0',
                description='Automatic initial heading in degrees',
            ),
            localization,
            start_navigation_when_localized,
            localization_initializer,
        ]
    )
