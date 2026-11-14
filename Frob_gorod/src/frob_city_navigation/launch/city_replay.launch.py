"""Hardware-isolated Frob City replay/headless launch (WP13)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('frob_city_navigation'))
    autonomy = share / 'launch' / 'city_autonomy.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('include_localization', default_value='false'),
        DeclareLaunchArgument('include_nav2', default_value='false'),
        DeclareLaunchArgument('include_detector', default_value='false'),
        DeclareLaunchArgument('include_map_planner', default_value='true'),
        DeclareLaunchArgument('include_navigator', default_value='true'),
        DeclareLaunchArgument('include_motion_guard', default_value='true'),
        DeclareLaunchArgument('include_readiness', default_value='true'),
        DeclareLaunchArgument('motion_guard_config', default_value=str(share / 'config' / 'motion_guard.yaml')),
        DeclareLaunchArgument('readiness_config', default_value=str(share / 'config' / 'readiness.yaml')),
        DeclareLaunchArgument('field_config', default_value=str(share / 'config' / 'wp11_test_field.yaml')),
        DeclareLaunchArgument('event_config', default_value=str(share / 'config' / 'wp11_test_event.yaml')),
        DeclareLaunchArgument('perception_config', default_value=str(share / 'config' / 'wp11_test_perception.yaml')),
        DeclareLaunchArgument('navigation_config', default_value=str(share / 'config' / 'wp11_test_navigation.yaml')),
        DeclareLaunchArgument('navigator_config', default_value=str(share / 'config' / 'navigator.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(autonomy)),
            launch_arguments={
                'include_robot_bringup': 'false',
                'include_localization': LaunchConfiguration('include_localization'),
                'include_nav2': LaunchConfiguration('include_nav2'),
                'include_detector': LaunchConfiguration('include_detector'),
                'include_map_planner': LaunchConfiguration('include_map_planner'),
                'include_navigator': LaunchConfiguration('include_navigator'),
                'include_motion_guard': LaunchConfiguration('include_motion_guard'),
                'include_readiness': LaunchConfiguration('include_readiness'),
                'motion_guard_config': LaunchConfiguration('motion_guard_config'),
                'readiness_config': LaunchConfiguration('readiness_config'),
                'field_config': LaunchConfiguration('field_config'),
                'event_config': LaunchConfiguration('event_config'),
                'perception_config': LaunchConfiguration('perception_config'),
                'navigation_config': LaunchConfiguration('navigation_config'),
                'navigator_config': LaunchConfiguration('navigator_config'),
                'replay_mode': 'true',
                'raw_cmd_topic': '/city/cmd_vel_raw',
                'driver_cmd_topic': '/city/replay/cmd_vel_sink',
                'real_driver_topic': '/cmd_vel',
                'readiness_log_path': '/tmp/frob_city_replay_readiness.jsonl',
            }.items(),
        ),
    ])
