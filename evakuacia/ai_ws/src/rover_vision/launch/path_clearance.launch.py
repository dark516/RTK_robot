from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('rover_vision')) / 'config' / 'path_clearance.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(config)),
        DeclareLaunchArgument('camera_url', default_value='http://127.0.0.1:8091/frame?mode=live'),
        Node(
            package='rover_vision',
            executable='path_clearance_node',
            name='path_clearance_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file'), {
                'camera_url': LaunchConfiguration('camera_url'),
            }],
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
    ])
