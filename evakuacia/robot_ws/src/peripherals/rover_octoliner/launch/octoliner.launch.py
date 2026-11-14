from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory('rover_octoliner'))
        / 'config'
        / 'default.example.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='rover_octoliner',
            executable='octoliner_node',
            name='octoliner_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
