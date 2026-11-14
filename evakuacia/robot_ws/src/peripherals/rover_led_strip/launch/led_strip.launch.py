from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory('rover_led_strip'))
        / 'config'
        / 'default.example.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='rover_led_strip',
            executable='led_strip_node',
            name='led_strip_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
