from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("traffic_sign_detector"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=str(share / "config" / "camera_streamer.yaml")
        ),
        Node(
            package="traffic_sign_detector",
            executable="traffic_sign_camera_streamer",
            name="traffic_sign_camera_streamer",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("config_file")],
        ),
    ])
