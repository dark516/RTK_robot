from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("frob_city_web"))
    default_config = str(share / "config" / "web.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        Node(
            package="frob_city_web",
            executable="frob_city_web",
            name="frob_city_web",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {"http_host": LaunchConfiguration("http_host"),
                 "http_port": LaunchConfiguration("http_port")},
            ],
        ),
    ])
