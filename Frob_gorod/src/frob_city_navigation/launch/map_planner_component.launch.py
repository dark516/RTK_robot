from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("frob_city_navigation"))
    config = share / "config"
    args = [
        DeclareLaunchArgument("field_config_path", default_value=str(config / "wp11_test_field.yaml")),
        DeclareLaunchArgument("event_config_path", default_value=str(config / "wp11_test_event.yaml")),
        DeclareLaunchArgument("perception_config_path", default_value=str(config / "wp11_test_perception.yaml")),
        DeclareLaunchArgument("navigation_config_path", default_value=str(config / "wp11_test_navigation.yaml")),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("detections_topic", default_value="/traffic_sign/detections"),
    ]
    node = Node(
        package="frob_city_navigation",
        executable="map_planner",
        name="map_planner",
        output="screen",
        parameters=[{
            "field_config_path": LaunchConfiguration("field_config_path"),
            "event_config_path": LaunchConfiguration("event_config_path"),
            "perception_config_path": LaunchConfiguration("perception_config_path"),
            "navigation_config_path": LaunchConfiguration("navigation_config_path"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "detections_topic": LaunchConfiguration("detections_topic"),
        }],
    )
    return LaunchDescription(args + [node])
