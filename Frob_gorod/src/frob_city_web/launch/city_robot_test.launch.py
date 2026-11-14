"""Integrated, held Frob City autonomy and operator dashboard launch.

Hardware, localization, Nav2 and detector includes remain explicit opt-ins.
The dashboard can only call the canonical mission control service; it has no
velocity publisher or Nav2 action client.
"""
from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _city(namespace: str, leaf: str) -> str:
    prefix = "/" + namespace.strip("/") if namespace.strip("/") else ""
    return f"{prefix}/{leaf.lstrip('/')}"


def _launch_setup(context, *args, **kwargs):
    nav_share = Path(get_package_share_directory("frob_city_navigation"))
    namespace = LaunchConfiguration("city_namespace").perform(context)
    http_port = int(LaunchConfiguration("http_port").perform(context))
    if not 1 <= http_port <= 65535:
        raise RuntimeError("http_port must be in 1..65535")

    names = (
        "field_config",
        "event_config",
        "perception_config",
        "navigation_config",
        "navigator_config",
        "motion_guard_config",
        "readiness_config",
        "detection_adapter_config",
        "nav2_params_file",
        "web_config",
    )
    values = {name: LaunchConfiguration(name).perform(context) for name in names}
    for name, value in values.items():
        if not Path(value).is_file():
            raise RuntimeError(f"required {name} file missing: {value}")

    forwarded = {
        "city_namespace": namespace,
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
        "include_robot_bringup": LaunchConfiguration("include_robot_bringup").perform(context),
        "include_localization": LaunchConfiguration("include_localization").perform(context),
        "include_nav2": LaunchConfiguration("include_nav2").perform(context),
        "include_detector": LaunchConfiguration("include_detector").perform(context),
        "include_camera_streamer": LaunchConfiguration("include_camera_streamer").perform(context),
        "include_detection_adapter": LaunchConfiguration("include_detection_adapter").perform(context),
        "field_config": values["field_config"],
        "event_config": values["event_config"],
        "perception_config": values["perception_config"],
        "navigation_config": values["navigation_config"],
        "navigator_config": values["navigator_config"],
        "motion_guard_config": values["motion_guard_config"],
        "readiness_config": values["readiness_config"],
        "detection_adapter_config": values["detection_adapter_config"],
        "nav2_params_file": values["nav2_params_file"],
        "odom_topic": LaunchConfiguration("odom_topic").perform(context),
        "detections_topic": LaunchConfiguration("detections_topic").perform(context),
        "legacy_detections_topic": LaunchConfiguration("legacy_detections_topic").perform(context),
        "follow_path_action": LaunchConfiguration("follow_path_action").perform(context),
        "raw_cmd_topic": LaunchConfiguration("raw_cmd_topic").perform(context),
        "driver_cmd_topic": LaunchConfiguration("driver_cmd_topic").perform(context),
        "real_driver_topic": LaunchConfiguration("real_driver_topic").perform(context),
        "model_files": LaunchConfiguration("model_files").perform(context),
        "record_bag": LaunchConfiguration("record_bag").perform(context),
        "bag_output": LaunchConfiguration("bag_output").perform(context),
    }
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav_share / "launch" / "city_autonomy.launch.py")),
        launch_arguments=forwarded.items(),
    )

    odom_topic = forwarded["odom_topic"]
    detections_topic = forwarded["detections_topic"]
    raw_cmd_topic = forwarded["raw_cmd_topic"]
    driver_cmd_topic = forwarded["driver_cmd_topic"]
    web = Node(
        package="frob_city_web",
        executable="frob_city_web",
        name="frob_city_web",
        output="screen",
        parameters=[
            values["web_config"],
            {
                "http_host": LaunchConfiguration("http_host").perform(context),
                "http_port": http_port,
                "map_frame": LaunchConfiguration("map_frame").perform(context),
                "base_frame": LaunchConfiguration("base_frame").perform(context),
                "map_topic": LaunchConfiguration("map_topic").perform(context),
                "odom_topic": odom_topic,
                "map_state_topic": _city(namespace, "map_state"),
                "mission_state_topic": _city(namespace, "mission_state"),
                "planned_route_topic": _city(namespace, "planned_route"),
                "guard_state_topic": _city(namespace, "guard_state"),
                "motion_permit_topic": _city(namespace, "motion_permit"),
                "detections_topic": detections_topic,
                "annotated_image_topic": LaunchConfiguration("annotated_image_topic").perform(context),
                "annotated_compressed_topic": LaunchConfiguration("annotated_compressed_topic").perform(context),
                "diagnostics_topic": _city(namespace, "diagnostics"),
                "map_planner_diagnostics_topic": _city(namespace, "map_planner_diagnostics"),
                "raw_cmd_topic": raw_cmd_topic,
                "final_cmd_topic": driver_cmd_topic,
                "control_service": _city(namespace, "control_mission"),
                "field_config_path": values["field_config"],
                "event_config_path": values["event_config"],
                "perception_config_path": values["perception_config"],
                "navigation_config_path": values["navigation_config"],
            },
        ],
    )
    return [autonomy, web]


def generate_launch_description():
    nav_share = Path(get_package_share_directory("frob_city_navigation"))
    web_share = Path(get_package_share_directory("frob_city_web"))
    nav_config = nav_share / "config"
    declarations = [
        DeclareLaunchArgument("city_namespace", default_value="/city"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("include_robot_bringup", default_value="false"),
        DeclareLaunchArgument("include_localization", default_value="false"),
        DeclareLaunchArgument("include_nav2", default_value="false"),
        DeclareLaunchArgument("include_detector", default_value="false"),
        DeclareLaunchArgument("include_camera_streamer", default_value="false"),
        DeclareLaunchArgument("include_detection_adapter", default_value="true"),
        DeclareLaunchArgument("field_config", default_value=str(nav_config / "wp11_test_field.yaml")),
        DeclareLaunchArgument("event_config", default_value=str(nav_config / "wp11_test_event.yaml")),
        DeclareLaunchArgument("perception_config", default_value=str(nav_config / "wp11_test_perception.yaml")),
        DeclareLaunchArgument("navigation_config", default_value=str(nav_config / "wp11_test_navigation.yaml")),
        DeclareLaunchArgument("navigator_config", default_value=str(nav_config / "navigator.yaml")),
        DeclareLaunchArgument("motion_guard_config", default_value=str(nav_config / "motion_guard.yaml")),
        DeclareLaunchArgument("readiness_config", default_value=str(nav_config / "readiness.yaml")),
        DeclareLaunchArgument("detection_adapter_config", default_value=str(nav_config / "detection_adapter.yaml")),
        DeclareLaunchArgument("nav2_params_file", default_value=str(nav_config / "nav2_city_params.generated.yaml")),
        DeclareLaunchArgument("web_config", default_value=str(web_share / "config" / "web.yaml")),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("detections_topic", default_value="/traffic_sign/detections"),
        DeclareLaunchArgument("legacy_detections_topic", default_value="/traffic_sign/detections_legacy"),
        DeclareLaunchArgument("annotated_image_topic", default_value="/traffic_sign/annotated_image"),
        DeclareLaunchArgument("annotated_compressed_topic", default_value="/traffic_sign/annotated_image/compressed"),
        DeclareLaunchArgument("follow_path_action", default_value="/follow_path"),
        DeclareLaunchArgument("raw_cmd_topic", default_value="/city/cmd_vel_raw"),
        DeclareLaunchArgument("driver_cmd_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("real_driver_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("model_files", default_value=""),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument("bag_output", default_value="/tmp/frob_city_bag"),
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8080"),
    ]
    return LaunchDescription([*declarations, OpaqueFunction(function=_launch_setup)])
