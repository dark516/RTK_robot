from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def add_override(context, overrides: dict, argument_name: str, parameter_name: str):
    value = LaunchConfiguration(argument_name).perform(context).strip()
    if value:
        overrides[parameter_name] = value


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    overrides = {}
    add_override(context, overrides, 'right_panel_mode', 'right_panel_mode')
    add_override(context, overrides, 'robot_serial', 'robot_serial')
    add_override(context, overrides, 'agent_text_topic', 'agent_text_topic')
    add_override(context, overrides, 'battery_topic', 'battery_topic')

    parameters = [config_file]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='rover_display',
            executable='status_display_node',
            name='rover_status_display_node',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory('rover_display'))
        / 'config'
        / 'default.example.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('right_panel_mode', default_value=''),
        DeclareLaunchArgument('robot_serial', default_value=''),
        DeclareLaunchArgument('agent_text_topic', default_value=''),
        DeclareLaunchArgument('battery_topic', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
