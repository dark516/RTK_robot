from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_url', default_value='http://127.0.0.1:8091/stream?mode=live'),
        DeclareLaunchArgument('vlm_server_url', default_value='http://127.0.0.1:8091/v1/chat/completions'),
        DeclareLaunchArgument('mode', default_value='service', choices=['service', 'continuous']),
        Node(
            package='rover_vision',
            executable='vlm_diagnostic_node',
            name='vlm_diagnostic_node',
            output='screen',
            parameters=[{
                name: LaunchConfiguration(name)
                for name in ('robot_url', 'vlm_server_url', 'mode')
            }],
        ),
    ])
