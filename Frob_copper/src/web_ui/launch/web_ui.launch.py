from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('web_port', default_value='5000'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),

        Node(
            package='web_ui',
            executable='web_server',
            name='web_server',
            parameters=[{
                'web_port': LaunchConfiguration('web_port'),
                'host': LaunchConfiguration('host'),
            }],
            output='screen',
        ),
    ])
