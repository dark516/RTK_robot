from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8888'),
        Node(
            package='rosboard',
            executable='rosboard_node',
            name='rosboard',
            output='screen',
            additional_env={
                # Avoid user-site NumPy 2.x shadowing the distro NumPy used by
                # OpenCV/PIL on Raspberry Pi.
                'PYTHONNOUSERSITE': '1',
            },
            parameters=[{
                'port': LaunchConfiguration('port'),
            }],
        ),
    ])
