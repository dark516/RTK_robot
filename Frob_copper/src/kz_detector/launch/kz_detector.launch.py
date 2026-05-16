from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hostname1', default_value='bunderkrivitka-12.local'),
        DeclareLaunchArgument('hostname2', default_value='bunderkrivitka-34.local'),
        DeclareLaunchArgument('port', default_value='8888'),

        Node(
            package='kz_detector',
            executable='kz_detector',
            name='kz_detector',
            parameters=[{
                'hostname1': LaunchConfiguration('hostname1'),
                'hostname2': LaunchConfiguration('hostname2'),
                'port': LaunchConfiguration('port'),
            }],
            output='screen'
        ),
    ])
