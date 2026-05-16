from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('base_speed', default_value='0.3'),
        DeclareLaunchArgument('kp', default_value='1.5'),
        DeclareLaunchArgument('ki', default_value='0.05'),
        DeclareLaunchArgument('kd', default_value='0.2'),
        DeclareLaunchArgument('show_window', default_value='False'),
        DeclareLaunchArgument('verbose', default_value='False'),

        Node(
            package='frob_qr', executable='qr_detector', name='qr_detector',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
                'show_window': LaunchConfiguration('show_window'),
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen'
        ),
        Node(
            package='frob_qr', executable='qr_controller', name='qr_controller',
            parameters=[{
                'base_speed': LaunchConfiguration('base_speed'),
                'kp': LaunchConfiguration('kp'),
                'ki': LaunchConfiguration('ki'),
                'kd': LaunchConfiguration('kd'),
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen'
        ),
    ])


