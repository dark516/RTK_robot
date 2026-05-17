from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def to_float(config):
    return PythonExpression(['float("', config, '")'])


def to_int(config):
    return PythonExpression(['int("', config, '")'])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('distance', default_value='50', description='Distance to drive forward (cm)'),
        DeclareLaunchArgument('base_speed', default_value='0.3'),
        DeclareLaunchArgument('wait_duration', default_value='2', description='Pause after reaching target (s)'),
        DeclareLaunchArgument('kp_fwd', default_value='1.5'),
        DeclareLaunchArgument('ki_fwd', default_value='0.05'),
        DeclareLaunchArgument('kd_fwd', default_value='0.2'),
        DeclareLaunchArgument('show_window', default_value='False'),
        DeclareLaunchArgument('verbose', default_value='False'),

        Node(
            package='frob_qr', executable='qr_detector', name='qr_detector',
            parameters=[{
                'camera_id': to_int(LaunchConfiguration('camera_id')),
                'show_window': LaunchConfiguration('show_window'),
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen'
        ),
        Node(
            package='frob_qr', executable='qr_navigator', name='qr_navigator',
            parameters=[{
                'distance': to_float(LaunchConfiguration('distance')),
                'base_speed': to_float(LaunchConfiguration('base_speed')),
                'wait_duration': to_float(LaunchConfiguration('wait_duration')),
                'kp_fwd': to_float(LaunchConfiguration('kp_fwd')),
                'ki_fwd': to_float(LaunchConfiguration('ki_fwd')),
                'kd_fwd': to_float(LaunchConfiguration('kd_fwd')),
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen'
        ),
    ])
