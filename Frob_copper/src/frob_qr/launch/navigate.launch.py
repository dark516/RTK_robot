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
        DeclareLaunchArgument('distance', default_value='50', description='Distance per leg (cm)'),
        DeclareLaunchArgument('base_speed', default_value='0.3'),
        DeclareLaunchArgument('wait_duration', default_value='2', description='Pause between legs (s)'),
        DeclareLaunchArgument('kp_fwd', default_value='1.5'),
        DeclareLaunchArgument('ki_fwd', default_value='0.05'),
        DeclareLaunchArgument('kd_fwd', default_value='0.2'),
        DeclareLaunchArgument('kp_bwd', default_value='-1.5'),
        DeclareLaunchArgument('ki_bwd', default_value='-0.05'),
        DeclareLaunchArgument('kd_bwd', default_value='-0.2'),
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
                'kp_bwd': to_float(LaunchConfiguration('kp_bwd')),
                'ki_bwd': to_float(LaunchConfiguration('ki_bwd')),
                'kd_bwd': to_float(LaunchConfiguration('kd_bwd')),
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen'
        ),
    ])
