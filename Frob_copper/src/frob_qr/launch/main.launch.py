from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def to_float(config):
    return PythonExpression(['float("', config, '")'])


def generate_launch_description():
    return LaunchDescription([
        # ===== Параметры =====
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('distance', default_value='150'),
        DeclareLaunchArgument('base_speed', default_value='0.07'),
        DeclareLaunchArgument('kp_fwd', default_value='-1.0'),
        DeclareLaunchArgument('ki_fwd', default_value='0.0'),
        DeclareLaunchArgument('kd_fwd', default_value='0.0'),
        DeclareLaunchArgument('wait_duration', default_value='2'),
        DeclareLaunchArgument('show_window', default_value='False'),
        DeclareLaunchArgument('verbose', default_value='False'),
        DeclareLaunchArgument('web_port', default_value='5000'),


        # ===== KZ-детектор (из kz_detector) =====
        Node(
            package='kz_detector',
            executable='kz_detector',
            name='kz_detector',
            output='screen',
        ),
        
        # ===== QR-детектор (из frob_qr) =====
        Node(
            package='frob_qr',
            executable='qr_detector',
            name='qr_detector',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
                'show_window': LaunchConfiguration('show_window'),
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen',
        ),
        
        # ===== QR-навигатор (из frob_qr) =====
        Node(
            package='frob_qr',
            executable='qr_navigator',
            name='qr_navigator',
            parameters=[{
                'distance': to_float(LaunchConfiguration('distance')),
                'base_speed': to_float(LaunchConfiguration('base_speed')),
                'wait_duration': to_float(LaunchConfiguration('wait_duration')),
                'kp_fwd': to_float(LaunchConfiguration('kp_fwd')),
                'ki_fwd': to_float(LaunchConfiguration('ki_fwd')),
                'kd_fwd': to_float(LaunchConfiguration('kd_fwd')),
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen',
        ),

        # ===== Web-интерфейс (из web_ui) =====
#        Node(
#            package='web_ui',
#            executable='web_server',
#            name='web_server',
#            parameters=[{
#                'web_port': LaunchConfiguration('web_port'),
#            }],
#            output='screen',
#        ),

    ])
