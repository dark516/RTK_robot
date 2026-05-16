from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        # ===== Параметры =====
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('distance', default_value='150'),
        DeclareLaunchArgument('base_speed', default_value='0.07'),
        DeclareLaunchArgument('kp_fwd', default_value='-1.0'),
        DeclareLaunchArgument('ki_fwd', default_value='0.0'),
        DeclareLaunchArgument('kd_fwd', default_value='0.0'),
        DeclareLaunchArgument('kp_bwd', default_value='-1.0'),
        DeclareLaunchArgument('ki_bwd', default_value='0.00'),
        DeclareLaunchArgument('kd_bwd', default_value='0.0'),
        DeclareLaunchArgument('wait_duration', default_value='2'),
        DeclareLaunchArgument('show_window', default_value='False'),
        DeclareLaunchArgument('verbose', default_value='False'),
        
        
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
                'distance': LaunchConfiguration('distance'),
                'base_speed': LaunchConfiguration('base_speed'),
                'wait_duration': LaunchConfiguration('wait_duration'),
                'kp_fwd': LaunchConfiguration('kp_fwd'),
                'ki_fwd': LaunchConfiguration('ki_fwd'),
                'kd_fwd': LaunchConfiguration('kd_fwd'),
                'kp_bwd': LaunchConfiguration('kp_bwd'),
                'ki_bwd': LaunchConfiguration('ki_bwd'),
                'kd_bwd': LaunchConfiguration('kd_bwd'),
                'verbose': LaunchConfiguration('verbose'),
            }],
            output='screen',
        ),

    ])
