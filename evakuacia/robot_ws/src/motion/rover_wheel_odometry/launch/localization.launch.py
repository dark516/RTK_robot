from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('rover_wheel_odometry'))
    config_dir = share / 'config' / 'localization'
    use_imu = LaunchConfiguration('use_imu')
    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='true'),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[str(config_dir / 'ekf_with_imu.default.example.yaml')],
            remappings=[('odometry/filtered', '/odom')],
            condition=IfCondition(use_imu),
        ),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[str(config_dir / 'ekf_wheel_only.default.example.yaml')],
            remappings=[('odometry/filtered', '/odom')],
            condition=UnlessCondition(use_imu),
        ),
    ])
