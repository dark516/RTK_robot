"""Test-only real Nav2 controller launch. Final command is isolated from robot hardware."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params=LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',output='screen',parameters=[params],remappings=[('cmd_vel','/wp14n/nav2_cmd')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_wp14',output='screen',parameters=[params]),
    ])
