from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'rviz', 'lidar_config.rviz'
    ])
    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2', name='rover_lidar_rviz',
            output='screen', arguments=['-d', config],
        )
    ])
