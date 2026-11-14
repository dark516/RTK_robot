from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = PathJoinSubstitution([
        FindPackageShare('rover_description'),
        'rviz',
        'display_navigation.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rover_navigation_rviz',
            output='screen',
            arguments=['-d', config],
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            }],
        ),
    ])
