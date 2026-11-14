from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    description_launch = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'launch', 'description.launch.py'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'rviz', 'display_config.rviz'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rover_model_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
