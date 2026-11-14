from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_float(context, name: str) -> float:
    return float(LaunchConfiguration(name).perform(context))


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    prefix = Path(LaunchConfiguration('posegraph').perform(context)).expanduser()
    start_mode = LaunchConfiguration('start_mode').perform(context).strip().lower()

    missing = [
        str(prefix.with_suffix(suffix))
        for suffix in ('.posegraph', '.data')
        if not prefix.with_suffix(suffix).is_file()
    ]
    if missing:
        return [
            LogInfo(msg='[ERROR] Pose graph files are missing: ' + ', '.join(missing)),
            LogInfo(msg='Save the map first with: ros2 run rover_navigation rover_map save <label>'),
            EmitEvent(event=Shutdown(reason='pose graph is missing')),
        ]

    overrides = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        'map_file_name': str(prefix),
    }

    if start_mode == 'first':
        overrides['map_start_at_dock'] = True
    elif start_mode == 'given':
        overrides['map_start_pose'] = [
            _as_float(context, 'initial_x'),
            _as_float(context, 'initial_y'),
            _as_float(context, 'initial_yaw'),
        ]
    else:
        return [
            LogInfo(msg=f'[ERROR] start_mode must be first or given, got: {start_mode}'),
            EmitEvent(event=Shutdown(reason='invalid update-map start mode')),
        ]

    return [
        LogInfo(msg=f'Loading SLAM pose graph: {prefix} (start_mode={start_mode})'),
        LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=[params_file, overrides],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': ['slam_toolbox'],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rover_update_map_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            condition=IfCondition(use_rviz),
        ),
    ]


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('rover_navigation'))
    description_share = Path(get_package_share_directory('rover_description'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(pkg_share / 'config' / 'slam_toolbox.default.example.yaml'),
        ),
        DeclareLaunchArgument(
            'posegraph',
            default_value=str(pkg_share / 'maps' / 'current' / 'map'),
            description='Pose graph filename prefix without .posegraph/.data',
        ),
        DeclareLaunchArgument(
            'start_mode',
            default_value='first',
            description='first: use first graph node; given: use initial_x/y/yaw',
        ),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(description_share / 'rviz' / 'display_slam.rviz'),
        ),
        OpaqueFunction(function=launch_setup),
    ])
