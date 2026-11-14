"""Complete held competition stack for the robot computer.

The launch never STARTs the mission.  It brings up hardware, localization,
Nav2, the offboard detection receiver, the semantic stack, safety guard and Web
dashboard.  The operator starts a run only after `/city/system_ready` is true.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    city_share = Path(get_package_share_directory('frob_city_navigation'))
    navigation_share = Path(get_package_share_directory('frob_navigation'))
    bringup_share = Path(get_package_share_directory('frob_bringup'))
    web_share = Path(get_package_share_directory('frob_city_web'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    config = city_share / 'config'

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / 'launch' / 'bringup.launch.py')),
        launch_arguments={'include_imu': LaunchConfiguration('include_imu')}.items(),
    )
    nav2_params = str(config / 'andrey_competition_nav2.yaml')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'localization_launch.py')),
        launch_arguments={
            'map': str(navigation_share / 'maps' / 'andrey_map.yaml'),
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )
    # The mission sends immutable paths directly to FollowPath.  Launch only
    # that controller chain; BT recovery, backup, docking and waypoint action
    # servers are intentionally absent.
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[nav2_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('cmd_vel', 'cmd_vel_nav')],
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen', parameters=[nav2_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('cmd_vel', 'cmd_vel_nav')],
    )
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen', parameters=[nav2_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['controller_server', 'velocity_smoother', 'collision_monitor'],
        }],
    )
    camera = Node(
        package='traffic_sign_detector',
        executable='traffic_sign_camera_streamer',
        name='traffic_sign_camera_streamer',
        output='screen',
        parameters=[str(Path(get_package_share_directory('traffic_sign_detector')) / 'config' / 'camera_streamer.yaml')],
    )
    receiver = Node(
        package='traffic_sign_detector',
        executable='traffic_sign_detection_tcp_receiver',
        name='traffic_sign_detection_tcp_receiver',
        output='screen',
        parameters=[{
            'server_host': LaunchConfiguration('offboard_host'),
            'server_port': LaunchConfiguration('offboard_port'),
            'output_topic': '/traffic_sign/detections_legacy',
        }],
    )
    city_and_web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(web_share / 'launch' / 'city_robot_test.launch.py')),
        launch_arguments={
            'include_robot_bringup': 'false',
            'include_localization': 'false',
            'include_nav2': 'false',
            'include_detector': 'false',
            'include_camera_streamer': 'false',
            'include_detection_adapter': 'true',
            'field_config': str(config / 'andrey_competition_field.yaml'),
            'event_config': str(config / 'andrey_competition_event.yaml'),
            'perception_config': str(config / 'andrey_competition_perception.yaml'),
            'navigation_config': str(config / 'andrey_competition_navigation.yaml'),
            'navigator_config': str(config / 'andrey_competition_navigator.yaml'),
            'motion_guard_config': str(config / 'andrey_competition_motion_guard.yaml'),
            'readiness_config': str(config / 'andrey_competition_readiness.yaml'),
            'detection_adapter_config': str(config / 'detection_adapter.yaml'),
            'nav2_params_file': str(config / 'andrey_competition_nav2.yaml'),
            'odom_topic': '/odometry/filtered',
            'record_bag': LaunchConfiguration('record_bag'),
            'bag_output': LaunchConfiguration('bag_output'),
            'http_host': '0.0.0.0',
            'http_port': LaunchConfiguration('http_port'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('offboard_host', default_value='10.42.0.1'),
        DeclareLaunchArgument('offboard_port', default_value='18451'),
        DeclareLaunchArgument('include_imu', default_value='false'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_output', default_value='/home/pi/Frob_gorod/bags/competition_run'),
        hardware,
        localization,
        controller,
        velocity_smoother,
        collision_monitor,
        navigation_lifecycle,
        camera,
        receiver,
        TimerAction(period=3.0, actions=[city_and_web]),
    ])
