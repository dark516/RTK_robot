"""Headless Frob City autonomy launch (WP13).

No teleoperation or RViz goal publisher is launched here.  Existing robot,
localization, Nav2, and detector infrastructure are explicit opt-in includes.
"""
from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truth(value: str) -> bool:
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _pkg_file(package: str, relative: str) -> str:
    try:
        path = Path(get_package_share_directory(package)) / relative
    except PackageNotFoundError as exc:
        raise RuntimeError(f'mandatory included package not found: {package}') from exc
    if not path.is_file():
        raise RuntimeError(f'mandatory included file not found: {path}')
    return str(path)


def _city(ns: str, leaf: str) -> str:
    ns = '/' + ns.strip('/') if ns.strip('/') else ''
    return f'{ns}/{leaf.lstrip("/")}'


def _launch_setup(context, *args, **kwargs):
    own_share = Path(get_package_share_directory('frob_city_navigation'))
    city_ns = LaunchConfiguration('city_namespace').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    detections_topic = LaunchConfiguration('detections_topic').perform(context)
    follow_path = LaunchConfiguration('follow_path_action').perform(context)
    raw_cmd = LaunchConfiguration('raw_cmd_topic').perform(context)
    driver_cmd = LaunchConfiguration('driver_cmd_topic').perform(context)
    replay_mode = _truth(LaunchConfiguration('replay_mode').perform(context))

    field_cfg = LaunchConfiguration('field_config').perform(context)
    event_cfg = LaunchConfiguration('event_config').perform(context)
    perception_cfg = LaunchConfiguration('perception_config').perform(context)
    navigation_cfg = LaunchConfiguration('navigation_config').perform(context)
    navigator_cfg = LaunchConfiguration('navigator_config').perform(context)
    guard_cfg = LaunchConfiguration('motion_guard_config').perform(context)
    readiness_cfg = LaunchConfiguration('readiness_config').perform(context)
    adapter_cfg = LaunchConfiguration('detection_adapter_config').perform(context)
    nav2_cfg = LaunchConfiguration('nav2_params_file').perform(context)
    config_files = [field_cfg, event_cfg, perception_cfg, navigation_cfg, navigator_cfg, guard_cfg]
    if _truth(LaunchConfiguration('include_detection_adapter').perform(context)):
        config_files.append(adapter_cfg)
    if Path(nav2_cfg).is_file():
        config_files.append(nav2_cfg)
    for path in config_files:
        if not Path(path).is_file():
            raise RuntimeError(f'WP13 required config file missing: {path}')

    model_files = [p for p in LaunchConfiguration('model_files').perform(context).split(';') if p]

    topics = {
        'map': _city(city_ns, 'map_state'),
        'guard': _city(city_ns, 'guard_state'),
        'permit': _city(city_ns, 'motion_permit'),
        'mission': _city(city_ns, 'mission_state'),
        'control': _city(city_ns, 'control_mission'),
        'reset': _city(city_ns, 'reset_map'),
        'plan': _city(city_ns, 'plan_route'),
        'validate': _city(city_ns, 'validate_segment'),
        'report': _city(city_ns, 'report_execution'),
        'observe': _city(city_ns, 'observe'),
        'ready': _city(city_ns, 'system_ready'),
        'diagnostics': _city(city_ns, 'diagnostics'),
        'fingerprint': _city(city_ns, 'config_fingerprint'),
        'map_diag': _city(city_ns, 'map_planner_diagnostics'),
    }

    actions = []
    if _truth(LaunchConfiguration('include_robot_bringup').perform(context)):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            _pkg_file(
                LaunchConfiguration('robot_bringup_package').perform(context),
                LaunchConfiguration('robot_bringup_launch').perform(context),
            )
        )))
    if _truth(LaunchConfiguration('include_localization').perform(context)):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            _pkg_file(
                LaunchConfiguration('localization_package').perform(context),
                LaunchConfiguration('localization_launch').perform(context),
            )
        )))
    if _truth(LaunchConfiguration('include_nav2').perform(context)):
        nav2_launch = _pkg_file(
            LaunchConfiguration('nav2_package').perform(context),
            LaunchConfiguration('nav2_launch').perform(context),
        )
        if not Path(nav2_cfg).is_file():
            raise RuntimeError(f'Nav2 params file missing: {nav2_cfg}')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': nav2_cfg,
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                'autostart': 'true',
            }.items(),
        ))
    if _truth(LaunchConfiguration('include_detector').perform(context)):
        detector_package = LaunchConfiguration('detector_package').perform(context)
        detector_executable = LaunchConfiguration('detector_executable').perform(context)
        detector_cfg = _pkg_file(detector_package, LaunchConfiguration('detector_config').perform(context))
        actions.append(Node(
            package=detector_package,
            executable=detector_executable,
            name='traffic_sign_detector',
            output='screen',
            parameters=[detector_cfg, {
                'detections_topic': LaunchConfiguration('legacy_detections_topic').perform(context)
            }],
        ))
    if _truth(LaunchConfiguration('include_camera_streamer').perform(context)):
        streamer_package = LaunchConfiguration('camera_streamer_package').perform(context)
        streamer_executable = LaunchConfiguration('camera_streamer_executable').perform(context)
        streamer_cfg = _pkg_file(
            streamer_package,
            LaunchConfiguration('camera_streamer_config').perform(context),
        )
        actions.append(Node(
            package=streamer_package,
            executable=streamer_executable,
            name='traffic_sign_camera_streamer',
            output='screen',
            parameters=[streamer_cfg],
        ))
    if _truth(LaunchConfiguration('include_detection_adapter').perform(context)):
        actions.append(Node(
            package='frob_city_navigation', executable='detection_adapter',
            name='detection_adapter', output='screen',
            parameters=[adapter_cfg, {
                'input_topic': LaunchConfiguration('legacy_detections_topic').perform(context),
                'output_topic': detections_topic,
            }],
        ))

    if _truth(LaunchConfiguration('include_map_planner').perform(context)):
        actions.append(Node(
            package='frob_city_navigation', executable='map_planner', name='map_planner', output='screen',
            parameters=[{
                'field_config_path': field_cfg,
                'event_config_path': event_cfg,
                'perception_config_path': perception_cfg,
                'navigation_config_path': navigation_cfg,
                'odom_topic': odom_topic,
                'detections_topic': detections_topic,
                'use_sim_time': _truth(LaunchConfiguration('use_sim_time').perform(context)),
            }],
            remappings=[
                ('/city/map_state', topics['map']), ('/city/plan_route', topics['plan']),
                ('/city/validate_segment', topics['validate']), ('/city/report_execution', topics['report']),
                ('/city/reset_map', topics['reset']), ('/city/observe', topics['observe']),
                ('/diagnostics', topics['map_diag']),
            ],
        ))
    if _truth(LaunchConfiguration('include_navigator').perform(context)):
        actions.append(Node(
            package='frob_city_navigation', executable='navigator', name='navigator', output='screen',
            parameters=[navigator_cfg, {
                'map_topic': topics['map'], 'odom_topic': odom_topic,
                'guard_state_topic': topics['guard'], 'permit_topic': topics['permit'],
                'mission_state_topic': topics['mission'], 'control_service': topics['control'],
                'reset_map_service': topics['reset'], 'plan_service': topics['plan'],
                'validate_service': topics['validate'], 'report_service': topics['report'],
                'observe_action': topics['observe'], 'follow_path_action': follow_path,
                'system_ready_topic': topics['ready'], 'require_system_ready': True,
                'use_sim_time': _truth(LaunchConfiguration('use_sim_time').perform(context)),
            }],
        ))
    if _truth(LaunchConfiguration('include_motion_guard').perform(context)):
        actions.append(Node(
            package='frob_city_navigation', executable='motion_guard', name='motion_guard', output='screen',
            parameters=[guard_cfg, {
                'raw_cmd_topic': raw_cmd, 'driver_cmd_topic': driver_cmd,
                'map_topic': topics['map'], 'permit_topic': topics['permit'],
                'guard_state_topic': topics['guard'], 'odom_topic': odom_topic,
                'use_sim_time': _truth(LaunchConfiguration('use_sim_time').perform(context)),
            }],
        ))
    if _truth(LaunchConfiguration('include_readiness').perform(context)):
        actions.append(Node(
            package='frob_city_navigation', executable='city_readiness', name='city_readiness', output='screen',
            parameters=[readiness_cfg, {
                'system_ready_topic': topics['ready'], 'diagnostics_topic': topics['diagnostics'],
                'fingerprint_topic': topics['fingerprint'], 'map_topic': topics['map'],
                'guard_state_topic': topics['guard'], 'mission_state_topic': topics['mission'],
                'odom_topic': odom_topic, 'detections_topic': detections_topic,
                'raw_cmd_topic': raw_cmd, 'driver_cmd_topic': driver_cmd,
                'real_driver_topic': LaunchConfiguration('real_driver_topic').perform(context),
                'follow_path_action': follow_path, 'replay_mode': replay_mode,
                # launch_ros cannot infer the ROS parameter-array type from an
                # empty Python list: it normalizes [] to tuple(), then rejects
                # the untyped value.  Readiness already ignores blank entries,
                # so preserve a typed string array with a blank sentinel.
                'config_files': config_files,
                'model_files': model_files if model_files else [''],
                'log_path': LaunchConfiguration('readiness_log_path').perform(context),
                'use_sim_time': _truth(LaunchConfiguration('use_sim_time').perform(context)),
            }],
        ))

    if _truth(LaunchConfiguration('record_bag').perform(context)):
        bag_dir = LaunchConfiguration('bag_output').perform(context)
        actions.append(ExecuteProcess(cmd=[
            'ros2', 'bag', 'record', '-o', bag_dir,
            topics['map'], topics['mission'], topics['guard'], topics['permit'], topics['ready'],
            topics['diagnostics'], topics['fingerprint'], odom_topic, detections_topic,
            raw_cmd, driver_cmd, '/tf', '/tf_static',
        ], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('frob_city_navigation'))
    config = share / 'config'
    args = [
        DeclareLaunchArgument('city_namespace', default_value='/city'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('include_robot_bringup', default_value='false'),
        DeclareLaunchArgument('include_localization', default_value='false'),
        DeclareLaunchArgument('include_nav2', default_value='false'),
        DeclareLaunchArgument('include_detector', default_value='false'),
        DeclareLaunchArgument('include_camera_streamer', default_value='false'),
        DeclareLaunchArgument('include_detection_adapter', default_value='true'),
        DeclareLaunchArgument('include_map_planner', default_value='true'),
        DeclareLaunchArgument('include_navigator', default_value='true'),
        DeclareLaunchArgument('include_motion_guard', default_value='true'),
        DeclareLaunchArgument('include_readiness', default_value='true'),
        DeclareLaunchArgument('robot_bringup_package', default_value='frob_bringup'),
        DeclareLaunchArgument('robot_bringup_launch', default_value='launch/bringup.launch.py'),
        DeclareLaunchArgument('localization_package', default_value='frob_odometry'),
        DeclareLaunchArgument('localization_launch', default_value='launch/odometry.launch.py'),
        DeclareLaunchArgument('nav2_package', default_value='nav2_bringup'),
        DeclareLaunchArgument('nav2_launch', default_value='launch/navigation_launch.py'),
        DeclareLaunchArgument('detector_package', default_value='traffic_sign_detector'),
        DeclareLaunchArgument('detector_executable', default_value='traffic_sign_detector'),
        DeclareLaunchArgument('detector_config', default_value='config/detector.yaml'),
        DeclareLaunchArgument('camera_streamer_package', default_value='traffic_sign_detector'),
        DeclareLaunchArgument('camera_streamer_executable', default_value='traffic_sign_camera_streamer'),
        DeclareLaunchArgument('camera_streamer_config', default_value='config/camera_streamer.yaml'),
        DeclareLaunchArgument('field_config', default_value=str(config / 'wp11_test_field.yaml')),
        DeclareLaunchArgument('event_config', default_value=str(config / 'wp11_test_event.yaml')),
        DeclareLaunchArgument('perception_config', default_value=str(config / 'wp11_test_perception.yaml')),
        DeclareLaunchArgument('navigation_config', default_value=str(config / 'wp11_test_navigation.yaml')),
        DeclareLaunchArgument('navigator_config', default_value=str(config / 'navigator.yaml')),
        DeclareLaunchArgument('motion_guard_config', default_value=str(config / 'motion_guard.yaml')),
        DeclareLaunchArgument('readiness_config', default_value=str(config / 'readiness.yaml')),
        DeclareLaunchArgument('detection_adapter_config', default_value=str(config / 'detection_adapter.yaml')),
        DeclareLaunchArgument('nav2_params_file', default_value=str(config / 'nav2_city_params.generated.yaml')),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('detections_topic', default_value='/traffic_sign/detections'),
        DeclareLaunchArgument('legacy_detections_topic', default_value='/traffic_sign/detections_legacy'),
        DeclareLaunchArgument('follow_path_action', default_value='/follow_path'),
        DeclareLaunchArgument('raw_cmd_topic', default_value='/city/cmd_vel_raw'),
        DeclareLaunchArgument('driver_cmd_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('real_driver_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('replay_mode', default_value='false'),
        DeclareLaunchArgument('model_files', default_value=''),
        DeclareLaunchArgument('readiness_log_path', default_value='/tmp/frob_city_readiness.jsonl'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_output', default_value='/tmp/frob_city_bag'),
    ]
    return LaunchDescription([*args, OpaqueFunction(function=_launch_setup)])
