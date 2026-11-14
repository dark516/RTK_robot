from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from rover_bringup.configuration import (
    as_launch_bool,
    bringup_config_path,
    component_enabled,
    deep_merge,
    load_component,
    load_profile,
    override_bool,
    read_yaml_file,
    resolve_references,
)
from rover_device_manager.discovery import DEFAULT_DEVICE_CONFIG, prepare_devices


def add_if_set(arguments: dict[str, str], key: str, value) -> None:
    if value is not None and str(value).strip():
        arguments[key] = str(value)


def set_if_blank(target: dict, key: str, value) -> None:
    if not str(target.get(key, '')).strip():
        target[key] = value


def flag(context, profile_config: dict, argument: str, component: str, default: bool):
    value = component_enabled(profile_config, component, default)
    return override_bool(LaunchConfiguration(argument).perform(context), value)


def component_section(components_dir: str, name: str, section: str) -> dict:
    config = load_component(components_dir, name)
    value = config.get(section, {})
    return dict(value) if isinstance(value, dict) else {}


def launch_setup(context):
    profile_name = LaunchConfiguration('profile').perform(context).strip() or 'full'
    profile_file = LaunchConfiguration('profile_file').perform(context).strip()
    profile_config = load_profile(profile_name, profile_file)

    components_dir = (
        LaunchConfiguration('components_config_dir').perform(context).strip()
        or bringup_config_path('components')
    )
    robot_config_file = (
        LaunchConfiguration('robot_config_file').perform(context).strip()
        or bringup_config_path('rover_v1.yaml')
    )
    legacy_config_file = LaunchConfiguration('config_file').perform(context).strip()
    if legacy_config_file:
        robot_config_file = legacy_config_file

    topics_config_file = (
        LaunchConfiguration('topics_config_file').perform(context).strip()
        or bringup_config_path('topics.yaml')
    )
    peripherals_config_file = LaunchConfiguration('peripherals_config_file').perform(
        context
    ).strip()
    ui_config_file = (
        LaunchConfiguration('ui_config_file').perform(context).strip()
        or str(Path(components_dir) / 'ui.yaml')
    )
    device_manager_config = component_section(
        components_dir,
        'device_manager',
        'device_manager',
    )
    runtime_dir = (
        LaunchConfiguration('runtime_dir').perform(context).strip()
        or str(device_manager_config.get('runtime_dir', '/tmp/rover_devices'))
    )
    device_config = (
        LaunchConfiguration('device_config').perform(context).strip()
        or str(device_manager_config.get('device_config', DEFAULT_DEVICE_CONFIG))
    )
    discovery_mode = (
        LaunchConfiguration('discovery_mode').perform(context).strip()
        or str(device_manager_config.get('discovery_mode', 'configured'))
    )

    use_base = flag(context, profile_config, 'use_base', 'base', True)
    use_odometry = flag(context, profile_config, 'use_odometry', 'odometry', True)
    use_description = flag(
        context, profile_config, 'use_description', 'description', True
    )
    use_localization = flag(
        context, profile_config, 'use_localization', 'localization', True
    )
    use_imu = flag(context, profile_config, 'use_imu', 'imu', True)
    use_lidar = flag(context, profile_config, 'use_lidar', 'lidar', True)
    use_camera = flag(context, profile_config, 'use_camera', 'camera', True)
    use_vision = flag(context, profile_config, 'use_vision', 'vision', True)
    use_display = flag(context, profile_config, 'use_display', 'display', False)
    use_led_strip = flag(
        context, profile_config, 'use_led_strip', 'led_strip', True
    )
    use_octoliner = flag(
        context, profile_config, 'use_octoliner', 'octoliner', True
    )
    use_waveshare_audio = flag(
        context, profile_config, 'use_waveshare_audio', 'waveshare_audio', False
    )
    use_web = flag(context, profile_config, 'use_web', 'web', True)
    use_rosboard = flag(
        context, profile_config, 'use_rosboard', 'rosboard', True
    )
    use_agent = flag(context, profile_config, 'use_agent', 'agent', False)
    use_fleet_bridge = flag(
        context, profile_config, 'use_fleet_bridge', 'fleet_bridge', False
    )
    use_nav2 = flag(context, profile_config, 'use_nav2', 'nav2', False)
    use_slam = flag(context, profile_config, 'use_slam', 'slam', False)
    use_mux = flag(context, profile_config, 'use_twist_mux', 'twist_mux', False)
    use_sim_time = override_bool(
        LaunchConfiguration('use_sim_time').perform(context),
        False,
    )
    use_rviz = override_bool(
        LaunchConfiguration('use_rviz').perform(context),
        False,
    )

    rosboard_port = LaunchConfiguration('rosboard_port').perform(context).strip() or '8888'
    display_panel_mode = LaunchConfiguration('display_panel_mode').perform(context).strip()
    display_robot_serial = LaunchConfiguration('display_robot_serial').perform(context).strip()
    motor_override = LaunchConfiguration('motor_device').perform(context).strip() or None
    imu_override = LaunchConfiguration('imu_device').perform(context).strip() or None
    lidar_override = LaunchConfiguration('lidar_device').perform(context).strip() or None
    nav2_map_file = LaunchConfiguration('map').perform(context).strip()
    nav2_params_file = (
        LaunchConfiguration('nav2_params_file').perform(context).strip()
        or bringup_config_path('navigation', 'nav2_params.yaml')
    )
    slam_params_file = (
        LaunchConfiguration('slam_params_file').perform(context).strip()
        or bringup_config_path('navigation', 'slam_toolbox_params.yaml')
    )
    nav2_start_delay = float(
        LaunchConfiguration('nav2_start_delay').perform(context).strip() or '2.0'
    )
    slam_start_delay = float(
        LaunchConfiguration('slam_start_delay').perform(context).strip() or '2.0'
    )

    robot_config = read_yaml_file(robot_config_file)
    topics_config = read_yaml_file(topics_config_file)
    topics = dict(topics_config.get('topics', {}))
    frames = dict(topics_config.get('frames', {}))
    robot_identity = dict(robot_config.get('robot', {}))
    robot_id = str(robot_identity.get('id', 'rover-01'))
    reference_context = {
        'robot': robot_identity,
        'topics': topics,
    }

    if peripherals_config_file:
        legacy_peripherals = read_yaml_file(peripherals_config_file)
        lidar_config = dict(legacy_peripherals.get('lidar', {}))
    else:
        lidar_config = component_section(components_dir, 'lidar', 'lidar')

    results = {}
    needs_serial_devices = (
        use_base
        or use_imu
        or use_lidar
        or bool(motor_override or imu_override or lidar_override)
    )
    if needs_serial_devices:
        try:
            probe_baudrates = tuple(
                int(value) for value in lidar_config.get(
                    'probe_baudrates', [460800, 115200, 256000, 1000000]
                )
            )
            results = prepare_devices(
                mode=discovery_mode,
                config_path=device_config,
                runtime_dir=runtime_dir,
                require_imu=use_imu,
                require_lidar=use_lidar,
                motor_device=motor_override,
                imu_device=imu_override,
                lidar_device=lidar_override,
                lidar_baudrates=probe_baudrates,
            )
        except Exception as exc:
            return [
                LogInfo(msg=f'[ERROR] Hardware discovery failed: {exc}'),
                EmitEvent(event=Shutdown(reason='serial device discovery failed')),
            ]

        detected = [
            f"motor controller: {results['motor_controller'].resolved_device}"
        ]
        if 'imu' in results:
            detected.append(f"IMU: {results['imu'].resolved_device}")
        if 'lidar' in results:
            detected.append(f"lidar: {results['lidar'].resolved_device}")
        device_status = f'device mode={discovery_mode}; ' + '; '.join(detected)
    else:
        device_status = 'serial device discovery skipped; no serial hardware enabled'
    actions = [LogInfo(
        msg=(
            f'Profile={profile_name}; {device_status}'
        )
    )]
    if use_nav2 and use_slam:
        return [
            LogInfo(msg='[ERROR] use_nav2 and use_slam cannot both be true.'),
            EmitEvent(event=Shutdown(reason='conflicting navigation modes')),
        ]

    geometry = dict(robot_config['geometry'])
    encoders = dict(robot_config['encoders'])
    lidar_filter_params = component_section(
        components_dir,
        'lidar_filter',
        'lidar_filter',
    )
    base_component = load_component(components_dir, 'base')
    base_params = deep_merge(
        dict(base_component.get('base_driver', {})),
        dict(robot_config.get('base_driver', {})),
    )
    base_params.update({
        'serial_device': str(Path(runtime_dir) / 'motor_controller'),
        'cmd_vel_topic': topics.get('cmd_vel', base_params.get('cmd_vel_topic', '/cmd_vel')),
        'wheel_radius_m': geometry['wheel_radius_m'],
        'wheelbase_m': geometry['wheelbase_m'],
        'track_width_m': geometry['track_width_m'],
        **encoders,
        'use_sim_time': use_sim_time,
    })
    odom_params = deep_merge(
        dict(base_component.get('wheel_odometry', {})),
        dict(robot_config.get('wheel_odometry', {})),
    )
    odom_params.update({
        'encoder_topic': topics.get('wheel_encoders', odom_params.get('encoder_topic', '/wheel/encoders')),
        'odometry_topic': topics.get('wheel_odometry', odom_params.get('odometry_topic', '/wheel/odometry')),
        'odom_frame_id': frames.get('odom', odom_params.get('odom_frame_id', 'odom')),
        'base_frame_id': frames.get('base', odom_params.get('base_frame_id', 'base_link')),
        'wheel_radius_m': geometry['wheel_radius_m'],
        'wheelbase_m': geometry['wheelbase_m'],
        'track_width_m': geometry['track_width_m'],
        **encoders,
        'use_sim_time': use_sim_time,
    })
    imu_params = deep_merge(
        component_section(components_dir, 'imu', 'imu'),
        dict(robot_config.get('imu', {})),
    )
    imu_params.update({
        'frame_id': frames.get('imu', imu_params.get('frame_id', 'imu_link')),
        'imu_topic': topics.get('imu_data', imu_params.get('imu_topic', '/imu/data')),
        'mag_topic': topics.get('imu_mag', imu_params.get('mag_topic', '/imu/mag')),
        'euler_topic': topics.get('imu_euler', imu_params.get('euler_topic', '/imu/euler')),
        'frame_count_topic': topics.get(
            'imu_frame_count',
            imu_params.get('frame_count_topic', '/imu/valid_frame_count'),
        ),
        'use_sim_time': use_sim_time,
    })

    xacro_file = PathJoinSubstitution([
        FindPackageShare('rover_description'), 'urdf', 'rover.urdf.xacro'
    ])
    chassis_xyz = geometry.get('chassis_xyz', [0.0125, 0.0, 0.0096])
    imu_xyz, imu_rpy = geometry['imu_xyz'], geometry['imu_rpy']
    lidar_xyz = geometry.get('lidar_xyz', [0.0, 0.0, 0.10])
    lidar_rpy = geometry.get('lidar_rpy', [0.0, 0.0, 0.0])
    chassis_half_length = float(geometry['chassis_length_m']) / 2.0
    chassis_half_width = float(geometry['chassis_width_m']) / 2.0
    lidar_filter_params.update({
        'input_topic': topics.get('scan_raw', '/scan'),
        'output_topic': topics.get('scan', '/scan_filtered'),
        'base_frame_id': frames.get('base', 'base_link'),
        'footprint_min_x_m': float(chassis_xyz[0]) - chassis_half_length,
        'footprint_max_x_m': float(chassis_xyz[0]) + chassis_half_length,
        'footprint_min_y_m': float(chassis_xyz[1]) - chassis_half_width,
        'footprint_max_y_m': float(chassis_xyz[1]) + chassis_half_width,
        'fallback_sensor_x_m': float(lidar_xyz[0]),
        'fallback_sensor_y_m': float(lidar_xyz[1]),
        'fallback_sensor_yaw_rad': float(lidar_rpy[2]),
        'use_sim_time': use_sim_time,
    })
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' wheel_radius:=', str(geometry['wheel_radius_m']),
        ' wheel_width:=', str(geometry['wheel_width_m']),
        ' wheelbase:=', str(geometry['wheelbase_m']),
        ' track_width:=', str(geometry['track_width_m']),
        ' chassis_length:=', str(geometry['chassis_length_m']),
        ' chassis_width:=', str(geometry['chassis_width_m']),
        ' chassis_height:=', str(geometry['chassis_height_m']),
        ' chassis_x:=', str(chassis_xyz[0]),
        ' chassis_y:=', str(chassis_xyz[1]),
        ' chassis_z:=', str(chassis_xyz[2]),
        ' imu_x:=', str(imu_xyz[0]),
        ' imu_y:=', str(imu_xyz[1]),
        ' imu_z:=', str(imu_xyz[2]),
        ' imu_roll:=', str(imu_rpy[0]),
        ' imu_pitch:=', str(imu_rpy[1]),
        ' imu_yaw:=', str(imu_rpy[2]),
        ' lidar_x:=', str(lidar_xyz[0]),
        ' lidar_y:=', str(lidar_xyz[1]),
        ' lidar_z:=', str(lidar_xyz[2]),
        ' lidar_roll:=', str(lidar_rpy[0]),
        ' lidar_pitch:=', str(lidar_rpy[1]),
        ' lidar_yaw:=', str(lidar_rpy[2]),
    ]), value_type=str)

    if use_base:
        actions.append(Node(
            package='rover_base_driver',
            executable='base_driver_node',
            name='base_driver_node',
            output='screen',
            parameters=[base_params],
        ))

    if use_odometry:
        actions.append(Node(
            package='rover_wheel_odometry',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            parameters=[odom_params],
        ))

    if use_description:
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ))

    if use_lidar:
        actions.append(Node(
            package='rover_lidar_filter',
            executable='lidar_footprint_filter',
            name='lidar_footprint_filter',
            output='screen',
            parameters=[lidar_filter_params],
        ))

    if use_mux:
        mux_config = LaunchConfiguration('twist_mux_config_file').perform(context).strip()
        actions.append(Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_config],
            remappings=[('cmd_vel_out', topics.get('cmd_vel', '/cmd_vel'))],
        ))

    if (
        use_lidar
        or use_camera
        or use_vision
        or use_led_strip
        or use_octoliner
        or use_waveshare_audio
    ):
        peripheral_arguments = {
            'components_config_dir': components_dir,
            'topics_config_file': topics_config_file,
            'use_lidar': as_launch_bool(use_lidar),
            'use_camera': as_launch_bool(use_camera),
            'use_vision': as_launch_bool(use_vision),
            'use_led_strip': as_launch_bool(use_led_strip),
            'use_octoliner': as_launch_bool(use_octoliner),
            'use_waveshare_audio': as_launch_bool(use_waveshare_audio),
            'use_sim_time': as_launch_bool(use_sim_time),
        }
        if peripherals_config_file:
            peripheral_arguments['config_file'] = peripherals_config_file
        if use_lidar:
            detected_lidar = results['lidar']
            detected_lidar_params = dict(detected_lidar.parameters)
            peripheral_arguments.update({
                'lidar_device': str(Path(runtime_dir) / 'lidar'),
                'lidar_baudrate': str(detected_lidar.baudrate),
            })
            add_if_set(
                peripheral_arguments,
                'lidar_scan_mode',
                detected_lidar_params.get('scan_mode'),
            )
            add_if_set(
                peripheral_arguments,
                'lidar_scan_frequency',
                detected_lidar_params.get('scan_frequency'),
            )
            add_if_set(
                peripheral_arguments,
                'lidar_range_min',
                detected_lidar_params.get('range_min'),
            )
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rover_bringup'), 'launch', 'peripherals.launch.py'
            ])),
            launch_arguments=peripheral_arguments.items(),
        ))

    if use_web or use_display or use_rosboard:
        web_command_topic = (
            topics.get('cmd_vel_teleop', '/cmd_vel_teleop')
            if use_mux else topics.get('cmd_vel', '/cmd_vel')
        )
        ui_arguments = {
            'config_file': ui_config_file,
            'use_web': as_launch_bool(use_web),
            'use_display': as_launch_bool(use_display),
            'use_rosboard': as_launch_bool(use_rosboard),
            'command_topic': web_command_topic,
            'rover_config_file': robot_config_file,
            'rosboard_port': rosboard_port,
        }
        add_if_set(ui_arguments, 'display_panel_mode', display_panel_mode)
        add_if_set(ui_arguments, 'display_robot_serial', display_robot_serial)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rover_bringup'), 'launch', 'ui.launch.py'
            ])),
            launch_arguments=ui_arguments.items(),
        ))

    if use_imu:
        imu_params.update({
            'serial_device': str(Path(runtime_dir) / 'imu'),
            'baudrate': results['imu'].baudrate,
        })
        actions.append(Node(
            package='rover_imu',
            executable='yahboom_imu_node',
            name='yahboom_imu_node',
            output='screen',
            parameters=[imu_params],
        ))

    if use_localization:
        ekf_config = (
            LaunchConfiguration('ekf_with_imu_config_file').perform(context).strip()
            if use_imu
            else LaunchConfiguration('ekf_wheel_only_config_file').perform(context).strip()
        )
        actions.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', topics.get('odom', '/odom'))],
        ))

    if use_agent or use_fleet_bridge:
        agent_component = load_component(components_dir, 'agent')
        agent_command_topic = topics.get('agent_text_command', '/agent/text_command')
        agent_status_topic = topics.get('agent_status', '/agent/status')
        agent_answer_topic = topics.get('agent_answer', '/agent/answer')
        mcp_params = deep_merge(
            {
                'mcp_host': '127.0.0.1',
                'mcp_port': 8766,
                'cmd_vel_topic': topics.get('cmd_vel_test', '/cmd_vel_test'),
                'led_set_state_service': topics.get(
                    'led_set_state', '/led_strip/set_state'
                ),
                'led_state_topic': topics.get('led_state', '/led_strip/state'),
                'nav2_action_name': '/navigate_to_pose',
                'odom_topic': topics.get('odom', '/odom'),
                'amcl_pose_topic': topics.get('amcl_pose', '/amcl_pose'),
                'scan_topic': topics.get('scan', '/scan_filtered'),
            },
            dict(agent_component.get('mcp_server', {})),
        )
        mcp_params = resolve_references(mcp_params, reference_context)
        mcp_url = (
            f"http://127.0.0.1:{int(mcp_params.get('mcp_port', 8766))}/mcp"
        )
        set_if_blank(
            mcp_params,
            'cmd_vel_topic',
            topics.get('cmd_vel_test', '/cmd_vel_test'),
        )
        set_if_blank(
            mcp_params,
            'led_set_state_service',
            topics.get('led_set_state', '/led_strip/set_state'),
        )
        set_if_blank(
            mcp_params,
            'led_state_topic',
            topics.get('led_state', '/led_strip/state'),
        )
        set_if_blank(mcp_params, 'nav2_action_name', '/navigate_to_pose')
        set_if_blank(mcp_params, 'odom_topic', topics.get('odom', '/odom'))
        set_if_blank(
            mcp_params,
            'amcl_pose_topic',
            topics.get('amcl_pose', '/amcl_pose'),
        )
        set_if_blank(mcp_params, 'scan_topic', topics.get('scan', '/scan_filtered'))
        default_prompt_file = str(
            Path(get_package_share_directory('rover_agent_mcp'))
            / 'config'
            / 'default_system_prompt.md'
        )
        agent_reference_context = {
            **reference_context,
            'mcp': {'url': mcp_url},
            'paths': {'default_agent_prompt': default_prompt_file},
        }
        text_agent_params = deep_merge(
            {
                'robot_id': robot_id,
                'mcp_url': mcp_url,
                'text_command_topic': agent_command_topic,
                'status_topic': agent_status_topic,
                'answer_topic': agent_answer_topic,
                'prompt_file': default_prompt_file,
                'llm_base_url': '',
                'llm_model': '',
                'llm_api_key_env': 'OPENAI_API_KEY',
                'native_tool_mode': 'auto',
                'timeout_s': 120.0,
                'max_tool_rounds': 8,
            },
            dict(agent_component.get('text_agent', {})),
        )
        text_agent_params = resolve_references(
            text_agent_params,
            agent_reference_context,
        )
        if not str(text_agent_params.get('mcp_url', '')).strip():
            text_agent_params['mcp_url'] = mcp_url
        if not str(text_agent_params.get('prompt_file', '')).strip():
            text_agent_params['prompt_file'] = default_prompt_file
        set_if_blank(text_agent_params, 'robot_id', robot_id)
        set_if_blank(text_agent_params, 'text_command_topic', agent_command_topic)
        set_if_blank(text_agent_params, 'status_topic', agent_status_topic)
        set_if_blank(text_agent_params, 'answer_topic', agent_answer_topic)

        if use_agent:
            actions.append(Node(
                package='rover_agent_mcp',
                executable='rover_mcp_server',
                name='rover_mcp_server',
                output='screen',
                parameters=[mcp_params],
            ))
            actions.append(Node(
                package='rover_agent_mcp',
                executable='agent_text_node',
                name='rover_agent_text_node',
                output='screen',
                parameters=[text_agent_params],
            ))

        if use_fleet_bridge:
            fleet_bridge_params = deep_merge(
                {
                    'robot_id': robot_id,
                    'mqtt_host': '127.0.0.1',
                    'mqtt_port': 1883,
                    'mqtt_topic_prefix': 'fleet/v1/robots',
                    'mqtt_username': '',
                    'mqtt_password_env': 'FLEET_MQTT_PASSWORD',
                    'command_topic': agent_command_topic,
                    'answer_topic': agent_answer_topic,
                    'status_topic': agent_status_topic,
                    'duplicate_cache_size': 100,
                    'agent_command_timeout_sec': 300.0,
                },
                dict(agent_component.get('fleet_bridge', {})),
            )
            fleet_bridge_params = resolve_references(
                fleet_bridge_params,
                agent_reference_context,
            )
            set_if_blank(fleet_bridge_params, 'robot_id', robot_id)
            set_if_blank(fleet_bridge_params, 'command_topic', agent_command_topic)
            set_if_blank(fleet_bridge_params, 'answer_topic', agent_answer_topic)
            set_if_blank(fleet_bridge_params, 'status_topic', agent_status_topic)
            actions.append(Node(
                package='fleet_text_bridge_ros2',
                executable='bridge_node',
                name='fleet_text_bridge',
                output='screen',
                parameters=[fleet_bridge_params],
            ))
    if use_nav2:
        if not nav2_map_file:
            nav2_map_file = str(
                Path(get_package_share_directory('rover_navigation'))
                / 'maps'
                / 'current'
                / 'map.yaml'
            )
        actions.append(TimerAction(
            period=nav2_start_delay,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('rover_navigation'),
                    'launch',
                    'navigation.launch.py',
                ])),
                launch_arguments={
                    'use_sim_time': as_launch_bool(use_sim_time),
                    'map': nav2_map_file,
                    'params_file': nav2_params_file,
                    'use_rviz': as_launch_bool(use_rviz),
                }.items(),
            )],
        ))

    if use_slam:
        actions.append(TimerAction(
            period=slam_start_delay,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('rover_navigation'),
                    'launch',
                    'slam.launch.py',
                ])),
                launch_arguments={
                    'use_sim_time': as_launch_bool(use_sim_time),
                    'params_file': slam_params_file,
                    'use_rviz': as_launch_bool(use_rviz),
                }.items(),
            )],
        ))
    return actions


def generate_launch_description():
    empty_default = ''
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='full'),
        DeclareLaunchArgument('profile_file', default_value=empty_default),
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=bringup_config_path('rover_v1.yaml'),
        ),
        # Deprecated compatibility alias for older commands.
        DeclareLaunchArgument('config_file', default_value=empty_default),
        DeclareLaunchArgument(
            'components_config_dir',
            default_value=bringup_config_path('components'),
        ),
        DeclareLaunchArgument(
            'topics_config_file',
            default_value=bringup_config_path('topics.yaml'),
        ),
        # Deprecated compatibility hook for the former monolithic peripheral file.
        DeclareLaunchArgument('peripherals_config_file', default_value=empty_default),
        DeclareLaunchArgument('ui_config_file', default_value=empty_default),
        DeclareLaunchArgument(
            'twist_mux_config_file',
            default_value=bringup_config_path('components', 'twist_mux.yaml'),
        ),
        DeclareLaunchArgument(
            'ekf_with_imu_config_file',
            default_value=bringup_config_path('localization', 'ekf_with_imu.yaml'),
        ),
        DeclareLaunchArgument(
            'ekf_wheel_only_config_file',
            default_value=bringup_config_path('localization', 'ekf_wheel_only.yaml'),
        ),
        DeclareLaunchArgument('runtime_dir', default_value=empty_default),
        DeclareLaunchArgument(
            'device_config',
            default_value=empty_default,
            description='Persistent device setup JSON file',
        ),
        DeclareLaunchArgument(
            'discovery_mode',
            default_value=empty_default,
            description='configured (fast), verify, or full',
        ),
        DeclareLaunchArgument('use_base', default_value=empty_default),
        DeclareLaunchArgument('use_odometry', default_value=empty_default),
        DeclareLaunchArgument('use_description', default_value=empty_default),
        DeclareLaunchArgument('use_localization', default_value=empty_default),
        DeclareLaunchArgument('use_imu', default_value=empty_default),
        DeclareLaunchArgument('use_lidar', default_value=empty_default),
        DeclareLaunchArgument('use_camera', default_value=empty_default),
        DeclareLaunchArgument('use_vision', default_value=empty_default),
        DeclareLaunchArgument('use_display', default_value=empty_default),
        DeclareLaunchArgument(
            'display_panel_mode',
            default_value=empty_default,
            description='Optional touchscreen right panel override: placeholder or agent',
        ),
        DeclareLaunchArgument(
            'display_robot_serial',
            default_value=empty_default,
            description='Optional touchscreen rover serial suffix',
        ),
        DeclareLaunchArgument('use_led_strip', default_value=empty_default),
        DeclareLaunchArgument('use_octoliner', default_value=empty_default),
        DeclareLaunchArgument('use_waveshare_audio', default_value=empty_default),
        DeclareLaunchArgument('use_web', default_value=empty_default),
        DeclareLaunchArgument('use_rosboard', default_value=empty_default),
        DeclareLaunchArgument('use_agent', default_value=empty_default),
        DeclareLaunchArgument('use_fleet_bridge', default_value=empty_default),
        DeclareLaunchArgument('use_nav2', default_value=empty_default),
        DeclareLaunchArgument('use_slam', default_value=empty_default),
        DeclareLaunchArgument('rosboard_port', default_value='8888'),
        DeclareLaunchArgument('use_twist_mux', default_value=empty_default),
        DeclareLaunchArgument('use_sim_time', default_value=empty_default),
        DeclareLaunchArgument('use_rviz', default_value=empty_default),
        DeclareLaunchArgument(
            'map',
            default_value=empty_default,
            description='Map YAML used when Nav2 is enabled',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=bringup_config_path('navigation', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=bringup_config_path('navigation', 'slam_toolbox_params.yaml'),
        ),
        DeclareLaunchArgument('nav2_start_delay', default_value='2.0'),
        DeclareLaunchArgument('slam_start_delay', default_value='2.0'),
        DeclareLaunchArgument('motor_device', default_value=empty_default),
        DeclareLaunchArgument('imu_device', default_value=empty_default),
        DeclareLaunchArgument('lidar_device', default_value=empty_default),
        OpaqueFunction(function=launch_setup),
    ])
