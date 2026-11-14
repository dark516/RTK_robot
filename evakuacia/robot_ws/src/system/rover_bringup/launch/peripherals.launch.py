from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_bringup.configuration import (
    as_bool,
    bringup_config_path,
    load_component,
    override_bool,
    read_yaml_file,
)


def config_value(config: dict, keys: tuple[str, ...], default):
    value = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def launch_value(context, name: str, config: dict, keys: tuple[str, ...], default):
    override = LaunchConfiguration(name).perform(context).strip()
    if override:
        return override
    return config_value(config, keys, default)


def as_int(value) -> int:
    return int(str(value).strip())


def as_float(value) -> float:
    return float(str(value).strip())


def component_section(components_dir: str, name: str, section: str) -> dict:
    config = load_component(components_dir, name)
    value = config.get(section, {})
    return dict(value) if isinstance(value, dict) else {}


def configured_section(
    legacy_config: dict,
    components_dir: str,
    component_name: str,
    section: str,
) -> dict:
    if legacy_config and section in legacy_config:
        return dict(legacy_config.get(section, {}))
    return component_section(components_dir, component_name, section)


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context).strip()
    legacy_config = read_yaml_file(config_file) if config_file else {}
    components_dir = (
        LaunchConfiguration('components_config_dir').perform(context).strip()
        or bringup_config_path('components')
    )
    topics_config_file = (
        LaunchConfiguration('topics_config_file').perform(context).strip()
        or bringup_config_path('topics.yaml')
    )
    topics_config = read_yaml_file(topics_config_file)
    topics = dict(topics_config.get('topics', {}))
    frames = dict(topics_config.get('frames', {}))
    use_sim_time = as_bool(LaunchConfiguration('use_sim_time').perform(context))

    defaults = legacy_config.get('peripherals', {}) if legacy_config else {}
    use_lidar = override_bool(
        LaunchConfiguration('use_lidar').perform(context),
        as_bool(defaults.get('use_lidar', True)),
    )
    use_camera = override_bool(
        LaunchConfiguration('use_camera').perform(context),
        as_bool(defaults.get('use_camera', True)),
    )
    use_vision = override_bool(
        LaunchConfiguration('use_vision').perform(context),
        as_bool(defaults.get('use_vision', True)),
    )
    use_led_strip = override_bool(
        LaunchConfiguration('use_led_strip').perform(context),
        as_bool(defaults.get('use_led_strip', True)),
    )
    use_octoliner = override_bool(
        LaunchConfiguration('use_octoliner').perform(context),
        as_bool(defaults.get('use_octoliner', True)),
    )
    use_waveshare_audio = override_bool(
        LaunchConfiguration('use_waveshare_audio').perform(context),
        as_bool(defaults.get('use_waveshare_audio', False)),
    )

    actions = []
    lidar_config = configured_section(
        legacy_config, components_dir, 'lidar', 'lidar'
    )
    camera_params = configured_section(
        legacy_config, components_dir, 'camera', 'camera'
    )
    vision_params = configured_section(
        legacy_config, components_dir, 'vision', 'vision'
    )
    led_strip_params = configured_section(
        legacy_config, components_dir, 'led_strip', 'led_strip'
    )
    octoliner_params = configured_section(
        legacy_config, components_dir, 'octoliner', 'octoliner'
    )
    waveshare_audio_params = configured_section(
        legacy_config, components_dir, 'audio', 'waveshare_audio'
    )

    if (
        use_led_strip
        and use_octoliner
        and str(led_strip_params.get('led_transport', 'auto')).strip().lower()
        in {'auto', 'spi'}
    ):
        actions.append(LogInfo(
            msg=(
                '[INFO] LED strip is configured for SPI transport. Make sure the data '
                'wire is connected to the SPI MOSI pin that matches spi_bus/spi_device.'
            )
        ))

    if use_lidar:
        lidar_params = {
            'channel_type': str(lidar_config.get('channel_type', 'serial')),
            'serial_port': str(launch_value(
                context,
                'lidar_device',
                lidar_config,
                ('serial_port',),
                '/tmp/rover_devices/lidar',
            )),
            'serial_baudrate': as_int(launch_value(
                context,
                'lidar_baudrate',
                lidar_config,
                ('serial_baudrate',),
                460800,
            )),
            'frame_id': str(launch_value(
                context,
                'lidar_frame_id',
                lidar_config,
                ('frame_id',),
                frames.get('lidar', 'lidar_link'),
            )),
            'inverted': as_bool(launch_value(
                context,
                'lidar_inverted',
                lidar_config,
                ('inverted',),
                False,
            )),
            'angle_compensate': as_bool(launch_value(
                context,
                'lidar_angle_compensate',
                lidar_config,
                ('angle_compensate',),
                True,
            )),
            'scan_mode': str(launch_value(
                context,
                'lidar_scan_mode',
                lidar_config,
                ('scan_mode',),
                'Standard',
            )),
            'scan_frequency': as_float(launch_value(
                context,
                'lidar_scan_frequency',
                lidar_config,
                ('scan_frequency',),
                10.0,
            )),
            'range_min': as_float(launch_value(
                context,
                'lidar_range_min',
                lidar_config,
                ('range_min',),
                0.17,
            )),
            'use_sim_time': use_sim_time,
        }
        actions.append(Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[lidar_params],
            remappings=[('scan', topics.get('scan_raw', '/scan'))],
        ))

    if use_camera:
        camera_params['image_topic'] = topics.get(
            'image_raw',
            camera_params.get('image_topic', '/image_raw'),
        )
        camera_params['compressed_image_topic'] = topics.get(
            'image_raw_compressed',
            camera_params.get('compressed_image_topic', '/image_raw/compressed'),
        )
        camera_params['frame_id'] = frames.get(
            'camera',
            camera_params.get('frame_id', 'camera_optical_frame'),
        )
        camera_params['use_sim_time'] = use_sim_time
        actions.append(Node(
            package='rover_camera',
            executable='usb_camera_node',
            name='usb_camera_node',
            output='screen',
            additional_env={
                # OpenCV on Raspberry Pi is installed from apt and is built
                # against the distro NumPy ABI. Whisper dependencies in
                # ~/.local can install NumPy 2.x and break cv2 imports.
                'PYTHONNOUSERSITE': '1',
            },
            parameters=[camera_params],
        ))

    if use_camera and use_vision:
        vision_params['input_topic'] = topics.get(
            'image_raw',
            vision_params.get('input_topic', camera_params['image_topic']),
        )
        vision_params['frame_id'] = frames.get(
            'camera',
            vision_params.get('frame_id', camera_params['frame_id']),
        )
        vision_params['processed_image_topic'] = topics.get(
            'image_processed',
            vision_params.get('processed_image_topic', '/image_processed'),
        )
        vision_params['processed_compressed_image_topic'] = topics.get(
            'image_processed_compressed',
            vision_params.get(
                'processed_compressed_image_topic',
                '/image_processed/compressed',
            ),
        )
        vision_params['detections_topic'] = topics.get(
            'detections',
            vision_params.get('detections_topic', '/detections'),
        )
        vision_params['use_sim_time'] = use_sim_time
        actions.append(Node(
            package='rover_vision',
            executable='camera_detector_node',
            name='camera_detector_node',
            output='screen',
            additional_env={
                # Keep cv2 on the same NumPy ABI as the system package.
                'PYTHONNOUSERSITE': '1',
            },
            parameters=[vision_params],
        ))

    if use_led_strip:
        led_strip_params['frame_id'] = frames.get(
            'led_strip',
            led_strip_params.get('frame_id', 'led_strip'),
        )
        led_strip_params['state_topic'] = topics.get(
            'led_state',
            led_strip_params.get('state_topic', '/led_strip/state'),
        )
        led_strip_params['set_state_service'] = topics.get(
            'led_set_state',
            led_strip_params.get('set_state_service', '/led_strip/set_state'),
        )
        led_strip_params['use_sim_time'] = use_sim_time
        actions.append(Node(
            package='rover_led_strip',
            executable='led_strip_node',
            name='led_strip_node',
            output='screen',
            parameters=[led_strip_params],
        ))

    if use_octoliner:
        octoliner_params['frame_id'] = frames.get(
            'octoliner',
            octoliner_params.get('frame_id', 'octoliner_link'),
        )
        octoliner_params['use_sim_time'] = use_sim_time
        actions.append(Node(
            package='rover_octoliner',
            executable='octoliner_node',
            name='octoliner_node',
            output='screen',
            parameters=[octoliner_params],
        ))

    if use_waveshare_audio:
        waveshare_audio_params['output_topic'] = topics.get(
            'voice_text',
            waveshare_audio_params.get('output_topic', '/voice/text'),
        )
        waveshare_audio_params['status_topic'] = topics.get(
            'voice_status',
            waveshare_audio_params.get('status_topic', '/waveshare_audio/status'),
        )
        waveshare_audio_params['transcript_json_topic'] = topics.get(
            'voice_transcript',
            waveshare_audio_params.get('transcript_json_topic', '/voice/transcript'),
        )
        actions.append(Node(
            package='rover_waveshare_audio',
            executable='waveshare_audio_node',
            name='waveshare_audio_node',
            output='screen',
            parameters=[waveshare_audio_params],
        ))

    return actions


def generate_launch_description():
    empty_default = ''
    return LaunchDescription([
        # Deprecated compatibility: the former monolithic peripherals.yaml.
        DeclareLaunchArgument('config_file', default_value=empty_default),
        DeclareLaunchArgument(
            'components_config_dir',
            default_value=bringup_config_path('components'),
        ),
        DeclareLaunchArgument(
            'topics_config_file',
            default_value=bringup_config_path('topics.yaml'),
        ),
        DeclareLaunchArgument('use_lidar', default_value=empty_default),
        DeclareLaunchArgument('use_camera', default_value=empty_default),
        DeclareLaunchArgument('use_vision', default_value=empty_default),
        DeclareLaunchArgument('use_led_strip', default_value=empty_default),
        DeclareLaunchArgument('use_octoliner', default_value=empty_default),
        DeclareLaunchArgument('use_waveshare_audio', default_value=empty_default),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_device', default_value=empty_default),
        DeclareLaunchArgument('lidar_baudrate', default_value=empty_default),
        DeclareLaunchArgument('lidar_frame_id', default_value=empty_default),
        DeclareLaunchArgument('lidar_inverted', default_value=empty_default),
        DeclareLaunchArgument('lidar_angle_compensate', default_value=empty_default),
        DeclareLaunchArgument('lidar_scan_mode', default_value=empty_default),
        DeclareLaunchArgument('lidar_scan_frequency', default_value=empty_default),
        DeclareLaunchArgument('lidar_range_min', default_value=empty_default),
        OpaqueFunction(function=launch_setup),
    ])
