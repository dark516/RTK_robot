from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_config_file() -> str:
    return str(
        Path(get_package_share_directory('rover_waveshare_audio'))
        / 'config'
        / 'default.example.yaml'
    )


def maybe_override(context, name: str, cast=str):
    value = LaunchConfiguration(name).perform(context).strip()
    if not value:
        return None
    return cast(value)


def as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def launch_setup(context):
    overrides = {}
    for name, cast in (
        ('serial_device', str),
        ('baudrate', int),
        ('output_topic', str),
        ('status_topic', str),
        ('transcript_json_topic', str),
        ('whisper_model', str),
        ('language', str),
        ('device', str),
        ('min_rms', float),
        ('enable_tts', as_bool),
        ('tts_service_name', str),
        ('tts_engine', str),
        ('tts_voice', str),
        ('tts_rate', int),
        ('piper_module', str),
        ('piper_model', str),
        ('piper_data_dir', str),
    ):
        value = maybe_override(context, name, cast)
        if value is not None:
            overrides[name] = value

    parameters = [LaunchConfiguration('config_file')]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='rover_waveshare_audio',
            executable='waveshare_audio_node',
            name='waveshare_audio_node',
            output='screen',
            parameters=parameters,
        )
    ]


def generate_launch_description():
    empty_default = ''
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config_file()),
        DeclareLaunchArgument('serial_device', default_value=empty_default),
        DeclareLaunchArgument('baudrate', default_value=empty_default),
        DeclareLaunchArgument('output_topic', default_value=empty_default),
        DeclareLaunchArgument('status_topic', default_value=empty_default),
        DeclareLaunchArgument('transcript_json_topic', default_value=empty_default),
        DeclareLaunchArgument('whisper_model', default_value=empty_default),
        DeclareLaunchArgument('language', default_value=empty_default),
        DeclareLaunchArgument('device', default_value=empty_default),
        DeclareLaunchArgument('min_rms', default_value=empty_default),
        DeclareLaunchArgument('enable_tts', default_value=empty_default),
        DeclareLaunchArgument('tts_service_name', default_value=empty_default),
        DeclareLaunchArgument('tts_engine', default_value=empty_default),
        DeclareLaunchArgument('tts_voice', default_value=empty_default),
        DeclareLaunchArgument('tts_rate', default_value=empty_default),
        DeclareLaunchArgument('piper_module', default_value=empty_default),
        DeclareLaunchArgument('piper_model', default_value=empty_default),
        DeclareLaunchArgument('piper_data_dir', default_value=empty_default),
        OpaqueFunction(function=launch_setup),
    ])
