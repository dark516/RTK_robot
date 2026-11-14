from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

from rover_bringup.configuration import bringup_config_path


def default_ui_config_file() -> str:
    return bringup_config_path('components', 'ui.yaml')


def package_file(package_name: str, *parts: str) -> str:
    return str(Path(get_package_share_directory(package_name)).joinpath(*parts))


def workspace_root() -> str:
    try:
        return str(Path(get_package_share_directory('rover_web')).parents[3])
    except Exception:
        return str(Path.home() / 'sverk_rover')


def default_plans_directory() -> str:
    return str(Path.home() / '.local' / 'share' / 'sverh-rover-web' / 'plans')


def default_hackathon_files_root() -> str:
    return str(Path(workspace_root()) / 'hackathon_files')


def load_ui_config(path: str) -> dict:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise FileNotFoundError(f'UI config file not found: {config_path}')
    with config_path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def config_value(config: dict, keys: tuple[str, ...], default):
    value = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return to_launch_text(default)
        value = value[key]
    return to_launch_text(value)


def launch_value(context, name: str, config: dict, keys: tuple[str, ...], default):
    override = LaunchConfiguration(name).perform(context).strip()
    if override:
        return override
    return config_value(config, keys, default)


def launch_value_or_default(
    context,
    name: str,
    config: dict,
    keys: tuple[str, ...],
    default,
):
    value = launch_value(context, name, config, keys, default).strip()
    return value if value else to_launch_text(default)


def to_launch_text(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None:
        return ''
    return str(value)


def add_if_set(arguments: dict, name: str, value: str):
    if value:
        arguments[name] = value


def launch_setup(context):
    config_file = LaunchConfiguration('config_file').perform(context).strip()
    config = load_ui_config(config_file) if config_file else {}

    use_web = launch_value(context, 'use_web', config, ('ui', 'use_web'), True)
    use_display = launch_value(
        context, 'use_display', config, ('ui', 'use_display'), True
    )
    use_rosboard = launch_value(
        context, 'use_rosboard', config, ('ui', 'use_rosboard'), True
    )

    web_arguments = {
        'config_file': launch_value_or_default(
            context,
            'web_config_file',
            config,
            ('web', 'config_file'),
            package_file('rover_bringup', 'config', 'components', 'web.yaml'),
        ),
        'bind_address': launch_value(
            context, 'web_bind_address', config, ('web', 'bind_address'), '0.0.0.0'
        ),
        'port': launch_value(context, 'web_port', config, ('web', 'port'), 8765),
        'command_topic': launch_value(
            context, 'command_topic', config, ('web', 'command_topic'), '/cmd_vel'
        ),
        'identity_file': launch_value_or_default(
            context,
            'identity_file',
            config,
            ('web', 'identity_file'),
            package_file('rover_bringup', 'config', 'rover_v1.yaml'),
        ),
        'terminal_enabled': launch_value(
            context, 'terminal_enabled', config, ('terminal', 'enabled'), True
        ),
        'start_terminal': launch_value(
            context, 'start_terminal', config, ('terminal', 'start'), True
        ),
        'terminal_bind_address': launch_value(
            context,
            'terminal_bind_address',
            config,
            ('terminal', 'bind_address'),
            '0.0.0.0',
        ),
        'terminal_port': launch_value(
            context, 'terminal_port', config, ('terminal', 'port'), 7681
        ),
        'terminal_path': launch_value(
            context, 'terminal_path', config, ('terminal', 'path'), '/'
        ),
        'rosboard_enabled': use_rosboard,
        'rosboard_port': launch_value(
            context, 'rosboard_port', config, ('rosboard', 'port'), 8888
        ),
        'rover_config_file': launch_value_or_default(
            context,
            'rover_config_file',
            config,
            ('web', 'rover_config_file'),
            package_file('rover_bringup', 'config', 'rover_v1.yaml'),
        ),
        'plans_directory': launch_value_or_default(
            context,
            'plans_directory',
            config,
            ('web', 'plans_directory'),
            default_plans_directory(),
        ),
        'hackathon_files_root': launch_value_or_default(
            context,
            'hackathon_files_root',
            config,
            ('web', 'hackathon_files_root'),
            default_hackathon_files_root(),
        ),
        'terminal_url': launch_value(
            context, 'terminal_url', config, ('terminal', 'url'), ''
        ),
        'terminal_workspace': launch_value_or_default(
            context,
            'terminal_workspace',
            config,
            ('terminal', 'workspace'),
            workspace_root(),
        ),
    }

    display_arguments = {
        'config_file': launch_value_or_default(
            context,
            'display_config_file',
            config,
            ('display', 'config_file'),
            package_file('rover_bringup', 'config', 'components', 'display.yaml'),
        ),
        'right_panel_mode': launch_value(
            context,
            'display_panel_mode',
            config,
            ('display', 'panel_mode'),
            'placeholder',
        ),
        'robot_serial': launch_value(
            context, 'display_robot_serial', config, ('display', 'robot_serial'), '1'
        ),
    }
    add_if_set(
        display_arguments,
        'agent_text_topic',
        launch_value(
            context,
            'display_agent_text_topic',
            config,
            ('display', 'agent_text_topic'),
            '',
        ),
    )
    add_if_set(
        display_arguments,
        'battery_topic',
        launch_value(
            context,
            'display_battery_topic',
            config,
            ('display', 'battery_topic'),
            '',
        ),
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rover_web'), 'launch', 'web.launch.py'
            ])),
            condition=IfCondition(TextSubstitution(text=use_web)),
            launch_arguments=web_arguments.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rosboard'), 'launch', 'rosboard.launch.py'
            ])),
            condition=IfCondition(TextSubstitution(text=use_rosboard)),
            launch_arguments={
                'port': launch_value(
                    context, 'rosboard_port', config, ('rosboard', 'port'), 8888
                ),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rover_display'), 'launch', 'display.launch.py'
            ])),
            condition=IfCondition(TextSubstitution(text=use_display)),
            launch_arguments=display_arguments.items(),
        ),
    ]


def generate_launch_description():
    empty_default = ''

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_ui_config_file()),
        DeclareLaunchArgument('use_web', default_value=empty_default),
        DeclareLaunchArgument('use_display', default_value=empty_default),
        DeclareLaunchArgument('use_rosboard', default_value=empty_default),
        DeclareLaunchArgument('web_config_file', default_value=empty_default),
        DeclareLaunchArgument('web_bind_address', default_value=empty_default),
        DeclareLaunchArgument('web_port', default_value=empty_default),
        DeclareLaunchArgument('command_topic', default_value=empty_default),
        DeclareLaunchArgument('identity_file', default_value=empty_default),
        DeclareLaunchArgument('rover_config_file', default_value=empty_default),
        DeclareLaunchArgument('plans_directory', default_value=empty_default),
        DeclareLaunchArgument('hackathon_files_root', default_value=empty_default),
        DeclareLaunchArgument('terminal_enabled', default_value=empty_default),
        DeclareLaunchArgument('start_terminal', default_value=empty_default),
        DeclareLaunchArgument('terminal_bind_address', default_value=empty_default),
        DeclareLaunchArgument('terminal_port', default_value=empty_default),
        DeclareLaunchArgument('terminal_path', default_value=empty_default),
        DeclareLaunchArgument('terminal_url', default_value=empty_default),
        DeclareLaunchArgument('terminal_workspace', default_value=empty_default),
        DeclareLaunchArgument('rosboard_port', default_value=empty_default),
        DeclareLaunchArgument('display_config_file', default_value=empty_default),
        DeclareLaunchArgument('display_agent_text_topic', default_value=empty_default),
        DeclareLaunchArgument('display_battery_topic', default_value=empty_default),
        DeclareLaunchArgument(
            'display_panel_mode',
            default_value=empty_default,
            description='Touchscreen right panel: placeholder or agent',
        ),
        DeclareLaunchArgument(
            'display_robot_serial',
            default_value=empty_default,
            description='Touchscreen rover serial suffix',
        ),
        OpaqueFunction(function=launch_setup),
    ])
