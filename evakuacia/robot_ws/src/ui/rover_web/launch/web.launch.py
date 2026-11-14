from pathlib import Path
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_rover_config_file() -> str:
    try:
        return str(
            Path(get_package_share_directory('rover_bringup'))
            / 'config'
            / 'rover_v1.yaml'
        )
    except Exception:
        return ''


def default_workspace_root(web_share: Path) -> str:
    try:
        return str(web_share.parents[3])
    except Exception:
        return str(Path.home() / 'sverk_rover')


def as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def launch_setup(context, web_share_text: str, terminal_shell: str):
    web_share = Path(web_share_text)
    start_terminal = as_bool(
        LaunchConfiguration('start_terminal').perform(context)
    )
    terminal_enabled = as_bool(
        LaunchConfiguration('terminal_enabled').perform(context)
    )
    terminal_url = LaunchConfiguration('terminal_url').perform(context).strip()
    actions = []

    if terminal_enabled and start_terminal:
        ttyd_path = shutil.which('ttyd')
        if ttyd_path:
            actions.append(ExecuteProcess(
                cmd=[
                    ttyd_path,
                    '-i',
                    LaunchConfiguration('terminal_bind_address'),
                    '-p',
                    LaunchConfiguration('terminal_port'),
                    '-W',
                    '/bin/bash',
                    terminal_shell,
                    LaunchConfiguration('terminal_workspace'),
                ],
                output='screen',
            ))
        elif terminal_url:
            actions.append(LogInfo(
                msg='[WARN] ttyd executable not found; using configured terminal_url.'
            ))
        else:
            terminal_enabled = False
            actions.append(LogInfo(
                msg=(
                    '[WARN] ttyd executable not found; web terminal disabled. '
                    'Install ttyd or launch with start_terminal:=false.'
                )
            ))

    actions.append(Node(
        package='rover_web',
        executable='web_gateway_node',
        name='web_gateway_node',
        output='screen',
        additional_env={
            # Keep ROS/OpenCV on the distro-provided NumPy ABI. User-site
            # packages installed for Whisper can pull NumPy 2.x and break
            # cv2, which is built against Ubuntu/ROS NumPy 1.x on the Pi.
            'PYTHONNOUSERSITE': '1',
        },
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'bind_address': LaunchConfiguration('bind_address'),
                'port': ParameterValue(
                    LaunchConfiguration('port'),
                    value_type=int,
                ),
                'command_topic': LaunchConfiguration('command_topic'),
                'rover_config_file': LaunchConfiguration('rover_config_file'),
                'plans_directory': LaunchConfiguration('plans_directory'),
                'hackathon_files_root': LaunchConfiguration('hackathon_files_root'),
                'terminal_enabled': terminal_enabled,
                'terminal_url': terminal_url,
                'terminal_port': ParameterValue(
                    LaunchConfiguration('terminal_port'),
                    value_type=int,
                ),
                'terminal_path': LaunchConfiguration('terminal_path'),
                'rosboard_enabled': ParameterValue(
                    LaunchConfiguration('rosboard_enabled'),
                    value_type=bool,
                ),
                'rosboard_port': ParameterValue(
                    LaunchConfiguration('rosboard_port'),
                    value_type=int,
                ),
                'identity_file': LaunchConfiguration('identity_file'),
                'web_root': str(web_share / 'web'),
                'motion_executor_path': str(
                    web_share / 'tools' / 'rover_motion_executor.py'
                ),
                'seed_plans_directory': str(web_share / 'plans'),
            },
        ],
    ))
    return actions


def generate_launch_description():
    web_share = Path(get_package_share_directory('rover_web'))
    terminal_shell = str(web_share / 'tools' / 'rover_terminal_shell.sh')
    actions = [
        DeclareLaunchArgument(
            'config_file',
            default_value=str(web_share / 'config' / 'default.example.yaml'),
        ),
        DeclareLaunchArgument('bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('command_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'identity_file',
            default_value=default_rover_config_file(),
        ),
        DeclareLaunchArgument(
            'rover_config_file',
            default_value=default_rover_config_file(),
        ),
        DeclareLaunchArgument(
            'plans_directory',
            default_value=str(
                Path.home() / '.local' / 'share' / 'sverh-rover-web' / 'plans'
            ),
        ),
        DeclareLaunchArgument(
            'hackathon_files_root',
            default_value=str(
                Path(default_workspace_root(web_share)) / 'hackathon_files'
            ),
        ),
        DeclareLaunchArgument('terminal_enabled', default_value='true'),
        DeclareLaunchArgument('start_terminal', default_value='true'),
        DeclareLaunchArgument('terminal_url', default_value=''),
        DeclareLaunchArgument('terminal_bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('terminal_port', default_value='7681'),
        DeclareLaunchArgument('terminal_path', default_value='/'),
        DeclareLaunchArgument('rosboard_enabled', default_value='true'),
        DeclareLaunchArgument('rosboard_port', default_value='8888'),
        DeclareLaunchArgument(
            'terminal_workspace',
            default_value=default_workspace_root(web_share),
        ),
        OpaqueFunction(
            function=launch_setup,
            args=[str(web_share), terminal_shell],
        ),
    ]

    return LaunchDescription(actions)
