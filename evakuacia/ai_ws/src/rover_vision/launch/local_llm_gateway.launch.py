from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('rover_vision')) / 'config' / 'local_llm_gateway.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(config)),
        DeclareLaunchArgument('llama_server', default_value='~/.local/bin/llama-server'),
        DeclareLaunchArgument('models_dir', default_value='~/.local/share/rtk-evacuation/models/llm'),
        DeclareLaunchArgument('default_model', default_value='qwen3.5-9b'),
        DeclareLaunchArgument('device', default_value='Vulkan0'),
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(
            package='rover_vision',
            executable='local_llm_gateway_node',
            name='local_llm_gateway_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file'), {
                name: LaunchConfiguration(name)
                for name in ('llama_server', 'models_dir', 'default_model', 'device')
            }, {
                'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
            }],
        ),
    ])
