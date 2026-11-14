import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def i(name, default):
    try: return int(os.getenv(name, str(default)))
    except ValueError: return default

def f(name, default):
    try: return float(os.getenv(name, str(default)))
    except ValueError: return default

def generate_launch_description():
    share = get_package_share_directory('rover_agent_mcp')
    mcp_port = i('MCP_PORT', 8765)
    default_prompt = os.path.join(share, 'config', 'default_system_prompt.md')
    return LaunchDescription([
        Node(package='rover_agent_mcp', executable='rover_mcp_server', name='rover_mcp_server', output='screen', parameters=[{
            'mcp_host': os.getenv('MCP_HOST', '127.0.0.1'),
            'mcp_port': mcp_port,
            'cmd_vel_topic': os.getenv('ROVER_CMD_VEL_TOPIC', '/cmd_vel_test'),
            'led_set_state_service': os.getenv('ROVER_LED_SERVICE', '/led_strip/set_state'),
            'led_state_topic': os.getenv('ROVER_LED_STATE_TOPIC', '/led_strip/state'),
            'nav2_action_name': os.getenv('ROVER_NAV_ACTION', '/navigate_to_pose'),
            'odom_topic': os.getenv('ROVER_ODOM_TOPIC', '/odom'),
            'amcl_pose_topic': os.getenv('ROVER_AMCL_POSE_TOPIC', '/amcl_pose'),
            'scan_topic': os.getenv('ROVER_SCAN_TOPIC', '/scan_filtered'),
        }]),
        Node(package='rover_agent_mcp', executable='agent_text_node', name='rover_agent_text_node', output='screen', parameters=[{
            'robot_id': os.getenv('FLEET_ROBOT_ID', 'rover-01'),
            'mcp_url': os.getenv('MCP_URL', f'http://127.0.0.1:{mcp_port}/mcp'),
            'text_command_topic': os.getenv('AGENT_TEXT_COMMAND_TOPIC', '/agent/text_command'),
            'status_topic': os.getenv('AGENT_STATUS_TOPIC', '/agent/status'),
            'answer_topic': os.getenv('AGENT_ANSWER_TOPIC', '/agent/answer'),
            'prompt_file': os.getenv('AGENT_PROMPT_FILE', default_prompt),
            'llm_base_url': os.getenv('OPENAI_BASE_URL', os.getenv('OPENROUTER_BASE_URL', os.getenv('SVERK_BASE_URL', ''))),
            'llm_model': os.getenv('OPENAI_MODEL', os.getenv('OPENROUTER_MODEL', os.getenv('SVERK_MODEL', ''))),
            'llm_api_key_env': os.getenv('LLM_API_KEY_ENV', 'OPENAI_API_KEY'),
            'native_tool_mode': os.getenv('LLM_NATIVE_TOOL_MODE', 'auto'),
            'timeout_s': f('LLM_TIMEOUT_SEC', 120.0),
            'max_tool_rounds': i('LLM_MAX_TOOL_ROUNDS', 8),
        }]),
    ])
