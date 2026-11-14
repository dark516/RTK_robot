import os
from launch import LaunchDescription
from launch_ros.actions import Node

def i(name, default):
    try: return int(os.getenv(name, str(default)))
    except ValueError: return default

def f(name, default):
    try: return float(os.getenv(name, str(default)))
    except ValueError: return default

def generate_launch_description():
    return LaunchDescription([Node(
        package='fleet_text_bridge_ros2', executable='bridge_node', name='fleet_text_bridge', output='screen',
        parameters=[{
            'robot_id': os.getenv('FLEET_ROBOT_ID', 'rover-01'),
            'mqtt_host': os.getenv('FLEET_MQTT_HOST', os.getenv('FLEET_SERVER_IP', '127.0.0.1')),
            'mqtt_port': i('FLEET_MQTT_PORT', 1883),
            'mqtt_topic_prefix': os.getenv('FLEET_MQTT_TOPIC_PREFIX', 'fleet/v1/robots'),
            'mqtt_username': os.getenv('FLEET_MQTT_USERNAME', ''),
            'mqtt_password_env': os.getenv('FLEET_MQTT_PASSWORD_ENV', 'FLEET_MQTT_PASSWORD'),
            'command_topic': os.getenv('AGENT_TEXT_COMMAND_TOPIC', '/agent/text_command'),
            'answer_topic': os.getenv('AGENT_ANSWER_TOPIC', '/agent/answer'),
            'status_topic': os.getenv('AGENT_STATUS_TOPIC', '/agent/status'),
            'duplicate_cache_size': i('FLEET_DUPLICATE_CACHE_SIZE', 100),
            'agent_command_timeout_sec': f('FLEET_AGENT_COMMAND_TIMEOUT_SEC', 300.0),
        }]
    )])
