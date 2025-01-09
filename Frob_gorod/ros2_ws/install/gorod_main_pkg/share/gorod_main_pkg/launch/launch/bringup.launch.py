from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Узел для работы с Arduino
        Node(
            package='ros2_arduino_bridge',
            executable='arduino_bridge',
            name='arduino_bridge_node',
            arguments=['/dev/ttyACM0'],
            output='screen'
        ),
        #Узел для публикации изображений
        Node(
            package='image_processor',
            executable='image_processor',
            name='image_processor_node',
            output='screen'
        ),
        # Публикация числа 3 в топик /robot_state
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',  '/task/state', 'std_msgs/msg/Int32', '{data: 1}'],
            output='screen'
        )
    ])
