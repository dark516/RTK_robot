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
            arguments=['/dev/ttyUSB0'],
            output='screen'
        ),
        # Узел для публикации изображений
#        Node(
 #           package='image_publisher_pkg',
  #          executable='image_publisher_node',
   #         name='image_publisher_node',
    #        output='screen'
     #   ),
        # Публикация числа 3 в топик /robot_state
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',  '/robot_state', 'std_msgs/msg/Int32', '{data: 3}'],
            output='screen'
        )
    ])
