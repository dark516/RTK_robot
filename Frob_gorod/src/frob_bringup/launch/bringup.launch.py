from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Узел ros2_arduino_bridge
    arduino_bridge = Node(
        package='ros2_arduino_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        # The lidar is /dev/ttyUSB0. Explicitly pin the Arduino so the bridge
        # never probes and corrupts the lidar serial stream during reconnects.
        arguments=['--port', '/dev/ttyACM0']
    )
    #Lidar YDLIDAR X4
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
        )
    )
    lidar_filter = Node(
        package='lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter',
        output='screen'
    )
    #Imu mpu6050
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_mpu6050'), 'launch', 'ros2_mpu6050.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('include_imu')),
    )

    #Description
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frob_description'), 'launch', 'description.launch.py')
        )
    )

    #Odometry
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frob_odometry'), 'launch', 'odometry.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'include_imu',
            default_value='true',
            description='Start MPU6050 driver; disable when the sensor is unavailable',
        ),
        arduino_bridge,
        lidar_launch,
        imu_launch,
        description_launch,
        odometry_launch,
        lidar_filter
    ])
