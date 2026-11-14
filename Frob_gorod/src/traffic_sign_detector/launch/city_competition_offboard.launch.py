"""Laptop inference and TCP relay for a competition run."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = Path(get_package_share_directory('traffic_sign_detector'))
    vendor_path = LaunchConfiguration('ultralytics_vendor_path').perform(context).strip()
    detector_env = {}
    if vendor_path:
        vendor = Path(vendor_path).expanduser()
        if not vendor.is_dir():
            raise RuntimeError(f'Ultralytics vendor directory is missing: {vendor}')
        detector_env['PYTHONPATH'] = f"{vendor}:{os.environ.get('PYTHONPATH', '')}"
    detector = Node(
        package='traffic_sign_detector',
        executable='traffic_sign_detector',
        # Keep the configured node name: ROS parameter-file sections are keyed
        # by this exact name.
        name='traffic_sign_detector',
        output='screen',
        parameters=[
            str(share / 'config' / 'offboard_detector.yaml'),
            {
                'detections_topic': '/traffic_sign/detections_offboard',
                'yolo_device': LaunchConfiguration('yolo_device').perform(context),
            },
        ],
        additional_env=detector_env,
    )
    relay = Node(
        package='traffic_sign_detector',
        executable='traffic_sign_detection_tcp_server',
        name='traffic_sign_detection_tcp_server',
        output='screen',
        parameters=[{
            'input_topic': '/traffic_sign/detections_offboard',
            'listen_host': LaunchConfiguration('listen_host').perform(context),
            'listen_port': int(LaunchConfiguration('listen_port').perform(context)),
        }],
    )
    return [detector, relay]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ultralytics_vendor_path',
            default_value='',
            description='Optional directory containing ultralytics; otherwise use the Python environment',
        ),
        DeclareLaunchArgument('yolo_device', default_value='cpu'),
        DeclareLaunchArgument('listen_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('listen_port', default_value='18451'),
        OpaqueFunction(function=_setup),
    ])
