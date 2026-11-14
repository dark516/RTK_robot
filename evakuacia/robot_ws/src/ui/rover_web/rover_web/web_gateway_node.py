#!/usr/bin/env python3
"""Web gateway for rover status, ROS inspection, camera preview and drive."""

from __future__ import annotations

from array import array
from collections import deque
from dataclasses import dataclass
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, quote, unquote, urlparse

from ament_index_python.packages import get_package_share_directory
import cv2
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service
from rover_vision.model_registry import (
    discover_model_manifests,
    manifest_to_dict,
    resolve_models_directory,
)
from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import String
import yaml


MAX_REQUEST_BYTES = 1_000_000
IMAGE_TOPIC_TYPES = {
    'sensor_msgs/msg/Image',
    'sensor_msgs/msg/CompressedImage',
}
LASER_SCAN_TYPE = 'sensor_msgs/msg/LaserScan'
LED_STRIP_STATE_TYPE = 'rover_interfaces/msg/LedStripState'
OCTOLINER_READING_TYPE = 'rover_interfaces/msg/OctolinerReading'
PLAN_NAME_RE = re.compile(r'^[A-Za-z0-9_.-]+$')
V4L2_FMT_RE = re.compile(r"\[\d+\]: '([^']+)' \((.+)\)")
V4L2_SIZE_RE = re.compile(r'Size:\s+Discrete\s+(\d+)x(\d+)')
V4L2_INTERVAL_RE = re.compile(r'Interval:\s+Discrete\s+([0-9.]+)s\s+\(([0-9.]+)\s+fps\)')
HACKATHON_FILE_TYPES = {
    '.md': ('markdown', 'text/markdown; charset=utf-8'),
    '.markdown': ('markdown', 'text/markdown; charset=utf-8'),
    '.html': ('html', 'text/html; charset=utf-8'),
    '.htm': ('html', 'text/html; charset=utf-8'),
    '.pdf': ('pdf', 'application/pdf'),
}
MAP_YAML_EXTENSIONS = {'.yaml', '.yml'}
MAP_IMAGE_EXTENSIONS = {
    '.pgm',
    '.png',
    '.jpg',
    '.jpeg',
    '.bmp',
    '.tif',
    '.tiff',
}


def default_maps_root(workspace_root: Path) -> str:
    source_maps = (
        workspace_root
        / 'src'
        / 'motion'
        / 'rover_navigation'
        / 'maps'
        / 'current'
    )
    if source_maps.exists():
        return str(source_maps)
    try:
        return str(
            Path(get_package_share_directory('rover_navigation'))
            / 'maps'
            / 'current'
        )
    except Exception:
        return str(source_maps)


CAMERA_PARAMETER_NAMES = [
    'device',
    'image_topic',
    'compressed_image_topic',
    'frame_id',
    'width',
    'height',
    'fps',
    'use_mjpeg',
    'publish_raw',
    'publish_compressed',
    'jpeg_quality',
    'reconnect_interval_sec',
]
VISION_PARAMETER_NAMES = [
    'enabled',
    'model_name',
    'models_directory',
    'input_topic',
    'processed_image_topic',
    'processed_compressed_image_topic',
    'frame_id',
    'publish_raw',
    'publish_compressed',
    'publish_detections',
    'detections_topic',
    'confidence_threshold',
    'nms_threshold',
    'max_processing_fps',
    'annotate_labels',
    'annotate_confidence',
    'line_thickness',
    'jpeg_quality',
]
VISION_RUNTIME_PARAMETER_NAMES = {
    'enabled',
    'model_name',
    'models_directory',
    'input_topic',
    'processed_image_topic',
    'processed_compressed_image_topic',
    'frame_id',
    'publish_raw',
    'publish_compressed',
    'publish_detections',
    'detections_topic',
    'confidence_threshold',
    'nms_threshold',
    'max_processing_fps',
    'annotate_labels',
    'annotate_confidence',
    'line_thickness',
    'jpeg_quality',
}
LIDAR_PARAMETER_NAMES = [
    'channel_type',
    'serial_port',
    'serial_baudrate',
    'frame_id',
    'inverted',
    'angle_compensate',
    'scan_mode',
    'scan_frequency',
]
LIDAR_RUNTIME_PARAMETER_NAMES = {
    'channel_type',
    'serial_port',
    'serial_baudrate',
    'frame_id',
    'inverted',
    'angle_compensate',
    'scan_mode',
    'scan_frequency',
}
LED_STRIP_PARAMETER_NAMES = [
    'led_transport',
    'spi_bus',
    'spi_device',
    'led_count',
    'frame_id',
    'enabled',
    'brightness',
    'effect',
    'effect_speed_hz',
    'primary_color',
    'secondary_color',
    'animation_rate_hz',
    'state_publish_hz',
    'state_topic',
    'set_state_service',
    'native_state_topic',
    'set_effect_service',
    'set_leds_service',
]
LED_STRIP_RUNTIME_PARAMETER_NAMES = {
    'led_transport',
    'spi_bus',
    'spi_device',
    'led_count',
    'frame_id',
    'enabled',
    'brightness',
    'effect',
    'effect_speed_hz',
    'primary_color',
    'secondary_color',
    'animation_rate_hz',
    'state_publish_hz',
}
OCTOLINER_PARAMETER_NAMES = [
    'i2c_address',
    'i2c_bus',
    'poll_rate_hz',
    'frame_id',
    'sensitivity',
    'auto_optimize_on_start',
    'set_sensitivity_service',
    'optimize_on_black_service',
]
OCTOLINER_RUNTIME_PARAMETER_NAMES = {
    'poll_rate_hz',
    'frame_id',
    'sensitivity',
    'auto_optimize_on_start',
}


def current_ipv4_addresses() -> list[str]:
    addresses: list[str] = []
    try:
        output = subprocess.run(
            ['hostname', '-I'],
            check=False,
            capture_output=True,
            text=True,
            timeout=0.5,
        ).stdout
        for candidate in output.split():
            if ':' not in candidate and not candidate.startswith('127.'):
                addresses.append(candidate)
    except (OSError, subprocess.SubprocessError):
        pass
    return list(dict.fromkeys(addresses))


def age_seconds(monotonic_timestamp: float) -> float | None:
    if monotonic_timestamp <= 0.0:
        return None
    return max(0.0, time.monotonic() - monotonic_timestamp)


def normalize_topic_name(name: str) -> str:
    text = name.strip()
    if not text:
        raise ValueError('Topic name is required')
    return text if text.startswith('/') else f'/{text}'


def normalize_service_name(name: str) -> str:
    text = name.strip()
    if not text:
        raise ValueError('Service name is required')
    return text if text.startswith('/') else f'/{text}'


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def sanitize_payload(value: Any, *, depth: int = 0, max_items: int = 64) -> Any:
    if depth > 8:
        return '...'

    if isinstance(value, (str, int, float, bool)) or value is None:
        if isinstance(value, float) and not math.isfinite(value):
            return str(value)
        return value

    if isinstance(value, (bytes, bytearray, memoryview)):
        return f'<{len(value)} bytes>'

    if isinstance(value, array):
        return sanitize_payload(list(value), depth=depth + 1, max_items=max_items)

    if isinstance(value, dict):
        items = list(value.items())
        output = {
            str(key): sanitize_payload(item, depth=depth + 1, max_items=max_items)
            for key, item in items[:max_items]
        }
        if len(items) > max_items:
            output['__truncated__'] = f'{len(items) - max_items} more fields'
        return output

    if isinstance(value, (list, tuple)):
        items = [
            sanitize_payload(item, depth=depth + 1, max_items=max_items)
            for item in value[:max_items]
        ]
        if len(value) > max_items:
            items.append(f'... {len(value) - max_items} more items')
        return items

    if hasattr(value, 'tolist'):
        return sanitize_payload(value.tolist(), depth=depth + 1, max_items=max_items)

    return str(value)


def make_message_template(message_type: type[Any]) -> dict[str, Any]:
    return sanitize_payload(message_to_ordereddict(message_type()), max_items=24)


def compressed_content_type(format_text: str) -> str:
    lowered = format_text.lower()
    if 'png' in lowered:
        return 'image/png'
    return 'image/jpeg'


def parameter_value_to_python(value: Any) -> Any:
    type_code = int(getattr(value, 'type', 0))
    if type_code == Parameter.Type.BOOL.value:
        return bool(value.bool_value)
    if type_code == Parameter.Type.INTEGER.value:
        return int(value.integer_value)
    if type_code == Parameter.Type.DOUBLE.value:
        return float(value.double_value)
    if type_code == Parameter.Type.STRING.value:
        return str(value.string_value)
    if type_code == Parameter.Type.BOOL_ARRAY.value:
        return list(value.bool_array_value)
    if type_code == Parameter.Type.INTEGER_ARRAY.value:
        return [int(item) for item in value.integer_array_value]
    if type_code == Parameter.Type.DOUBLE_ARRAY.value:
        return [float(item) for item in value.double_array_value]
    if type_code == Parameter.Type.STRING_ARRAY.value:
        return list(value.string_array_value)
    return None


def parse_v4l2_capabilities(text: str) -> list[dict[str, Any]]:
    formats: list[dict[str, Any]] = []
    current_format: dict[str, Any] | None = None
    current_mode: dict[str, Any] | None = None

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        format_match = V4L2_FMT_RE.search(line)
        if format_match:
            current_format = {
                'pixel_format': format_match.group(1),
                'description': format_match.group(2),
                'modes': [],
            }
            formats.append(current_format)
            current_mode = None
            continue

        size_match = V4L2_SIZE_RE.search(line)
        if size_match and current_format is not None:
            current_mode = {
                'width': int(size_match.group(1)),
                'height': int(size_match.group(2)),
                'fps': [],
            }
            current_format['modes'].append(current_mode)
            continue

        interval_match = V4L2_INTERVAL_RE.search(line)
        if interval_match and current_mode is not None:
            current_mode['fps'].append(float(interval_match.group(2)))

    return formats


def read_yaml(path: Path, fallback: dict[str, Any]) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else fallback
    except (OSError, yaml.YAMLError):
        return fallback


def normalize_identity_config(config: dict[str, Any]) -> dict[str, Any]:
    robot = config.get('robot')
    if not isinstance(robot, dict):
        return dict(config)

    identity = dict(robot)
    robot_id = identity.pop('id', None)
    if robot_id is not None:
        identity['robot_id'] = robot_id
    return identity


def quaternion_yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass
class TopicSnapshot:
    received_monotonic: float = 0.0
    message_count: int = 0


@dataclass
class TopicWatch:
    topic: str
    type_name: str
    msg_class: type[Any]
    subscription: Any
    raw_message: Any = None
    last_message: Any = None
    last_updated_monotonic: float = 0.0
    message_count: int = 0
    last_error: str | None = None
    last_access_monotonic: float = 0.0


@dataclass
class ImageWatch:
    topic: str
    type_name: str
    subscription: Any
    frame_bytes: bytes | None = None
    content_type: str = 'image/jpeg'
    width: int = 0
    height: int = 0
    encoding: str = ''
    message_count: int = 0
    last_updated_monotonic: float = 0.0
    last_error: str | None = None
    last_access_monotonic: float = 0.0


@dataclass
class PublisherHandle:
    topic: str
    type_name: str
    msg_class: type[Any]
    publisher: Any


@dataclass
class ServiceHandle:
    service: str
    type_name: str
    srv_class: type[Any]
    client: Any


class RoverWebGateway(Node):
    def __init__(self) -> None:
        super().__init__('web_gateway_node')

        share = Path(get_package_share_directory('rover_web'))
        try:
            rover_share = Path(get_package_share_directory('rover_bringup'))
        except Exception:
            rover_share = share
        home = Path.home()
        try:
            workspace_root = share.parents[3]
        except Exception:
            workspace_root = home / 'sverk_rover'

        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter(
            'identity_file',
            str(share / 'config' / 'robot_identity.default.example.yaml'),
        )
        self.declare_parameter(
            'rover_config_file',
            str(rover_share / 'config' / 'rover_v1.yaml'),
        )
        self.declare_parameter('web_root', str(share / 'web'))
        self.declare_parameter(
            'motion_executor_path',
            str(share / 'tools' / 'rover_motion_executor.py'),
        )
        self.declare_parameter(
            'plans_directory',
            str(home / '.local' / 'share' / 'sverh-rover-web' / 'plans'),
        )
        self.declare_parameter(
            'hackathon_files_root',
            str(workspace_root / 'hackathon_files'),
        )
        self.declare_parameter('maps_root', default_maps_root(workspace_root))
        self.declare_parameter(
            'seed_plans_directory',
            str(share / 'plans'),
        )
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('camera_node_name', '/usb_camera_node')
        self.declare_parameter('vision_node_name', '/camera_detector_node')
        self.declare_parameter('lidar_node_name', '/sllidar_node')
        self.declare_parameter('led_strip_node_name', '/led_strip_node')
        self.declare_parameter('octoliner_node_name', '/octoliner_node')
        self.declare_parameter('voice_node_name', '/waveshare_audio_node')
        self.declare_parameter('voice_text_topic', '/voice/text')
        self.declare_parameter('voice_status_topic', '/waveshare_audio/status')
        self.declare_parameter('voice_package', 'rover_waveshare_audio')
        self.declare_parameter('voice_executable', 'waveshare_audio_node')
        self.declare_parameter('voice_config_file', '')
        self.declare_parameter('voice_enabled_on_start', False)
        self.declare_parameter('terminal_enabled', False)
        self.declare_parameter('terminal_url', '')
        self.declare_parameter('terminal_port', 7681)
        self.declare_parameter('terminal_path', '/terminal/')
        self.declare_parameter('rosboard_enabled', True)
        self.declare_parameter('rosboard_port', 8888)
        self.declare_parameter('servo_enabled', True)
        self.declare_parameter('drive_command_timeout_sec', 0.25)
        self.declare_parameter('default_linear_speed_mps', 0.18)
        self.declare_parameter('default_lateral_speed_mps', 0.16)
        self.declare_parameter('default_angular_speed_radps', 0.70)
        self.declare_parameter('max_linear_speed_mps', 0.35)
        self.declare_parameter('max_lateral_speed_mps', 0.35)
        self.declare_parameter('max_angular_speed_radps', 1.50)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_odom_topic', '/wheel/odometry')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('runtime_dir', '/tmp/rover_devices')
        self.declare_parameter('activity_limit', 1000)
        self.declare_parameter('stop_hold_sec', 0.75)

        self.bind_address = str(self.get_parameter('bind_address').value)
        self.port = int(self.get_parameter('port').value)
        self.identity_path = Path(
            str(self.get_parameter('identity_file').value)
        ).expanduser()
        self.rover_config_path = Path(
            str(self.get_parameter('rover_config_file').value)
        ).expanduser()
        self.web_root = Path(
            str(self.get_parameter('web_root').value)
        ).expanduser().resolve()
        self.executor_path = Path(
            str(self.get_parameter('motion_executor_path').value)
        ).expanduser()
        self.plans_directory = Path(
            str(self.get_parameter('plans_directory').value)
        ).expanduser()
        self.hackathon_files_root = Path(
            str(self.get_parameter('hackathon_files_root').value)
        ).expanduser().resolve()
        self.maps_root = Path(
            str(self.get_parameter('maps_root').value)
        ).expanduser().resolve()
        self.seed_plans_directory = Path(
            str(self.get_parameter('seed_plans_directory').value)
        ).expanduser()
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.camera_node_name = str(
            self.get_parameter('camera_node_name').value
        ).strip() or '/usb_camera_node'
        self.vision_node_name = str(
            self.get_parameter('vision_node_name').value
        ).strip() or '/camera_detector_node'
        self.lidar_node_name = str(
            self.get_parameter('lidar_node_name').value
        ).strip() or '/sllidar_node'
        self.led_strip_node_name = str(
            self.get_parameter('led_strip_node_name').value
        ).strip() or '/led_strip_node'
        self.octoliner_node_name = str(
            self.get_parameter('octoliner_node_name').value
        ).strip() or '/octoliner_node'
        self.voice_node_name = str(
            self.get_parameter('voice_node_name').value
        ).strip() or '/waveshare_audio_node'
        self.voice_text_topic = str(
            self.get_parameter('voice_text_topic').value
        ).strip() or '/voice/text'
        self.voice_status_topic = str(
            self.get_parameter('voice_status_topic').value
        ).strip() or '/waveshare_audio/status'
        self.voice_package = str(
            self.get_parameter('voice_package').value
        ).strip() or 'rover_waveshare_audio'
        self.voice_executable = str(
            self.get_parameter('voice_executable').value
        ).strip() or 'waveshare_audio_node'
        voice_config_file = str(self.get_parameter('voice_config_file').value).strip()
        if not voice_config_file:
            try:
                voice_config_file = str(
                    Path(get_package_share_directory(self.voice_package))
                    / 'config'
                    / 'default.example.yaml'
                )
            except Exception:
                voice_config_file = ''
        self.voice_config_file = Path(voice_config_file).expanduser() if voice_config_file else None
        self.voice_enabled_on_start = bool(
            self.get_parameter('voice_enabled_on_start').value
        )
        self.terminal_enabled = bool(self.get_parameter('terminal_enabled').value)
        self.terminal_url = str(self.get_parameter('terminal_url').value).strip()
        self.terminal_port = int(self.get_parameter('terminal_port').value)
        self.terminal_path = (
            str(self.get_parameter('terminal_path').value).strip() or '/terminal/'
        )
        self.rosboard_enabled = bool(self.get_parameter('rosboard_enabled').value)
        self.rosboard_port = int(self.get_parameter('rosboard_port').value)
        self.servo_enabled = bool(self.get_parameter('servo_enabled').value)
        self.drive_command_timeout_sec = max(
            0.1, float(self.get_parameter('drive_command_timeout_sec').value)
        )
        self.default_linear_speed = max(
            0.05, float(self.get_parameter('default_linear_speed_mps').value)
        )
        self.default_lateral_speed = max(
            0.05, float(self.get_parameter('default_lateral_speed_mps').value)
        )
        self.default_angular_speed = max(
            0.05, float(self.get_parameter('default_angular_speed_radps').value)
        )
        self.max_linear_speed = max(
            self.default_linear_speed,
            float(self.get_parameter('max_linear_speed_mps').value),
        )
        self.max_lateral_speed = max(
            self.default_lateral_speed,
            float(self.get_parameter('max_lateral_speed_mps').value),
        )
        self.max_angular_speed = max(
            self.default_angular_speed,
            float(self.get_parameter('max_angular_speed_radps').value),
        )
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.diagnostics_topic = str(self.get_parameter('diagnostics_topic').value)
        self.runtime_dir = Path(
            str(self.get_parameter('runtime_dir').value)
        ).expanduser()
        self.activity_limit = max(
            100, int(self.get_parameter('activity_limit').value)
        )
        self.stop_hold_sec = max(
            0.2, float(self.get_parameter('stop_hold_sec').value)
        )

        identity_fallback = {
            'robot_id': 'sverh-rover-0001',
            'hostname': socket.gethostname(),
            'company': 'Сверх',
            'model': 'mecanum-rover-v1',
            'software_version': '0.1.0',
        }
        self.identity = normalize_identity_config(
            read_yaml(self.identity_path, identity_fallback)
        )
        self.rover_config = read_yaml(self.rover_config_path, {})

        self.started_at = time.time()
        self._lock = threading.RLock()
        self._topic_watches: dict[tuple[str, str], TopicWatch] = {}
        self._image_watches: dict[tuple[str, str], ImageWatch] = {}
        self._publisher_cache: dict[tuple[str, str], PublisherHandle] = {}
        self._service_client_cache: dict[tuple[str, str], ServiceHandle] = {}
        self._parameter_client_cache: dict[str, AsyncParameterClient] = {}
        self._latest_drive_command = Twist()
        self._latest_drive_monotonic = 0.0
        self._drive_active = False
        self._stop_until = 0.0

        self._odom: Odometry | None = None
        self._wheel_odom: Odometry | None = None
        self._imu: Imu | None = None
        self._diagnostics: list[dict[str, Any]] = []
        self._topic_state = {
            'odom': TopicSnapshot(),
            'wheel_odometry': TopicSnapshot(),
            'imu': TopicSnapshot(),
            'diagnostics': TopicSnapshot(),
        }
        self._web_clients: dict[str, dict[str, Any]] = {}
        self._activity: deque[dict[str, Any]] = deque(maxlen=self.activity_limit)
        self._motion_process: subprocess.Popen[str] | None = None
        self._motion_started_at: float | None = None
        self._motion_command: list[str] = []
        self._motion_log: deque[str] = deque(maxlen=500)
        self._motion_return_code: int | None = None
        self._voice_process: subprocess.Popen[str] | None = None
        self._voice_started_at: float | None = None
        self._voice_return_code: int | None = None
        self._voice_command: list[str] = []
        self._voice_log: deque[str] = deque(maxlen=300)
        self._voice_latest_text = ''
        self._voice_latest_status = ''
        self._voice_latest_text_at: float | None = None
        self._voice_latest_status_at: float | None = None
        self._voice_text_history: deque[dict[str, Any]] = deque(maxlen=60)

        self._cpu_last_total = 0
        self._cpu_last_idle = 0
        self._system_cache_monotonic = 0.0
        self._system_cache: dict[str, Any] = {}
        self._graph_cache_monotonic = 0.0
        self._graph_cache: dict[str, Any] = {}
        self._last_ip_check = 0.0
        self._cached_ip_addresses: list[str] = []

        self.drive_publisher = self.create_publisher(Twist, self.command_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 20)
        self.create_subscription(
            Odometry,
            self.wheel_odom_topic,
            self._wheel_odom_callback,
            20,
        )
        self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray,
            self.diagnostics_topic,
            self._diagnostics_callback,
            10,
        )
        self.create_subscription(String, self.voice_text_topic, self._voice_text_callback, 10)
        self.create_subscription(
            String,
            self.voice_status_topic,
            self._voice_status_callback,
            10,
        )
        self.create_timer(0.05, self._drive_output_timer)
        self.create_timer(1.0, self._maintenance_timer)

        self.plans_directory.mkdir(parents=True, exist_ok=True)
        self.hackathon_files_root.mkdir(parents=True, exist_ok=True)
        self._seed_default_plans()
        self.record_activity('system', 'Web gateway started', {'port': self.port})
        if self.voice_enabled_on_start:
            try:
                self.start_voice_processing()
            except Exception as exc:
                self.get_logger().warn(f'Voice processing autostart failed: {exc}')

        handler = self._build_handler()
        self._http_server = ThreadingHTTPServer((self.bind_address, self.port), handler)
        self._http_server.daemon_threads = True
        self._http_server.gateway = self  # type: ignore[attr-defined]
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name='rover-web-http',
            daemon=True,
        )
        self._http_thread.start()

        addresses = current_ipv4_addresses()
        address_list = ', '.join(addresses) if addresses else 'no IPv4 detected'
        self.get_logger().info(
            f'Rover web listening on http://{self.bind_address}:{self.port} '
            f'(LAN addresses: {address_list})'
        )

    def _touch_snapshot(self, key: str) -> None:
        snapshot = self._topic_state[key]
        snapshot.received_monotonic = time.monotonic()
        snapshot.message_count += 1

    def _odom_callback(self, message: Odometry) -> None:
        with self._lock:
            self._odom = message
            self._touch_snapshot('odom')

    def _wheel_odom_callback(self, message: Odometry) -> None:
        with self._lock:
            self._wheel_odom = message
            self._touch_snapshot('wheel_odometry')

    def _imu_callback(self, message: Imu) -> None:
        with self._lock:
            self._imu = message
            self._touch_snapshot('imu')

    def _diagnostics_callback(self, message: DiagnosticArray) -> None:
        converted: list[dict[str, Any]] = []
        for status in message.status[:100]:
            level = status.level
            if isinstance(level, (bytes, bytearray, memoryview)):
                level_value = int(level[0]) if len(level) > 0 else 0
            else:
                level_value = int(level)
            converted.append({
                'name': status.name,
                'hardware_id': status.hardware_id,
                'level': level_value,
                'message': status.message,
                'values': {
                    item.key: item.value
                    for item in status.values[:40]
                },
            })
        with self._lock:
            self._diagnostics = converted
            self._touch_snapshot('diagnostics')

    def _voice_text_callback(self, message: String) -> None:
        text = str(message.data or '').strip()
        now = time.time()
        with self._lock:
            self._voice_latest_text = text
            self._voice_latest_text_at = now
            if text:
                self._voice_text_history.append({
                    'timestamp': now,
                    'text': text,
                })

    def _voice_status_callback(self, message: String) -> None:
        with self._lock:
            self._voice_latest_status = str(message.data or '').strip()
            self._voice_latest_status_at = time.time()

    def _topic_graph(self) -> tuple[dict[str, list[str]], dict[str, list[str]]]:
        topic_types = {
            name: list(types)
            for name, types in self.get_topic_names_and_types()
        }
        service_types = {
            name: list(types)
            for name, types in self.get_service_names_and_types()
        }
        return topic_types, service_types

    def _resolve_topic_type(
        self,
        topic: str,
        requested_type: str | None = None,
    ) -> tuple[str, list[str]]:
        topic_types, _ = self._topic_graph()
        available_types = topic_types.get(topic, [])
        if requested_type:
            return requested_type, available_types
        if not available_types:
            raise ValueError(f'No ROS type is currently visible for topic {topic}')
        return available_types[0], available_types

    def _resolve_service_type(
        self,
        service: str,
        requested_type: str | None = None,
    ) -> tuple[str, list[str]]:
        _, service_types = self._topic_graph()
        available_types = service_types.get(service, [])
        if requested_type:
            return requested_type, available_types
        if not available_types:
            raise ValueError(f'No ROS type is currently visible for service {service}')
        return available_types[0], available_types

    def _message_summary(self, message: Any) -> Any:
        return sanitize_payload(message_to_ordereddict(message))

    def _ensure_topic_watch(self, topic: str, type_name: str) -> TopicWatch:
        key = (topic, type_name)
        with self._lock:
            existing = self._topic_watches.get(key)
            if existing is not None:
                existing.last_access_monotonic = time.monotonic()
                return existing

            msg_class = get_message(type_name)

            def callback(message: Any) -> None:
                with self._lock:
                    watch = self._topic_watches.get(key)
                    if watch is None:
                        return
                    watch.last_updated_monotonic = time.monotonic()
                    watch.message_count += 1
                    watch.last_error = None
                    watch.raw_message = message
                    try:
                        watch.last_message = self._message_summary(message)
                    except Exception as exc:
                        watch.last_message = {
                            'summary_error': f'{type(exc).__name__}: {exc}'
                        }
                        watch.last_error = str(exc)

            subscription = self.create_subscription(
                msg_class,
                topic,
                callback,
                10,
            )
            watch = TopicWatch(
                topic=topic,
                type_name=type_name,
                msg_class=msg_class,
                subscription=subscription,
                last_access_monotonic=time.monotonic(),
            )
            self._topic_watches[key] = watch
            return watch

    def _reshape_raw_image(
        self,
        message: Image,
        *,
        channels: int,
    ) -> np.ndarray:
        expected_row_size = int(message.width * channels)
        if message.step < expected_row_size:
            raise ValueError('Image step is smaller than expected row size')

        data = np.frombuffer(message.data, dtype=np.uint8)
        expected_bytes = int(message.step * message.height)
        if data.size < expected_bytes:
            raise ValueError('Image payload is shorter than expected')

        rows = data[:expected_bytes].reshape((message.height, message.step))
        cropped = rows[:, :expected_row_size]
        if channels == 1:
            return cropped.reshape((message.height, message.width))
        return cropped.reshape((message.height, message.width, channels))

    def _encode_image_message(self, message: Image) -> tuple[bytes, str, int, int, str]:
        encoding = message.encoding.lower()
        if encoding == 'bgr8':
            frame = self._reshape_raw_image(message, channels=3)
        elif encoding == 'rgb8':
            frame = cv2.cvtColor(
                self._reshape_raw_image(message, channels=3),
                cv2.COLOR_RGB2BGR,
            )
        elif encoding == 'mono8':
            frame = self._reshape_raw_image(message, channels=1)
        elif encoding == 'bgra8':
            frame = cv2.cvtColor(
                self._reshape_raw_image(message, channels=4),
                cv2.COLOR_BGRA2BGR,
            )
        elif encoding == 'rgba8':
            frame = cv2.cvtColor(
                self._reshape_raw_image(message, channels=4),
                cv2.COLOR_RGBA2BGR,
            )
        else:
            raise ValueError(f'Unsupported image encoding: {message.encoding}')

        ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if not ok:
            raise ValueError('OpenCV could not encode frame as JPEG')
        return (
            encoded.tobytes(),
            'image/jpeg',
            int(message.width),
            int(message.height),
            message.encoding,
        )

    def _ensure_image_watch(self, topic: str, type_name: str) -> ImageWatch:
        key = (topic, type_name)
        with self._lock:
            existing = self._image_watches.get(key)
            if existing is not None:
                existing.last_access_monotonic = time.monotonic()
                return existing

            if type_name == 'sensor_msgs/msg/Image':

                def callback(message: Image) -> None:
                    with self._lock:
                        watch = self._image_watches.get(key)
                        if watch is None:
                            return
                        watch.last_access_monotonic = time.monotonic()
                        watch.last_updated_monotonic = time.monotonic()
                        watch.message_count += 1
                        try:
                            (
                                watch.frame_bytes,
                                watch.content_type,
                                watch.width,
                                watch.height,
                                watch.encoding,
                            ) = self._encode_image_message(message)
                            watch.last_error = None
                        except Exception as exc:
                            watch.last_error = f'{type(exc).__name__}: {exc}'

                subscription = self.create_subscription(
                    Image,
                    topic,
                    callback,
                    qos_profile_sensor_data,
                )

            elif type_name == 'sensor_msgs/msg/CompressedImage':

                def callback(message: CompressedImage) -> None:
                    with self._lock:
                        watch = self._image_watches.get(key)
                        if watch is None:
                            return
                        watch.last_access_monotonic = time.monotonic()
                        watch.last_updated_monotonic = time.monotonic()
                        watch.message_count += 1
                        watch.frame_bytes = bytes(message.data)
                        watch.content_type = compressed_content_type(message.format)
                        watch.width = 0
                        watch.height = 0
                        watch.encoding = message.format or 'compressed'
                        watch.last_error = None

                subscription = self.create_subscription(
                    CompressedImage,
                    topic,
                    callback,
                    qos_profile_sensor_data,
                )

            else:
                raise ValueError(f'Unsupported image topic type: {type_name}')

            watch = ImageWatch(
                topic=topic,
                type_name=type_name,
                subscription=subscription,
                last_access_monotonic=time.monotonic(),
            )
            self._image_watches[key] = watch
            return watch

    def _ensure_publisher(self, topic: str, type_name: str) -> PublisherHandle:
        key = (topic, type_name)
        with self._lock:
            existing = self._publisher_cache.get(key)
            if existing is not None:
                return existing
            msg_class = get_message(type_name)
            publisher = self.create_publisher(msg_class, topic, 10)
            handle = PublisherHandle(
                topic=topic,
                type_name=type_name,
                msg_class=msg_class,
                publisher=publisher,
            )
            self._publisher_cache[key] = handle
            return handle

    def _ensure_service_client(self, service: str, type_name: str) -> ServiceHandle:
        key = (service, type_name)
        with self._lock:
            existing = self._service_client_cache.get(key)
            if existing is not None:
                return existing
            srv_class = get_service(type_name)
            client = self.create_client(srv_class, service)
            handle = ServiceHandle(
                service=service,
                type_name=type_name,
                srv_class=srv_class,
                client=client,
            )
            self._service_client_cache[key] = handle
            return handle

    def _ensure_parameter_client(self, node_name: str) -> AsyncParameterClient:
        with self._lock:
            existing = self._parameter_client_cache.get(node_name)
            if existing is not None:
                return existing
            client = AsyncParameterClient(self, node_name)
            self._parameter_client_cache[node_name] = client
            return client

    def _wait_for_future(
        self,
        future: Any,
        *,
        timeout_sec: float,
        label: str,
    ) -> Any:
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError(f'Timeout while waiting for {label}')
        return future.result()

    def _maintenance_timer(self) -> None:
        cutoff = time.monotonic() - 10.0
        with self._lock:
            expired = [
                key
                for key, value in self._web_clients.items()
                if value.get('last_seen', 0.0) < cutoff
            ]
            for key in expired:
                del self._web_clients[key]

            process = self._motion_process
            if process is not None:
                return_code = process.poll()
                if return_code is None:
                    pass
                else:
                    self._motion_return_code = return_code
                    self._motion_process = None
                    self.record_activity(
                        'routes',
                        'Motion process finished',
                        {'return_code': return_code},
                    )

            voice_process = self._voice_process
            if voice_process is not None:
                return_code = voice_process.poll()
                if return_code is not None:
                    self._voice_return_code = return_code
                    self._voice_process = None
                    self.record_activity(
                        'voice',
                        'Voice processing node finished',
                        {'return_code': return_code},
                    )

    def record_activity(
        self,
        source: str,
        message: str,
        details: dict[str, Any] | None = None,
    ) -> None:
        item = {
            'timestamp': time.time(),
            'source': source[:64],
            'message': message[:300],
            'details': sanitize_payload(details or {}),
        }
        with self._lock:
            self._activity.append(item)

    def register_heartbeat(self, session_id: str, page: str, client_ip: str) -> None:
        if not session_id or len(session_id) > 128:
            return
        with self._lock:
            self._web_clients[session_id] = {
                'session_id': session_id,
                'page': page[:64],
                'client_ip': client_ip,
                'last_seen': time.monotonic(),
            }

    def request_stop(
        self,
        source: str,
        details: dict[str, Any] | None = None,
    ) -> None:
        with self._lock:
            self._stop_until = max(
                self._stop_until,
                time.monotonic() + self.stop_hold_sec,
            )
            self._latest_drive_command = Twist()
            self._latest_drive_monotonic = time.monotonic()
        self.record_activity(source, 'Software STOP requested', details or {})

    def identity_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            if now - self._last_ip_check > 10.0 or not self._cached_ip_addresses:
                self._cached_ip_addresses = current_ipv4_addresses()
                self._last_ip_check = now
            addresses = list(self._cached_ip_addresses)
        payload = dict(self.identity)
        payload['runtime_hostname'] = socket.gethostname()
        payload['ip_addresses'] = addresses
        return payload

    def _seed_default_plans(self) -> None:
        if any(self.plans_directory.glob('*.yaml')):
            return
        if not self.seed_plans_directory.is_dir():
            return
        for source in sorted(self.seed_plans_directory.glob('*.yaml')):
            target = self.plans_directory / source.name
            if target.exists():
                continue
            try:
                shutil.copy2(source, target)
            except OSError:
                continue

    def _pose_payload(self, message: Odometry | None) -> dict[str, Any] | None:
        if message is None:
            return None
        return {
            'x': float(message.pose.pose.position.x),
            'y': float(message.pose.pose.position.y),
            'yaw': quaternion_yaw(message),
            'vx': float(message.twist.twist.linear.x),
            'vy': float(message.twist.twist.linear.y),
            'wz': float(message.twist.twist.angular.z),
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
        }

    def _cpu_percent(self) -> float | None:
        try:
            fields = [
                int(value)
                for value in Path('/proc/stat').read_text().splitlines()[0].split()[1:]
            ]
            idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
            total = sum(fields)
            if self._cpu_last_total == 0:
                value = None
            else:
                total_delta = total - self._cpu_last_total
                idle_delta = idle - self._cpu_last_idle
                value = (
                    None
                    if total_delta <= 0
                    else 100.0 * (1.0 - idle_delta / total_delta)
                )
            self._cpu_last_total = total
            self._cpu_last_idle = idle
            return value
        except (OSError, ValueError, IndexError):
            return None

    def public_config_payload(self) -> dict[str, Any]:
        geometry = self.rover_config.get('geometry', {})
        encoders = self.rover_config.get('encoders', {})
        base = self.rover_config.get('base_driver', {})
        return {
            'command_topic': self.command_topic,
            'drive_command_timeout_sec': self.drive_command_timeout_sec,
            'drive_defaults': {
                'linear_x': self.default_linear_speed,
                'linear_y': self.default_lateral_speed,
                'angular_z': self.default_angular_speed,
            },
            'drive_limits': {
                'linear_x': self.max_linear_speed,
                'linear_y': self.max_lateral_speed,
                'angular_z': self.max_angular_speed,
            },
            'odom_topic': self.odom_topic,
            'wheel_odom_topic': self.wheel_odom_topic,
            'imu_topic': self.imu_topic,
            'diagnostics_topic': self.diagnostics_topic,
            'camera_node_name': self.camera_node_name,
            'vision_node_name': self.vision_node_name,
            'lidar_node_name': self.lidar_node_name,
            'led_strip_node_name': self.led_strip_node_name,
            'octoliner_node_name': self.octoliner_node_name,
            'voice_node_name': self.voice_node_name,
            'voice_text_topic': self.voice_text_topic,
            'voice_status_topic': self.voice_status_topic,
            'plans_directory': str(self.plans_directory),
            'hackathon_files_root': str(self.hackathon_files_root),
            'maps_root': str(self.maps_root),
            'web': {
                'terminal_enabled': self.terminal_enabled,
                'terminal_url': self.terminal_url,
                'terminal_port': self.terminal_port,
                'terminal_path': self.terminal_path,
                'rosboard_enabled': self.rosboard_enabled,
                'rosboard_port': self.rosboard_port,
                'servo_enabled': self.servo_enabled,
            },
            'geometry': geometry,
            'encoders': encoders,
            'limits': {
                'max_wheel_speed_mps': base.get('max_wheel_speed_mps', 0.35),
                'command_timeout_sec': base.get('command_timeout_sec', 0.5),
                'feedback_timeout_sec': base.get('feedback_timeout_sec', 0.35),
            },
            'recommended': {
                'manual_linear_speed_mps': min(self.max_linear_speed, 0.18),
                'manual_lateral_speed_mps': min(self.max_lateral_speed, 0.16),
                'manual_angular_speed_radps': min(self.max_angular_speed, 0.70),
            },
        }

    def _node_is_visible(self, full_name: str) -> bool:
        target = full_name if full_name.startswith('/') else f'/{full_name}'
        try:
            for name, namespace in self.get_node_names_and_namespaces():
                namespace = namespace.strip()
                if namespace in {'', '/'}:
                    candidate = f'/{name}'
                else:
                    candidate = f'{namespace.rstrip("/")}/{name}'
                if candidate == target:
                    return True
        except Exception:
            return False
        return False

    def _voice_command_line(self) -> list[str]:
        command = [
            shutil.which('ros2') or 'ros2',
            'run',
            self.voice_package,
            self.voice_executable,
        ]
        if self.voice_config_file is not None and self.voice_config_file.is_file():
            command += [
                '--ros-args',
                '--params-file',
                str(self.voice_config_file),
            ]
        return command

    def start_voice_processing(self) -> dict[str, Any]:
        with self._lock:
            process = self._voice_process
            if process is not None and process.poll() is None:
                return self.voice_status_payload_locked()

        if self._node_is_visible(self.voice_node_name):
            self.record_activity(
                'voice',
                'Voice processing node is already running externally',
                {'node': self.voice_node_name},
            )
            return self.voice_status_payload()

        environment = os.environ.copy()
        environment.pop('PYTHONNOUSERSITE', None)
        environment.setdefault('PYTHONUNBUFFERED', '1')
        command = self._voice_command_line()
        try:
            process = subprocess.Popen(
                command,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except OSError as exc:
            raise RuntimeError(f'Unable to start voice processing: {exc}') from exc

        with self._lock:
            self._voice_process = process
            self._voice_started_at = time.time()
            self._voice_return_code = None
            self._voice_command = command
            self._voice_log.clear()

        threading.Thread(
            target=self._read_voice_output,
            args=(process,),
            name='rover-voice-output',
            daemon=True,
        ).start()
        self.record_activity(
            'voice',
            'Voice processing node started',
            {'command': command},
        )
        return self.voice_status_payload()

    def _read_voice_output(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            cleaned = line.rstrip('\r\n')
            with self._lock:
                self._voice_log.append(cleaned)

    def stop_voice_processing(self) -> dict[str, Any]:
        with self._lock:
            process = self._voice_process
        if process is None:
            if self._node_is_visible(self.voice_node_name):
                raise RuntimeError(
                    f'{self.voice_node_name} is running, but it was not started '
                    'by web gateway'
                )
            return self.voice_status_payload()

        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            threading.Thread(
                target=self._ensure_voice_stopped,
                args=(process,),
                name='rover-voice-stop',
                daemon=True,
            ).start()
        else:
            with self._lock:
                self._voice_return_code = process.returncode
                self._voice_process = None

        self.record_activity('voice', 'Voice processing node stop requested', {})
        return self.voice_status_payload()

    def _ensure_voice_stopped(self, process: subprocess.Popen[str]) -> None:
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def voice_status_payload_locked(self) -> dict[str, Any]:
        process = self._voice_process
        managed_running = process is not None and process.poll() is None
        node_visible = self._node_is_visible(self.voice_node_name)
        return {
            'ok': True,
            'enabled': managed_running or node_visible,
            'managed_running': managed_running,
            'external_running': node_visible and not managed_running,
            'node_visible': node_visible,
            'pid': process.pid if managed_running and process is not None else None,
            'started_at': self._voice_started_at,
            'return_code': self._voice_return_code,
            'command': list(self._voice_command),
            'node_name': self.voice_node_name,
            'text_topic': self.voice_text_topic,
            'status_topic': self.voice_status_topic,
            'config_file': str(self.voice_config_file or ''),
            'latest_text': self._voice_latest_text,
            'latest_text_at': self._voice_latest_text_at,
            'latest_status': self._voice_latest_status,
            'latest_status_at': self._voice_latest_status_at,
            'history': list(self._voice_text_history)[-30:],
            'log': list(self._voice_log)[-80:],
        }

    def voice_status_payload(self) -> dict[str, Any]:
        with self._lock:
            return self.voice_status_payload_locked()

    def set_voice_enabled(self, enabled: bool) -> dict[str, Any]:
        if enabled:
            return self.start_voice_processing()
        return self.stop_voice_processing()

    def _system_summary(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            if now - self._system_cache_monotonic < 0.5 and self._system_cache:
                return dict(self._system_cache)

        temperature_c = None
        try:
            raw = Path('/sys/class/thermal/thermal_zone0/temp').read_text().strip()
            temperature_c = int(raw) / 1000.0
        except (OSError, ValueError):
            pass

        memory_total = None
        memory_available = None
        try:
            values: dict[str, int] = {}
            for line in Path('/proc/meminfo').read_text().splitlines():
                key, value = line.split(':', 1)
                values[key] = int(value.strip().split()[0]) * 1024
            memory_total = values.get('MemTotal')
            memory_available = values.get('MemAvailable')
        except (OSError, ValueError, IndexError):
            pass

        disk_total = disk_free = None
        try:
            usage = shutil.disk_usage('/')
            disk_total = usage.total
            disk_free = usage.free
        except OSError:
            pass

        load_1 = load_5 = load_15 = None
        try:
            load_1, load_5, load_15 = os.getloadavg()
        except OSError:
            pass

        uptime_sec = None
        try:
            uptime_sec = float(Path('/proc/uptime').read_text().split()[0])
        except (OSError, ValueError, IndexError):
            uptime_sec = max(0.0, time.time() - self.started_at)

        throttled = 'unavailable'
        try:
            result = subprocess.run(
                ['vcgencmd', 'get_throttled'],
                check=False,
                capture_output=True,
                text=True,
                timeout=0.4,
            )
            throttled = result.stdout.strip() or 'unavailable'
        except (OSError, subprocess.SubprocessError):
            pass

        payload = {
            'ok': True,
            'hostname': socket.gethostname(),
            'ip_addresses': self.identity_payload().get('ip_addresses', []),
            'bind_address': self.bind_address,
            'port': self.port,
            'started_at': self.started_at,
            'uptime_sec': uptime_sec,
            'cpu_percent': self._cpu_percent(),
            'temperature_c': temperature_c,
            'memory_total_bytes': memory_total,
            'memory_available_bytes': memory_available,
            'disk_total_bytes': disk_total,
            'disk_free_bytes': disk_free,
            'throttled': throttled,
            'load_average': {
                'one_min': load_1,
                'five_min': load_5,
                'fifteen_min': load_15,
            },
            'command_topic': self.command_topic,
            'drive_command_timeout_sec': self.drive_command_timeout_sec,
            'drive_defaults': {
                'linear_x': self.default_linear_speed,
                'linear_y': self.default_lateral_speed,
                'angular_z': self.default_angular_speed,
            },
            'drive_limits': {
                'linear_x': self.max_linear_speed,
                'linear_y': self.max_lateral_speed,
                'angular_z': self.max_angular_speed,
            },
            'devices': self._devices_payload(),
            'ros': self._graph_counts(),
        }
        with self._lock:
            self._system_cache_monotonic = now
            self._system_cache = dict(payload)
        return payload

    def _graph_counts(self) -> dict[str, int]:
        graph = self._graph_payload()
        return {
            'nodes': len(graph['nodes']),
            'topics': len(graph['topics']),
            'services': len(graph['services']),
            'image_topics': len(graph['image_topics']),
        }

    def _devices_payload(self) -> dict[str, Any]:
        path = self.runtime_dir / 'devices.json'
        try:
            value = json.loads(path.read_text(encoding='utf-8'))
            return {
                'available': isinstance(value, dict),
                'path': str(path),
                'devices': value if isinstance(value, dict) else {},
            }
        except (OSError, json.JSONDecodeError):
            return {'available': False, 'path': str(path), 'devices': {}}

    def status_payload(self) -> dict[str, Any]:
        with self._lock:
            diagnostics = list(self._diagnostics)
            clients = [dict(value) for value in self._web_clients.values()]
            for client in clients:
                client.pop('last_seen', None)
            topic_state = {
                key: {
                    'age_sec': age_seconds(value.received_monotonic),
                    'message_count': value.message_count,
                }
                for key, value in self._topic_state.items()
            }
            odom = self._pose_payload(self._odom)
            wheel_odom = self._pose_payload(self._wheel_odom)
            imu = None
            if self._imu is not None:
                imu = {
                    'angular_velocity_z': float(self._imu.angular_velocity.z),
                    'linear_acceleration_x': float(self._imu.linear_acceleration.x),
                    'linear_acceleration_y': float(self._imu.linear_acceleration.y),
                    'linear_acceleration_z': float(self._imu.linear_acceleration.z),
                    'frame_id': self._imu.header.frame_id,
                }
            motion = self.motion_status_payload_locked()

        highest_level = max((entry['level'] for entry in diagnostics), default=-1)
        return {
            'ok': True,
            'server_time': time.time(),
            'identity': self.identity_payload(),
            'system': self._system_summary(),
            'topics': topic_state,
            'device_discovery': self._devices_payload(),
            'odom': odom,
            'wheel_odometry': wheel_odom,
            'imu': imu,
            'diagnostics': {
                'highest_level': highest_level,
                'items': diagnostics,
            },
            'clients': clients,
            'connected_clients': len(clients),
            'drive_clients': sum(1 for item in clients if item.get('page') == 'drive'),
            'motion': motion,
        }

    def activity_payload(self, limit: int) -> list[dict[str, Any]]:
        limit = min(max(limit, 1), 1000)
        with self._lock:
            return list(self._activity)[-limit:][::-1]

    def list_plans(self) -> list[dict[str, Any]]:
        plans: list[dict[str, Any]] = []
        for path in sorted(self.plans_directory.glob('*.yaml')):
            try:
                stat = path.stat()
                plan = read_yaml(path, {})
                steps = plan.get('steps', [])
                plans.append({
                    'name': path.name,
                    'size_bytes': stat.st_size,
                    'modified': stat.st_mtime,
                    'steps': len(steps) if isinstance(steps, list) else 0,
                })
            except OSError:
                continue
        return plans

    def _safe_plan_path(self, name: str) -> Path:
        name = unquote(name)
        if not PLAN_NAME_RE.fullmatch(name) or not name.endswith(('.yaml', '.yml')):
            raise ValueError('Invalid plan name')
        path = (self.plans_directory / name).resolve()
        if path.parent != self.plans_directory.resolve():
            raise ValueError('Invalid plan path')
        return path

    def read_plan(self, name: str) -> dict[str, Any]:
        path = self._safe_plan_path(name)
        if not path.exists():
            raise FileNotFoundError(name)
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(value, dict):
            raise ValueError('Plan must contain a YAML object')
        return value

    def save_plan(self, name: str, plan: dict[str, Any]) -> None:
        steps = plan.get('steps')
        if not isinstance(steps, list) or not steps:
            raise ValueError('Plan requires a non-empty steps list')
        path = self._safe_plan_path(name)
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(
            yaml.safe_dump(plan, allow_unicode=True, sort_keys=False),
            encoding='utf-8',
        )
        temporary.replace(path)
        self.record_activity('routes', 'Plan saved', {'name': name, 'steps': len(steps)})

    def _graph_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            if now - self._graph_cache_monotonic < 0.5 and self._graph_cache:
                return dict(self._graph_cache)

        topic_types, service_types = self._topic_graph()
        nodes = [
            {
                'name': name,
                'namespace': namespace,
                'full_name': f'{namespace.rstrip("/")}/{name}'.replace('//', '/'),
            }
            for name, namespace in sorted(self.get_node_names_and_namespaces())
        ]
        topics = []
        image_topics = []
        for name, types in sorted(topic_types.items()):
            entry = {
                'name': name,
                'types': types,
                'publishers': self.count_publishers(name),
                'subscribers': self.count_subscribers(name),
                'is_image': any(item in IMAGE_TOPIC_TYPES for item in types),
            }
            topics.append(entry)
            if entry['is_image']:
                image_topics.append(entry)

        services = [
            {
                'name': name,
                'types': types,
            }
            for name, types in sorted(service_types.items())
        ]

        payload = {
            'ok': True,
            'nodes': nodes,
            'topics': topics,
            'services': services,
            'image_topics': image_topics,
        }
        with self._lock:
            self._graph_cache_monotonic = now
            self._graph_cache = dict(payload)
        return payload

    def inspect_topic(self, topic_name: str, type_name: str | None = None) -> dict[str, Any]:
        topic = normalize_topic_name(topic_name)
        resolved_type, available_types = self._resolve_topic_type(topic, type_name)
        if resolved_type in IMAGE_TOPIC_TYPES:
            watch = self._ensure_image_watch(topic, resolved_type)
            return {
                'ok': True,
                'kind': 'image',
                'topic': topic,
                'type': resolved_type,
                'available_types': available_types,
                'publishers': self.count_publishers(topic),
                'subscribers': self.count_subscribers(topic),
                'message_count': watch.message_count,
                'age_sec': age_seconds(watch.last_updated_monotonic),
                'frame_url': (
                    f'/api/camera/frame?topic={topic}&type={resolved_type}'
                ),
                'width': watch.width,
                'height': watch.height,
                'encoding': watch.encoding,
                'last_error': watch.last_error,
            }

        watch = self._ensure_topic_watch(topic, resolved_type)
        return {
            'ok': True,
            'kind': 'message',
            'topic': topic,
            'type': resolved_type,
            'available_types': available_types,
            'publishers': self.count_publishers(topic),
            'subscribers': self.count_subscribers(topic),
            'message_count': watch.message_count,
            'age_sec': age_seconds(watch.last_updated_monotonic),
            'last_error': watch.last_error,
            'template': make_message_template(watch.msg_class),
            'latest_message': watch.last_message,
        }

    def publish_topic(
        self,
        topic_name: str,
        payload: dict[str, Any],
        type_name: str | None = None,
    ) -> dict[str, Any]:
        topic = normalize_topic_name(topic_name)
        resolved_type, _ = self._resolve_topic_type(topic, type_name)
        if resolved_type in IMAGE_TOPIC_TYPES:
            raise ValueError('Publishing image topics from the web UI is not supported yet')

        handle = self._ensure_publisher(topic, resolved_type)
        message = handle.msg_class()
        if not isinstance(payload, dict):
            raise ValueError('Topic payload must be a JSON object')
        set_message_fields(message, payload)
        handle.publisher.publish(message)
        self.record_activity(
            'ros',
            'Topic published',
            {'topic': topic, 'type': resolved_type},
        )
        return {
            'ok': True,
            'topic': topic,
            'type': resolved_type,
        }

    def inspect_service(
        self,
        service_name: str,
        type_name: str | None = None,
    ) -> dict[str, Any]:
        service = normalize_service_name(service_name)
        resolved_type, available_types = self._resolve_service_type(service, type_name)
        handle = self._ensure_service_client(service, resolved_type)
        return {
            'ok': True,
            'service': service,
            'type': resolved_type,
            'available_types': available_types,
            'ready': bool(handle.client.service_is_ready()),
            'request_template': make_message_template(handle.srv_class.Request),
        }

    def call_service(
        self,
        service_name: str,
        request_payload: dict[str, Any],
        type_name: str | None = None,
    ) -> dict[str, Any]:
        service = normalize_service_name(service_name)
        resolved_type, _ = self._resolve_service_type(service, type_name)
        handle = self._ensure_service_client(service, resolved_type)

        if not isinstance(request_payload, dict):
            raise ValueError('Service request must be a JSON object')

        if not handle.client.wait_for_service(timeout_sec=1.5):
            raise RuntimeError(f'Service {service} is not available')

        request = handle.srv_class.Request()
        set_message_fields(request, request_payload)

        started = time.monotonic()
        future = handle.client.call_async(request)
        deadline = started + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError(f'Service call timed out for {service}')

        response = future.result()
        if response is None:
            raise RuntimeError(f'Service {service} returned no response')

        duration = time.monotonic() - started
        self.record_activity(
            'ros',
            'Service called',
            {
                'service': service,
                'type': resolved_type,
                'duration_sec': duration,
            },
        )
        return {
            'ok': True,
            'service': service,
            'type': resolved_type,
            'duration_sec': duration,
            'response': sanitize_payload(message_to_ordereddict(response)),
        }

    def camera_topics(self) -> dict[str, Any]:
        graph = self._graph_payload()
        return {'ok': True, 'topics': graph['image_topics']}

    def lidar_topics(self) -> dict[str, Any]:
        graph = self._graph_payload()
        topics = [
            item for item in graph['topics']
            if LASER_SCAN_TYPE in item.get('types', [])
        ]
        return {'ok': True, 'topics': topics}

    def led_strip_topics(self) -> dict[str, Any]:
        graph = self._graph_payload()
        topics = [
            item for item in graph['topics']
            if LED_STRIP_STATE_TYPE in item.get('types', [])
        ]
        return {'ok': True, 'topics': topics}

    def octoliner_topics(self) -> dict[str, Any]:
        graph = self._graph_payload()
        topics = [
            item for item in graph['topics']
            if OCTOLINER_READING_TYPE in item.get('types', [])
        ]
        return {'ok': True, 'topics': topics}

    def _scan_points_payload(
        self,
        message: LaserScan,
        *,
        max_points: int = 720,
    ) -> dict[str, Any]:
        ranges = list(message.ranges)
        total_ranges = len(ranges)
        if total_ranges <= 0:
            return {
                'points': [],
                'total_ranges': 0,
                'valid_points': 0,
                'range_min': float(message.range_min),
                'range_max': float(message.range_max),
                'angle_min': float(message.angle_min),
                'angle_max': float(message.angle_max),
                'angle_increment': float(message.angle_increment),
                'frame_id': message.header.frame_id,
            }

        step = max(1, math.ceil(total_ranges / max_points))
        points: list[list[float]] = []
        valid_points = 0
        for index in range(0, total_ranges, step):
            distance = float(ranges[index])
            if not math.isfinite(distance):
                continue
            if distance < float(message.range_min) or distance > float(message.range_max):
                continue
            angle = float(message.angle_min + index * message.angle_increment)
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            points.append([x, y])
            valid_points += 1

        return {
            'points': points,
            'total_ranges': total_ranges,
            'valid_points': valid_points,
            'range_min': float(message.range_min),
            'range_max': float(message.range_max),
            'angle_min': float(message.angle_min),
            'angle_max': float(
                message.angle_min + max(0, total_ranges - 1) * message.angle_increment
            ),
            'angle_increment': float(message.angle_increment),
            'frame_id': message.header.frame_id,
        }

    def lidar_status(
        self,
        topic_name: str,
        type_name: str | None = None,
    ) -> dict[str, Any]:
        topic = normalize_topic_name(topic_name)
        resolved_type, available_types = self._resolve_topic_type(topic, type_name)
        if resolved_type != LASER_SCAN_TYPE:
            raise ValueError(f'Topic {topic} is not a LaserScan topic')

        watch = self._ensure_topic_watch(topic, resolved_type)
        payload = {
            'ok': True,
            'topic': topic,
            'type': resolved_type,
            'available_types': available_types,
            'message_count': watch.message_count,
            'age_sec': age_seconds(watch.last_updated_monotonic),
            'frame_ready': watch.raw_message is not None,
            'last_error': watch.last_error,
        }
        raw_message = watch.raw_message
        if raw_message is None:
            payload.update({
                'points': [],
                'total_ranges': 0,
                'valid_points': 0,
            })
            return payload

        payload.update(self._scan_points_payload(raw_message))
        return payload

    def led_strip_status(
        self,
        topic_name: str,
        type_name: str | None = None,
    ) -> dict[str, Any]:
        topic = normalize_topic_name(topic_name)
        resolved_type, available_types = self._resolve_topic_type(topic, type_name)
        if resolved_type != LED_STRIP_STATE_TYPE:
            raise ValueError(f'Topic {topic} is not a LedStripState topic')

        watch = self._ensure_topic_watch(topic, resolved_type)
        payload = {
            'ok': True,
            'topic': topic,
            'type': resolved_type,
            'available_types': available_types,
            'message_count': watch.message_count,
            'age_sec': age_seconds(watch.last_updated_monotonic),
            'frame_ready': watch.raw_message is not None,
            'last_error': watch.last_error,
        }
        raw_message = watch.raw_message
        if raw_message is None:
            payload.update({
                'connected': False,
                'enabled': False,
                'led_count': 0,
                'lit_count': 0,
                'brightness': None,
                'effect': '',
                'effect_speed_hz': None,
                'transport': '',
                'spi_bus': None,
                'spi_device': None,
                'gpio_pin': None,
                'pixel_order': '',
                'backend': '',
                'status_message': '',
                'primary_color': '#000000',
                'secondary_color': '#000000',
                'preview_colors': [],
                'frame_id': '',
            })
            return payload

        payload.update({
            'connected': bool(raw_message.connected),
            'enabled': bool(raw_message.enabled),
            'led_count': int(raw_message.led_count),
            'lit_count': int(raw_message.lit_count),
            'brightness': float(raw_message.brightness),
            'effect': str(raw_message.effect),
            'effect_speed_hz': float(raw_message.effect_speed_hz),
            'transport': str(getattr(raw_message, 'transport', '')),
            'spi_bus': int(getattr(raw_message, 'spi_bus', 0)),
            'spi_device': int(getattr(raw_message, 'spi_device', 0)),
            'gpio_pin': int(raw_message.gpio_pin),
            'pixel_order': str(raw_message.pixel_order),
            'backend': str(raw_message.backend),
            'status_message': str(raw_message.status_message),
            'primary_color': '#%02X%02X%02X' % (
                int(raw_message.red),
                int(raw_message.green),
                int(raw_message.blue),
            ),
            'secondary_color': '#%02X%02X%02X' % (
                int(raw_message.secondary_red),
                int(raw_message.secondary_green),
                int(raw_message.secondary_blue),
            ),
            'preview_colors': [int(value) for value in raw_message.preview_colors],
            'frame_id': str(raw_message.header.frame_id),
        })
        return payload

    def octoliner_status(
        self,
        topic_name: str,
        type_name: str | None = None,
    ) -> dict[str, Any]:
        topic = normalize_topic_name(topic_name)
        resolved_type, available_types = self._resolve_topic_type(topic, type_name)
        if resolved_type != OCTOLINER_READING_TYPE:
            raise ValueError(f'Topic {topic} is not an OctolinerReading topic')

        watch = self._ensure_topic_watch(topic, resolved_type)
        payload = {
            'ok': True,
            'topic': topic,
            'type': resolved_type,
            'available_types': available_types,
            'message_count': watch.message_count,
            'age_sec': age_seconds(watch.last_updated_monotonic),
            'frame_ready': watch.raw_message is not None,
            'last_error': watch.last_error,
        }
        raw_message = watch.raw_message
        if raw_message is None:
            payload.update({
                'analog_values': [],
                'pattern': 0,
                'pattern_bits': '00000000',
                'dark_sensor_count': 0,
                'line_visible': False,
                'line_position': None,
                'tracked_line_position': None,
                'sensitivity': None,
                'frame_id': '',
            })
            return payload

        pattern = int(raw_message.pattern)
        line_position = float(raw_message.line_position)
        tracked_line_position = float(raw_message.tracked_line_position)
        sensitivity = float(raw_message.sensitivity)
        payload.update({
            'analog_values': [float(value) for value in raw_message.analog_values],
            'pattern': pattern,
            'pattern_bits': format(pattern & 0xFF, '08b'),
            'dark_sensor_count': int(raw_message.dark_sensor_count),
            'line_visible': bool(raw_message.line_visible),
            'line_position': line_position if math.isfinite(line_position) else None,
            'tracked_line_position': (
                tracked_line_position
                if math.isfinite(tracked_line_position)
                else None
            ),
            'sensitivity': sensitivity if math.isfinite(sensitivity) else None,
            'frame_id': str(raw_message.header.frame_id),
        })
        return payload

    def _camera_parameter_values(self) -> dict[str, Any]:
        client = self._ensure_parameter_client(self.camera_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                f'Camera parameter services are not available for {self.camera_node_name}'
            )

        response = self._wait_for_future(
            client.get_parameters(CAMERA_PARAMETER_NAMES),
            timeout_sec=2.0,
            label='camera parameters',
        )
        values = {}
        for name, value in zip(CAMERA_PARAMETER_NAMES, response.values):
            values[name] = parameter_value_to_python(value)
        return values

    def _vision_parameter_values(self) -> dict[str, Any]:
        client = self._ensure_parameter_client(self.vision_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                f'Vision parameter services are not available for {self.vision_node_name}'
            )

        response = self._wait_for_future(
            client.get_parameters(VISION_PARAMETER_NAMES),
            timeout_sec=2.0,
            label='vision parameters',
        )
        values = {}
        for name, value in zip(VISION_PARAMETER_NAMES, response.values):
            values[name] = parameter_value_to_python(value)
        return values

    def _lidar_parameter_values(self) -> dict[str, Any]:
        client = self._ensure_parameter_client(self.lidar_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                f'Lidar parameter services are not available for {self.lidar_node_name}'
            )

        response = self._wait_for_future(
            client.get_parameters(LIDAR_PARAMETER_NAMES),
            timeout_sec=2.0,
            label='lidar parameters',
        )
        values = {}
        for name, value in zip(LIDAR_PARAMETER_NAMES, response.values):
            values[name] = parameter_value_to_python(value)
        return values

    def _led_strip_parameter_values(self) -> dict[str, Any]:
        client = self._ensure_parameter_client(self.led_strip_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                'LED strip parameter services are not available for '
                f'{self.led_strip_node_name}'
            )

        response = self._wait_for_future(
            client.get_parameters(LED_STRIP_PARAMETER_NAMES),
            timeout_sec=2.0,
            label='led strip parameters',
        )
        values = {}
        for name, value in zip(LED_STRIP_PARAMETER_NAMES, response.values):
            values[name] = parameter_value_to_python(value)
        return values

    def _octoliner_parameter_values(self) -> dict[str, Any]:
        client = self._ensure_parameter_client(self.octoliner_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                'Octoliner parameter services are not available for '
                f'{self.octoliner_node_name}'
            )

        response = self._wait_for_future(
            client.get_parameters(OCTOLINER_PARAMETER_NAMES),
            timeout_sec=2.0,
            label='octoliner parameters',
        )
        values = {}
        for name, value in zip(OCTOLINER_PARAMETER_NAMES, response.values):
            values[name] = parameter_value_to_python(value)
        return values

    def _camera_v4l2_capabilities(self, device: str) -> dict[str, Any]:
        executable = shutil.which('v4l2-ctl')
        if executable is None:
            return {
                'available': False,
                'device': device,
                'formats': [],
                'error': 'v4l2-ctl is not installed',
            }

        try:
            result = subprocess.run(
                [executable, '--device', device, '--list-formats-ext'],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.5,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return {
                'available': False,
                'device': device,
                'formats': [],
                'error': str(exc),
            }

        if result.returncode != 0:
            return {
                'available': False,
                'device': device,
                'formats': [],
                'error': result.stderr.strip() or result.stdout.strip() or 'v4l2-ctl failed',
            }

        return {
            'available': True,
            'device': device,
            'formats': parse_v4l2_capabilities(result.stdout),
        }

    def camera_settings(self) -> dict[str, Any]:
        parameters = self._camera_parameter_values()
        capabilities = self._camera_v4l2_capabilities(
            str(parameters.get('device') or '/dev/video0')
        )
        return {
            'ok': True,
            'node_name': self.camera_node_name,
            'parameters': parameters,
            'capabilities': capabilities,
        }

    def vision_settings(self) -> dict[str, Any]:
        parameters = self._vision_parameter_values()
        models_directory = resolve_models_directory(
            parameters.get('models_directory') or 'models'
        )
        models = [
            manifest_to_dict(manifest)
            for manifest in discover_model_manifests(models_directory)
        ]
        selected_model_name = str(parameters.get('model_name') or '').strip()
        selected_model = next(
            (item for item in models if item.get('id') == selected_model_name),
            None,
        )
        return {
            'ok': True,
            'node_name': self.vision_node_name,
            'parameters': parameters,
            'models_directory': str(models_directory),
            'models': models,
            'selected_model': selected_model,
            'runtime_parameters': sorted(VISION_RUNTIME_PARAMETER_NAMES),
            'notes': {
                'input_topic': 'Для обработки должен поступать raw image topic.',
                'processed_topics': (
                    'Новые image topics появятся в ROS graph, когда обработка включена '
                    'и модель успешно загружена.'
                ),
                'models_directory': (
                    'В директории models ожидаются manifest-файлы YAML и соответствующие веса.'
                ),
            },
        }

    def lidar_settings(self) -> dict[str, Any]:
        parameters = self._lidar_parameter_values()
        return {
            'ok': True,
            'node_name': self.lidar_node_name,
            'parameters': parameters,
            'runtime_parameters': sorted(LIDAR_RUNTIME_PARAMETER_NAMES),
            'notes': {
                'channel_type': 'Смена транспорта и serial-параметров обычно требует перезапуска драйвера.',
                'serial_port': 'Если устройство переобнаружено под новым путём, перезапусти lidar node.',
                'serial_baudrate': 'Изменение baudrate вступает в силу надёжнее после перезапуска ноды.',
                'scan_mode': 'Поддерживаемые режимы зависят от модели лидара.',
                'scan_frequency': 'Частота влияет на плотность точек и нагрузку.',
                'range_min': 'Минимальная дальность сейчас читается из самого LaserScan и не меняется отсюда.',
            },
        }

    def led_strip_settings(self) -> dict[str, Any]:
        parameters = self._led_strip_parameter_values()
        return {
            'ok': True,
            'node_name': self.led_strip_node_name,
            'parameters': parameters,
            'runtime_parameters': sorted(LED_STRIP_RUNTIME_PARAMETER_NAMES),
            'effects': [
                'fill',
                'blink',
                'blink_fast',
                'fade',
                'wipe',
                'flash',
                'rainbow',
                'rainbow_fill',
            ],
            'notes': {
                'state_topic': 'Изменяется только после перезапуска ноды.',
                'set_state_service': 'Изменяется только после перезапуска ноды.',
                'native_state_topic': 'Нативный статус SPI-ленты публикуется отдельно.',
                'set_effect_service': 'Нативный сервис эффекта совместим с новой реализацией.',
                'set_leds_service': 'Нативный низкоуровневый сервис покадровой настройки.',
                'wiring': (
                    'Эта реализация использует SPI MOSI через Linux spidev, а не GPIO18/one-wire. '
                    'Проверь, что data ленты подключен к MOSI соответствующей SPI шины.'
                ),
            },
        }

    def octoliner_settings(self) -> dict[str, Any]:
        parameters = self._octoliner_parameter_values()
        return {
            'ok': True,
            'node_name': self.octoliner_node_name,
            'parameters': parameters,
            'runtime_parameters': sorted(OCTOLINER_RUNTIME_PARAMETER_NAMES),
            'notes': {
                'i2c_address': 'Изменяется только после перезапуска ноды.',
                'i2c_bus': 'Изменяется только после перезапуска ноды.',
                'auto_optimize_on_start': 'Используется при следующем старте ноды.',
            },
        }

    def update_camera_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('Camera settings payload must be an object')

        updates: list[Parameter] = []
        for name in CAMERA_PARAMETER_NAMES:
            if name not in payload:
                continue

            value = payload[name]
            if name in {'device', 'image_topic', 'compressed_image_topic', 'frame_id'}:
                updates.append(Parameter(name, value=str(value)))
            elif name in {'width', 'height', 'jpeg_quality'}:
                updates.append(Parameter(name, value=int(value)))
            elif name in {'fps', 'reconnect_interval_sec'}:
                updates.append(Parameter(name, value=float(value)))
            elif name in {'use_mjpeg', 'publish_raw', 'publish_compressed'}:
                updates.append(Parameter(name, value=bool(value)))

        if not updates:
            raise ValueError('No supported camera settings were provided')

        client = self._ensure_parameter_client(self.camera_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                f'Camera parameter services are not available for {self.camera_node_name}'
            )

        response = self._wait_for_future(
            client.set_parameters(updates),
            timeout_sec=3.0,
            label='camera parameter update',
        )
        failures = [
            result.reason.strip() or 'parameter update rejected'
            for result in response.results
            if not result.successful
        ]
        if failures:
            raise RuntimeError('; '.join(failures))

        self.record_activity(
            'camera',
            'Camera settings updated',
            sanitize_payload(payload),
        )
        return self.camera_settings()

    def update_vision_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('Vision settings payload must be an object')

        updates: list[Parameter] = []
        for name in VISION_RUNTIME_PARAMETER_NAMES:
            if name not in payload:
                continue
            value = payload[name]
            if name in {
                'enabled',
                'publish_raw',
                'publish_compressed',
                'publish_detections',
                'annotate_labels',
                'annotate_confidence',
            }:
                updates.append(Parameter(name, value=bool(value)))
            elif name in {'confidence_threshold', 'nms_threshold', 'max_processing_fps'}:
                updates.append(Parameter(name, value=float(value)))
            elif name in {'line_thickness', 'jpeg_quality'}:
                updates.append(Parameter(name, value=int(value)))
            else:
                updates.append(Parameter(name, value=str(value)))

        if not updates:
            raise ValueError('No supported vision settings were provided')

        client = self._ensure_parameter_client(self.vision_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                f'Vision parameter services are not available for {self.vision_node_name}'
            )

        response = self._wait_for_future(
            client.set_parameters(updates),
            timeout_sec=3.0,
            label='vision parameter update',
        )
        failures = [
            result.reason.strip() or 'parameter update rejected'
            for result in response.results
            if not result.successful
        ]
        if failures:
            raise RuntimeError('; '.join(failures))

        self.record_activity(
            'camera',
            'Vision settings updated',
            sanitize_payload(payload),
        )
        return self.vision_settings()

    def update_lidar_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('Lidar settings payload must be an object')

        updates: list[Parameter] = []
        for name in LIDAR_RUNTIME_PARAMETER_NAMES:
            if name not in payload:
                continue
            value = payload[name]
            if name in {'serial_baudrate'}:
                updates.append(Parameter(name, value=int(value)))
            elif name in {'scan_frequency'}:
                updates.append(Parameter(name, value=float(value)))
            elif name in {'inverted', 'angle_compensate'}:
                updates.append(Parameter(name, value=bool(value)))
            else:
                updates.append(Parameter(name, value=str(value)))

        if not updates:
            raise ValueError('No supported lidar settings were provided')

        client = self._ensure_parameter_client(self.lidar_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                f'Lidar parameter services are not available for {self.lidar_node_name}'
            )

        response = self._wait_for_future(
            client.set_parameters(updates),
            timeout_sec=3.0,
            label='lidar parameter update',
        )
        failures = [
            result.reason.strip() or 'parameter update rejected'
            for result in response.results
            if not result.successful
        ]
        if failures:
            raise RuntimeError('; '.join(failures))

        self.record_activity(
            'lidar',
            'Lidar settings updated',
            sanitize_payload(payload),
        )
        return self.lidar_settings()

    def update_led_strip_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('LED strip settings payload must be an object')

        updates: list[Parameter] = []
        for name in LED_STRIP_RUNTIME_PARAMETER_NAMES:
            if name not in payload:
                continue
            value = payload[name]
            if name in {'spi_bus', 'spi_device', 'led_count'}:
                updates.append(Parameter(name, value=int(value)))
            elif name in {'brightness', 'effect_speed_hz', 'animation_rate_hz', 'state_publish_hz'}:
                updates.append(Parameter(name, value=float(value)))
            elif name == 'enabled':
                updates.append(Parameter(name, value=bool(value)))
            else:
                updates.append(Parameter(name, value=str(value)))

        if not updates:
            raise ValueError('No supported LED strip settings were provided')

        client = self._ensure_parameter_client(self.led_strip_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                'LED strip parameter services are not available for '
                f'{self.led_strip_node_name}'
            )

        response = self._wait_for_future(
            client.set_parameters(updates),
            timeout_sec=3.0,
            label='led strip parameter update',
        )
        failures = [
            result.reason.strip() or 'parameter update rejected'
            for result in response.results
            if not result.successful
        ]
        if failures:
            raise RuntimeError('; '.join(failures))

        self.record_activity(
            'lights',
            'LED strip settings updated',
            sanitize_payload(payload),
        )
        return self.led_strip_settings()

    def update_octoliner_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('Octoliner settings payload must be an object')

        updates: list[Parameter] = []
        for name in OCTOLINER_RUNTIME_PARAMETER_NAMES:
            if name not in payload:
                continue
            value = payload[name]
            if name in {'poll_rate_hz', 'sensitivity'}:
                updates.append(Parameter(name, value=float(value)))
            elif name == 'frame_id':
                updates.append(Parameter(name, value=str(value)))
            elif name == 'auto_optimize_on_start':
                updates.append(Parameter(name, value=bool(value)))

        if not updates:
            raise ValueError('No supported Octoliner settings were provided')

        client = self._ensure_parameter_client(self.octoliner_node_name)
        if not client.wait_for_services(timeout_sec=1.0):
            raise RuntimeError(
                'Octoliner parameter services are not available for '
                f'{self.octoliner_node_name}'
            )

        response = self._wait_for_future(
            client.set_parameters(updates),
            timeout_sec=3.0,
            label='octoliner parameter update',
        )
        failures = [
            result.reason.strip() or 'parameter update rejected'
            for result in response.results
            if not result.successful
        ]
        if failures:
            raise RuntimeError('; '.join(failures))

        self.record_activity(
            'octoliner',
            'Octoliner settings updated',
            sanitize_payload(payload),
        )
        return self.octoliner_settings()

    def command_led_strip(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('LED strip command payload must be an object')

        parameters = self._led_strip_parameter_values()
        service_name = str(parameters.get('set_state_service') or '').strip()
        if not service_name:
            raise RuntimeError('LED strip state service is not configured')

        handle = self._ensure_service_client(
            service_name,
            'rover_interfaces/srv/SetLedStripState',
        )
        if not handle.client.wait_for_service(timeout_sec=1.5):
            raise RuntimeError(f'Service {service_name} is not available')

        request = handle.srv_class.Request()
        request.enabled = bool(payload.get('enabled', False))
        request.brightness = float(payload.get('brightness', 0.35))
        request.effect = str(payload.get('effect', 'fill')).strip().lower()
        request.effect_speed_hz = float(payload.get('effect_speed_hz', 1.0))

        def parse_rgb(color_text: Any) -> tuple[int, int, int]:
            text = str(color_text or '#000000').strip().lstrip('#')
            if len(text) != 6:
                raise ValueError('Colors must be in #RRGGBB format')
            return (
                int(text[0:2], 16),
                int(text[2:4], 16),
                int(text[4:6], 16),
            )

        (
            request.red,
            request.green,
            request.blue,
        ) = parse_rgb(payload.get('primary_color', '#16B8F3'))
        (
            request.secondary_red,
            request.secondary_green,
            request.secondary_blue,
        ) = parse_rgb(payload.get('secondary_color', '#FFFFFF'))

        started = time.monotonic()
        future = handle.client.call_async(request)
        response = self._wait_for_future(
            future,
            timeout_sec=3.0,
            label='led strip set_state',
        )
        if response is None:
            raise RuntimeError('LED strip service returned no response')
        if not bool(response.success):
            raise RuntimeError(str(response.message or 'LED strip command failed'))

        duration = time.monotonic() - started
        self.record_activity(
            'lights',
            'LED strip command sent',
            {
                'service': service_name,
                'duration_sec': duration,
                'payload': sanitize_payload(payload),
            },
        )
        return {
            'ok': True,
            'service': service_name,
            'duration_sec': duration,
            'response': sanitize_payload(message_to_ordereddict(response)),
            'settings': self.led_strip_settings(),
        }

    def set_led_strip_pixels(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('LED strip pixels payload must be an object')

        parameters = self._led_strip_parameter_values()
        service_name = str(parameters.get('set_leds_service') or '').strip()
        if not service_name:
            raise RuntimeError('LED strip set_leds service is not configured')

        leds_payload = payload.get('leds', [])
        if not isinstance(leds_payload, list):
            raise ValueError('Field "leds" must be an array')

        handle = self._ensure_service_client(
            service_name,
            'rover_interfaces/srv/SetLEDs',
        )
        if not handle.client.wait_for_service(timeout_sec=1.5):
            raise RuntimeError(f'Service {service_name} is not available')

        request = handle.srv_class.Request()
        set_message_fields(request, {'leds': leds_payload})

        started = time.monotonic()
        future = handle.client.call_async(request)
        response = self._wait_for_future(
            future,
            timeout_sec=3.0,
            label='led strip set_leds',
        )
        if response is None:
            raise RuntimeError('LED strip set_leds service returned no response')
        if hasattr(response, 'success') and not bool(response.success):
            raise RuntimeError('LED strip manual frame was rejected')

        duration = time.monotonic() - started
        self.record_activity(
            'lights',
            'LED strip manual frame sent',
            {
                'service': service_name,
                'duration_sec': duration,
                'led_count': len(leds_payload),
            },
        )
        return {
            'ok': True,
            'service': service_name,
            'duration_sec': duration,
            'response': sanitize_payload(message_to_ordereddict(response)),
            'settings': self.led_strip_settings(),
        }

    def optimize_octoliner(self) -> dict[str, Any]:
        parameters = self._octoliner_parameter_values()
        service_name = str(parameters.get('optimize_on_black_service') or '').strip()
        if not service_name:
            raise RuntimeError('Octoliner optimize service is not configured')

        handle = self._ensure_service_client(service_name, 'std_srvs/srv/Trigger')
        if not handle.client.wait_for_service(timeout_sec=1.5):
            raise RuntimeError(f'Service {service_name} is not available')

        request = handle.srv_class.Request()
        started = time.monotonic()
        future = handle.client.call_async(request)
        response = self._wait_for_future(
            future,
            timeout_sec=4.0,
            label='octoliner optimize_on_black',
        )
        if response is None:
            raise RuntimeError('Octoliner optimize service returned no response')
        if not bool(response.success):
            raise RuntimeError(str(response.message or 'Octoliner calibration failed'))

        duration = time.monotonic() - started
        self.record_activity(
            'octoliner',
            'Octoliner optimized on black surface',
            {'service': service_name, 'duration_sec': duration},
        )
        return {
            'ok': True,
            'service': service_name,
            'duration_sec': duration,
            'response': sanitize_payload(message_to_ordereddict(response)),
            'settings': self.octoliner_settings(),
        }

    def camera_status(
        self,
        topic_name: str,
        type_name: str | None = None,
    ) -> dict[str, Any]:
        topic = normalize_topic_name(topic_name)
        resolved_type, available_types = self._resolve_topic_type(topic, type_name)
        if resolved_type not in IMAGE_TOPIC_TYPES:
            raise ValueError(f'Topic {topic} is not an image topic')
        watch = self._ensure_image_watch(topic, resolved_type)
        return {
            'ok': True,
            'topic': topic,
            'type': resolved_type,
            'available_types': available_types,
            'width': watch.width,
            'height': watch.height,
            'encoding': watch.encoding,
            'message_count': watch.message_count,
            'age_sec': age_seconds(watch.last_updated_monotonic),
            'frame_ready': watch.frame_bytes is not None,
            'last_error': watch.last_error,
            'frame_url': f'/api/camera/frame?topic={topic}&type={resolved_type}',
            'stream_url': f'/api/camera/stream?topic={topic}&type={resolved_type}',
        }

    def camera_frame(
        self,
        topic_name: str,
        type_name: str | None = None,
    ) -> tuple[bytes, str]:
        topic = normalize_topic_name(topic_name)
        resolved_type, _ = self._resolve_topic_type(topic, type_name)
        watch = self._ensure_image_watch(topic, resolved_type)
        if watch.frame_bytes is None:
            raise RuntimeError(f'No frame received yet from {topic}')
        return watch.frame_bytes, watch.content_type

    def camera_stream(
        self,
        topic_name: str,
        type_name: str | None = None,
    ) -> tuple[ImageWatch, str]:
        topic = normalize_topic_name(topic_name)
        resolved_type, _ = self._resolve_topic_type(topic, type_name)
        watch = self._ensure_image_watch(topic, resolved_type)
        return watch, 'frame'

    def drive_payload(self) -> dict[str, Any]:
        with self._lock:
            latest = self._latest_drive_command
            active = self._drive_active
            timestamp = self._latest_drive_monotonic
        return {
            'ok': True,
            'command_topic': self.command_topic,
            'timeout_sec': self.drive_command_timeout_sec,
            'defaults': {
                'linear_x': self.default_linear_speed,
                'linear_y': self.default_lateral_speed,
                'angular_z': self.default_angular_speed,
            },
            'limits': {
                'linear_x': self.max_linear_speed,
                'linear_y': self.max_lateral_speed,
                'angular_z': self.max_angular_speed,
            },
            'active': active,
            'age_sec': age_seconds(timestamp),
            'last_command': {
                'linear_x': float(latest.linear.x),
                'linear_y': float(latest.linear.y),
                'angular_z': float(latest.angular.z),
            },
        }

    def set_drive_command(self, linear_x: float, linear_y: float, angular_z: float) -> dict[str, Any]:
        command = Twist()
        command.linear.x = clamp(float(linear_x), self.max_linear_speed)
        command.linear.y = clamp(float(linear_y), self.max_lateral_speed)
        command.angular.z = clamp(float(angular_z), self.max_angular_speed)
        with self._lock:
            self._latest_drive_command = command
            self._latest_drive_monotonic = time.monotonic()
        return {
            'ok': True,
            'command': {
                'linear_x': command.linear.x,
                'linear_y': command.linear.y,
                'angular_z': command.angular.z,
            },
        }

    def stop_drive(self) -> dict[str, Any]:
        return self.set_drive_command(0.0, 0.0, 0.0)

    def _drive_output_timer(self) -> None:
        now = time.monotonic()
        with self._lock:
            fresh = now - self._latest_drive_monotonic <= self.drive_command_timeout_sec
            command = self._latest_drive_command
            was_active = self._drive_active
            stop_active = now < self._stop_until
            self._drive_active = fresh and not stop_active

        if stop_active:
            self.drive_publisher.publish(Twist())
        elif fresh:
            self.drive_publisher.publish(command)
        elif was_active:
            self.drive_publisher.publish(Twist())

    def start_motion(self, request: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._motion_process is not None and self._motion_process.poll() is None:
                raise RuntimeError('A motion command is already running')

        if not self.executor_path.exists():
            raise FileNotFoundError(f'Motion executor not found: {self.executor_path}')

        kind = str(request.get('kind', '')).strip()
        command = [
            sys.executable,
            str(self.executor_path),
            '--odom-topic',
            self.odom_topic,
            '--cmd-vel-topic',
            self.command_topic,
        ]
        summary: dict[str, Any] = {'kind': kind}

        if kind == 'plan':
            name = str(request.get('name', ''))
            path = self._safe_plan_path(name)
            if not path.exists():
                raise FileNotFoundError(name)
            command += ['run', str(path)]
            summary['name'] = name
        elif kind == 'move':
            forward = float(request.get('forward', 0.0))
            left = float(request.get('left', 0.0))
            speed = float(request.get('speed', 0.15))
            if abs(forward) > 5.0 or abs(left) > 5.0:
                raise ValueError('Move distance is limited to 5 m per command')
            if not 0.10 <= speed <= 0.35:
                raise ValueError('Move speed must be between 0.10 and 0.35 m/s')
            command += [
                'move',
                '--forward',
                str(forward),
                '--left',
                str(left),
                '--speed',
                str(speed),
            ]
            summary.update({'forward': forward, 'left': left, 'speed': speed})
        elif kind == 'turn':
            degrees = float(request.get('degrees', 0.0))
            speed = float(request.get('speed', 0.25))
            tolerance = float(request.get('tolerance_deg', 3.0))
            if abs(degrees) > 720.0:
                raise ValueError('Turn is limited to 720 degrees per command')
            if not 0.10 <= speed <= 1.0:
                raise ValueError('Angular speed must be between 0.10 and 1.0 rad/s')
            if not 1.0 <= tolerance <= 15.0:
                raise ValueError('Tolerance must be between 1 and 15 degrees')
            command += [
                'turn',
                str(degrees),
                '--speed',
                str(speed),
                '--tolerance-deg',
                str(tolerance),
            ]
            summary.update({
                'degrees': degrees,
                'speed': speed,
                'tolerance_deg': tolerance,
            })
        else:
            raise ValueError('Unknown motion kind')

        environment = os.environ.copy()
        environment.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
        process = subprocess.Popen(
            command,
            cwd=str(self.executor_path.parent.parent),
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        with self._lock:
            self._motion_process = process
            self._motion_started_at = time.time()
            self._motion_command = command
            self._motion_log.clear()
            self._motion_return_code = None

        threading.Thread(
            target=self._read_motion_output,
            args=(process,),
            name='rover-motion-output',
            daemon=True,
        ).start()
        self.record_activity('routes', 'Motion process started', summary)
        return self.motion_status_payload()

    def _read_motion_output(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            cleaned = line.rstrip('\r\n')
            with self._lock:
                self._motion_log.append(cleaned)

    def stop_motion(self, *, request_stop: bool = True) -> dict[str, Any]:
        with self._lock:
            process = self._motion_process
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            threading.Thread(
                target=self._ensure_motion_stopped,
                args=(process,),
                name='rover-motion-stop',
                daemon=True,
            ).start()
        if request_stop:
            self.request_stop('routes', {'reason': 'motion stop endpoint'})
        return self.motion_status_payload()

    def _ensure_motion_stopped(self, process: subprocess.Popen[str]) -> None:
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def motion_status_payload_locked(self) -> dict[str, Any]:
        process = self._motion_process
        running = process is not None and process.poll() is None
        return {
            'running': running,
            'pid': process.pid if running and process is not None else None,
            'started_at': self._motion_started_at,
            'return_code': self._motion_return_code,
            'command': list(self._motion_command),
            'log': list(self._motion_log)[-120:],
        }

    def motion_status_payload(self) -> dict[str, Any]:
        with self._lock:
            return self.motion_status_payload_locked()

    def _resolve_hackathon_file(self, relative_path: str) -> Path:
        requested = relative_path.strip().replace('\\', '/').lstrip('/')
        if not requested:
            raise FileNotFoundError('Hackathon file name is empty')
        candidate = (self.hackathon_files_root / requested).resolve()
        root = self.hackathon_files_root.resolve()
        if os.path.commonpath([str(root), str(candidate)]) != str(root):
            raise PermissionError('Forbidden')
        if candidate.suffix.lower() not in HACKATHON_FILE_TYPES:
            raise PermissionError('Unsupported hackathon file type')
        if not candidate.exists() or not candidate.is_file():
            raise FileNotFoundError(f'{relative_path} not found')
        return candidate

    def hackathon_files_payload(self) -> dict[str, Any]:
        root = self.hackathon_files_root.resolve()
        files: list[dict[str, Any]] = []
        if root.exists():
            for path in sorted(root.rglob('*')):
                if not path.is_file():
                    continue
                info = HACKATHON_FILE_TYPES.get(path.suffix.lower())
                if info is None:
                    continue
                kind, content_type = info
                relative_path = path.relative_to(root).as_posix()
                try:
                    size_bytes = path.stat().st_size
                except OSError:
                    size_bytes = 0
                files.append({
                    'path': relative_path,
                    'name': path.name,
                    'kind': kind,
                    'content_type': content_type,
                    'size_bytes': size_bytes,
                })
        return {
            'root': str(root),
            'files': files,
        }

    def hackathon_file(self, relative_path: str) -> tuple[bytes, str]:
        path = self._resolve_hackathon_file(relative_path)
        _, content_type = HACKATHON_FILE_TYPES[path.suffix.lower()]
        return path.read_bytes(), content_type

    def _resolve_map_yaml(self, relative_path: str) -> Path:
        requested = relative_path.strip().replace('\\', '/').lstrip('/')
        if not requested:
            raise FileNotFoundError('Map file name is empty')
        candidate = (self.maps_root / requested).resolve()
        root = self.maps_root.resolve()
        if os.path.commonpath([str(root), str(candidate)]) != str(root):
            raise PermissionError('Forbidden')
        if candidate.suffix.lower() not in MAP_YAML_EXTENSIONS:
            raise PermissionError('Unsupported map metadata file type')
        if not candidate.exists() or not candidate.is_file():
            raise FileNotFoundError(f'{relative_path} not found')
        return candidate

    def _resolve_map_image(self, map_yaml: Path, metadata: dict[str, Any]) -> Path:
        image_name = str(metadata.get('image') or '').strip()
        if not image_name:
            raise FileNotFoundError(f'{map_yaml.name} does not specify an image')
        image_path = Path(image_name).expanduser()
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path
        image_path = image_path.resolve()
        root = self.maps_root.resolve()
        if os.path.commonpath([str(root), str(image_path)]) != str(root):
            raise PermissionError('Map image must stay inside maps_root')
        if image_path.suffix.lower() not in MAP_IMAGE_EXTENSIONS:
            raise PermissionError('Unsupported map image type')
        if not image_path.exists() or not image_path.is_file():
            raise FileNotFoundError(f'{image_name} not found')
        return image_path

    @staticmethod
    def _map_origin(metadata: dict[str, Any]) -> list[float]:
        raw_origin = metadata.get('origin', [0.0, 0.0, 0.0])
        if not isinstance(raw_origin, list):
            raw_origin = [0.0, 0.0, 0.0]
        values = [0.0, 0.0, 0.0]
        for index, value in enumerate(raw_origin[:3]):
            try:
                values[index] = float(value)
            except (TypeError, ValueError):
                values[index] = 0.0
        return values

    def _map_metadata_payload(self, map_yaml: Path) -> dict[str, Any]:
        metadata = read_yaml(map_yaml, {})
        image_path = self._resolve_map_image(map_yaml, metadata)
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError(f'OpenCV could not read {image_path.name}')
        height, width = image.shape[:2]
        try:
            resolution = float(metadata.get('resolution', 0.05))
        except (TypeError, ValueError):
            resolution = 0.05
        if not math.isfinite(resolution) or resolution <= 0.0:
            resolution = 0.05

        relative_yaml = map_yaml.relative_to(self.maps_root).as_posix()
        relative_image = image_path.relative_to(self.maps_root).as_posix()
        return {
            'path': relative_yaml,
            'name': map_yaml.stem,
            'image': relative_image,
            'image_url': f'/api/maps/image?map={quote(relative_yaml)}',
            'resolution': resolution,
            'origin': self._map_origin(metadata),
            'negate': int(metadata.get('negate', 0) or 0),
            'occupied_thresh': float(metadata.get('occupied_thresh', 0.65)),
            'free_thresh': float(metadata.get('free_thresh', 0.196)),
            'width_px': int(width),
            'height_px': int(height),
            'width_m': float(width) * resolution,
            'height_m': float(height) * resolution,
            'valid': True,
        }

    def maps_payload(self) -> dict[str, Any]:
        root = self.maps_root.resolve()
        maps: list[dict[str, Any]] = []
        if root.exists():
            for path in sorted(root.rglob('*')):
                if not path.is_file() or path.suffix.lower() not in MAP_YAML_EXTENSIONS:
                    continue
                try:
                    maps.append(self._map_metadata_payload(path))
                except Exception as exc:
                    maps.append({
                        'path': path.relative_to(root).as_posix(),
                        'name': path.stem,
                        'valid': False,
                        'error': str(exc),
                    })
        return {
            'root': str(root),
            'maps': maps,
        }

    def map_image(self, relative_path: str) -> tuple[bytes, str]:
        map_yaml = self._resolve_map_yaml(relative_path)
        metadata = read_yaml(map_yaml, {})
        image_path = self._resolve_map_image(map_yaml, metadata)
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError(f'OpenCV could not read {image_path.name}')
        ok, encoded = cv2.imencode('.png', image)
        if not ok:
            raise RuntimeError(f'OpenCV could not encode {image_path.name} as PNG')
        return encoded.tobytes(), 'image/png'

    def _serve_static_file(self, request_path: str) -> tuple[bytes, str]:
        relative_path = 'index.html' if request_path in ('', '/') else request_path.lstrip('/')
        candidate = os.path.normpath(os.path.join(str(self.web_root), relative_path))
        if os.path.commonpath([str(self.web_root), candidate]) != str(self.web_root):
            raise PermissionError('Forbidden')
        path = Path(candidate)
        if not path.exists() or not path.is_file():
            raise FileNotFoundError(f'{request_path} not found')
        content_type, _ = mimetypes.guess_type(str(path))
        return path.read_bytes(), content_type or 'application/octet-stream'

    def _build_handler(self):
        gateway = self

        class Handler(BaseHTTPRequestHandler):
            server_version = 'RoverWeb/0.2'

            def do_GET(self) -> None:  # noqa: N802
                try:
                    parsed = urlparse(self.path)
                    path = parsed.path
                    query = parse_qs(parsed.query)

                    if path == '/api/health':
                        self._send_json(
                            {'ok': True, 'service': 'rover_web'},
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/identity':
                        self._send_json(gateway.identity_payload(), HTTPStatus.OK)
                        return
                    if path == '/api/config':
                        self._send_json(gateway.public_config_payload(), HTTPStatus.OK)
                        return
                    if path == '/api/status':
                        self._send_json(gateway.status_payload(), HTTPStatus.OK)
                        return
                    if path == '/api/activity':
                        limit_text = self._optional_query(query, 'limit') or '100'
                        self._send_json(
                            {'items': gateway.activity_payload(int(limit_text))},
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/system':
                        self._send_json(gateway._system_summary(), HTTPStatus.OK)
                        return
                    if path == '/api/hackathon/files':
                        self._send_json(gateway.hackathon_files_payload(), HTTPStatus.OK)
                        return
                    if path == '/api/maps':
                        self._send_json(gateway.maps_payload(), HTTPStatus.OK)
                        return
                    if path == '/api/maps/image':
                        map_path = self._required_query(query, 'map')
                        image_bytes, content_type = gateway.map_image(map_path)
                        self._send_bytes(image_bytes, content_type, HTTPStatus.OK)
                        return
                    if path == '/api/plans':
                        self._send_json(
                            {'plans': gateway.list_plans()},
                            HTTPStatus.OK,
                        )
                        return
                    if path.startswith('/api/plans/'):
                        name = path.removeprefix('/api/plans/')
                        self._send_json(
                            {'name': name, 'plan': gateway.read_plan(name)},
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/motion/status':
                        self._send_json(gateway.motion_status_payload(), HTTPStatus.OK)
                        return
                    if path == '/api/voice/status':
                        self._send_json(gateway.voice_status_payload(), HTTPStatus.OK)
                        return
                    if path == '/api/ros/graph':
                        self._send_json(gateway._graph_payload(), HTTPStatus.OK)
                        return
                    if path == '/api/ros/topic':
                        topic = self._required_query(query, 'name')
                        type_name = self._optional_query(query, 'type')
                        self._send_json(
                            gateway.inspect_topic(topic, type_name),
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/ros/service':
                        service = self._required_query(query, 'name')
                        type_name = self._optional_query(query, 'type')
                        self._send_json(
                            gateway.inspect_service(service, type_name),
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/camera/topics':
                        self._send_json(gateway.camera_topics(), HTTPStatus.OK)
                        return
                    if path == '/api/lidar/topics':
                        self._send_json(gateway.lidar_topics(), HTTPStatus.OK)
                        return
                    if path == '/api/led_strip/topics':
                        self._send_json(gateway.led_strip_topics(), HTTPStatus.OK)
                        return
                    if path == '/api/octoliner/topics':
                        self._send_json(gateway.octoliner_topics(), HTTPStatus.OK)
                        return
                    if path == '/api/vision/settings':
                        self._send_json(gateway.vision_settings(), HTTPStatus.OK)
                        return
                    if path == '/api/lidar/settings':
                        self._send_json(gateway.lidar_settings(), HTTPStatus.OK)
                        return
                    if path == '/api/led_strip/settings':
                        self._send_json(gateway.led_strip_settings(), HTTPStatus.OK)
                        return
                    if path == '/api/octoliner/settings':
                        self._send_json(gateway.octoliner_settings(), HTTPStatus.OK)
                        return
                    if path == '/api/camera/settings':
                        self._send_json(gateway.camera_settings(), HTTPStatus.OK)
                        return
                    if path == '/api/camera/status':
                        topic = self._required_query(query, 'topic')
                        type_name = self._optional_query(query, 'type')
                        self._send_json(
                            gateway.camera_status(topic, type_name),
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/camera/frame':
                        topic = self._required_query(query, 'topic')
                        type_name = self._optional_query(query, 'type')
                        frame_bytes, content_type = gateway.camera_frame(topic, type_name)
                        self._send_bytes(frame_bytes, content_type, HTTPStatus.OK)
                        return
                    if path == '/api/camera/stream':
                        topic = self._required_query(query, 'topic')
                        type_name = self._optional_query(query, 'type')
                        watch, boundary = gateway.camera_stream(topic, type_name)
                        self._send_mjpeg_stream(watch, boundary)
                        return
                    if path == '/api/lidar/status':
                        topic = self._required_query(query, 'topic')
                        type_name = self._optional_query(query, 'type')
                        self._send_json(
                            gateway.lidar_status(topic, type_name),
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/led_strip/status':
                        topic = self._required_query(query, 'topic')
                        type_name = self._optional_query(query, 'type')
                        self._send_json(
                            gateway.led_strip_status(topic, type_name),
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/octoliner/status':
                        topic = self._required_query(query, 'topic')
                        type_name = self._optional_query(query, 'type')
                        self._send_json(
                            gateway.octoliner_status(topic, type_name),
                            HTTPStatus.OK,
                        )
                        return
                    if path == '/api/drive':
                        self._send_json(gateway.drive_payload(), HTTPStatus.OK)
                        return
                    if path.startswith('/hackathon-files/'):
                        relative_path = unquote(path.removeprefix('/hackathon-files/'))
                        payload, content_type = gateway.hackathon_file(relative_path)
                        self._send_bytes(payload, content_type, HTTPStatus.OK, cache=False)
                        return

                    body, content_type = gateway._serve_static_file(path)
                    self._send_bytes(body, content_type, HTTPStatus.OK, cache=False)
                except FileNotFoundError:
                    self._send_error(HTTPStatus.NOT_FOUND, 'Not found')
                except PermissionError:
                    self._send_error(HTTPStatus.FORBIDDEN, 'Forbidden')
                except ValueError as exc:
                    self._send_error(HTTPStatus.BAD_REQUEST, str(exc))
                except RuntimeError as exc:
                    self._send_error(HTTPStatus.SERVICE_UNAVAILABLE, str(exc))
                except Exception as exc:
                    gateway.get_logger().error(f'GET {self.path} failed: {exc}')
                    self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))

            def do_POST(self) -> None:  # noqa: N802
                try:
                    parsed = urlparse(self.path)
                    payload = self._read_json_body()
                    if parsed.path == '/api/heartbeat':
                        gateway.register_heartbeat(
                            str(payload.get('session_id', '')),
                            str(payload.get('page', '')),
                            self._client_ip(),
                        )
                        self._send_json({'ok': True}, HTTPStatus.OK)
                        return
                    if parsed.path == '/api/activity':
                        gateway.record_activity(
                            str(payload.get('source', 'web')),
                            str(payload.get('message', 'Activity')),
                            payload.get('details', {})
                            if isinstance(payload.get('details', {}), dict)
                            else {},
                        )
                        self._send_json({'ok': True}, HTTPStatus.OK)
                        return
                    if parsed.path == '/api/stop':
                        details = (
                            payload.get('details', {})
                            if isinstance(payload.get('details', {}), dict)
                            else {}
                        )
                        gateway.request_stop(
                            str(payload.get('source', 'web')),
                            {'client_ip': self._client_ip(), **details},
                        )
                        self._send_json(
                            {
                                'ok': True,
                                'motion': gateway.stop_motion(request_stop=False),
                            },
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/ros/topic/publish':
                        topic = payload.get('topic') or ''
                        type_name = payload.get('type')
                        message = payload.get('message', {})
                        self._send_json(
                            gateway.publish_topic(str(topic), message, type_name),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/ros/service/call':
                        service = payload.get('service') or ''
                        type_name = payload.get('type')
                        request_payload = payload.get('request', {})
                        self._send_json(
                            gateway.call_service(str(service), request_payload, type_name),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/drive/command':
                        self._send_json(
                            gateway.set_drive_command(
                                float(payload.get('linear_x', 0.0)),
                                float(payload.get('linear_y', 0.0)),
                                float(payload.get('angular_z', 0.0)),
                            ),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/camera/settings':
                        self._send_json(
                            gateway.update_camera_settings(payload),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/vision/settings':
                        self._send_json(
                            gateway.update_vision_settings(payload),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/lidar/settings':
                        self._send_json(
                            gateway.update_lidar_settings(payload),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/led_strip/settings':
                        self._send_json(
                            gateway.update_led_strip_settings(payload),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/led_strip/command':
                        self._send_json(
                            gateway.command_led_strip(payload),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/led_strip/leds':
                        self._send_json(
                            gateway.set_led_strip_pixels(payload),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/octoliner/settings':
                        self._send_json(
                            gateway.update_octoliner_settings(payload),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/octoliner/optimize':
                        self._send_json(
                            gateway.optimize_octoliner(),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/drive/stop':
                        self._send_json(gateway.stop_drive(), HTTPStatus.OK)
                        return
                    if parsed.path == '/api/motion/start':
                        self._send_json(
                            {'ok': True, 'motion': gateway.start_motion(payload)},
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/motion/stop':
                        self._send_json(
                            {'ok': True, 'motion': gateway.stop_motion()},
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/voice/control':
                        self._send_json(
                            gateway.set_voice_enabled(
                                bool(payload.get('enabled', False))
                            ),
                            HTTPStatus.OK,
                        )
                        return
                    if parsed.path == '/api/plans/save':
                        name = str(payload.get('name', ''))
                        plan = payload.get('plan')
                        if not isinstance(plan, dict):
                            raise ValueError('plan must be an object')
                        gateway.save_plan(name, plan)
                        self._send_json({'ok': True}, HTTPStatus.OK)
                        return
                    self._send_error(HTTPStatus.NOT_FOUND, 'Not found')
                except json.JSONDecodeError:
                    self._send_error(HTTPStatus.BAD_REQUEST, 'Invalid JSON')
                except ValueError as exc:
                    self._send_error(HTTPStatus.BAD_REQUEST, str(exc))
                except RuntimeError as exc:
                    self._send_error(HTTPStatus.SERVICE_UNAVAILABLE, str(exc))
                except Exception as exc:
                    gateway.get_logger().error(f'POST {self.path} failed: {exc}')
                    self._send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))

            def _optional_query(
                self,
                query: dict[str, list[str]],
                key: str,
            ) -> str | None:
                values = query.get(key)
                if not values:
                    return None
                text = values[0].strip()
                return text or None

            def _required_query(self, query: dict[str, list[str]], key: str) -> str:
                value = self._optional_query(query, key)
                if value is None:
                    raise ValueError(f'Missing query parameter: {key}')
                return value

            def _client_ip(self) -> str:
                forwarded = self.headers.get('X-Forwarded-For', '').split(',')[0].strip()
                return forwarded or self.client_address[0]

            def _read_json_body(self) -> dict[str, Any]:
                content_length = int(self.headers.get('Content-Length', '0'))
                if content_length > MAX_REQUEST_BYTES:
                    raise ValueError('Request body is too large')
                raw = self.rfile.read(content_length) if content_length > 0 else b'{}'
                decoded = json.loads(raw.decode('utf-8'))
                if not isinstance(decoded, dict):
                    raise ValueError('JSON body must be an object')
                return decoded

            def _send_json(self, payload: dict[str, Any], status: HTTPStatus) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _send_bytes(
                self,
                payload: bytes,
                content_type: str,
                status: HTTPStatus,
                *,
                cache: bool = False,
            ) -> None:
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(payload)))
                self.send_header(
                    'Cache-Control',
                    'public, max-age=3600' if cache else 'no-store',
                )
                self.end_headers()
                self.wfile.write(payload)

            def _send_mjpeg_stream(
                self,
                watch: ImageWatch,
                boundary: str,
            ) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    'Content-Type',
                    f'multipart/x-mixed-replace; boundary={boundary}',
                )
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()

                last_count = -1
                while True:
                    with gateway._lock:
                        count = watch.message_count
                        payload = watch.frame_bytes
                        content_type = watch.content_type or 'image/jpeg'
                    if payload is None:
                        time.sleep(0.05)
                        continue
                    if count == last_count:
                        time.sleep(0.02)
                        continue

                    try:
                        self.wfile.write(f'--{boundary}\r\n'.encode('ascii'))
                        self.wfile.write(
                            f'Content-Type: {content_type}\r\n'.encode('ascii')
                        )
                        self.wfile.write(
                            f'Content-Length: {len(payload)}\r\n\r\n'.encode('ascii')
                        )
                        self.wfile.write(payload)
                        self.wfile.write(b'\r\n')
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    last_count = count

            def _send_error(self, status: HTTPStatus, message: str) -> None:
                payload = {'ok': False, 'error': message}
                body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format: str, *args) -> None:
                return

        return Handler

    def destroy_node(self) -> bool:
        self.request_stop('system', {'reason': 'gateway shutdown'})
        try:
            self.stop_motion()
        except Exception:
            pass
        try:
            self.stop_voice_processing()
        except Exception:
            pass
        try:
            self.drive_publisher.publish(Twist())
        except Exception:
            pass
        try:
            self._http_server.shutdown()
            self._http_server.server_close()
        except Exception:
            pass
        self._http_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoverWebGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
