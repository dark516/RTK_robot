from __future__ import annotations

import math
import threading
import time
from typing import Optional

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rover_interfaces.srv import GetFrame
from sensor_msgs.msg import CompressedImage, Image


def as_capture_source(device: str) -> str | int:
    text = device.strip()
    if text.isdigit():
        return int(text)
    return text


class UsbCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('usb_camera_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter(
            'compressed_image_topic',
            '/image_raw/compressed',
        )
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('use_mjpeg', True)
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('reconnect_interval_sec', 2.0)
        self.declare_parameter('get_frame_service', 'get_frame')

        self._config_lock = threading.RLock()
        self.capture_lock = threading.RLock()
        self.frame_lock = threading.RLock()

        self.capture: Optional[cv2.VideoCapture] = None
        self.raw_publisher = None
        self.compressed_publisher = None
        self.publish_timer = None

        self.last_open_attempt = 0.0
        self.last_warn_time = 0.0
        self.reconfigure_capture = False
        self.shutdown_event = threading.Event()

        self.latest_frame: Optional[np.ndarray] = None
        self.latest_compressed: Optional[bytes] = None
        self.latest_frame_seq = 0
        self.latest_header_stamp = None
        self.latest_width = 0
        self.latest_height = 0
        self.latest_capture_monotonic = 0.0

        self.actual_width = 0
        self.actual_height = 0
        self.actual_fps = 0.0
        self.actual_fourcc = ''

        self.frames_captured = 0
        self.frames_published_raw = 0
        self.frames_published_compressed = 0
        self.read_failures = 0

        self.last_published_seq_raw = 0
        self.last_published_seq_compressed = 0

        self._load_parameters()
        self._configure_publishers()
        self._configure_timer()
        self._configure_services()
        self.add_on_set_parameters_callback(self._handle_parameter_update)

        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            name='usb-camera-capture',
            daemon=True,
        )
        self.capture_thread.start()

        self._request_reopen(force=True)

    def _load_parameters(self) -> None:
        self.device = str(self.get_parameter('device').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.compressed_image_topic = str(
            self.get_parameter('compressed_image_topic').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.use_mjpeg = bool(self.get_parameter('use_mjpeg').value)
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.publish_compressed = bool(
            self.get_parameter('publish_compressed').value
        )
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value
        )
        self.get_frame_service_name = str(
            self.get_parameter('get_frame_service').value
        )
        self._validate_configuration()

    def _validate_configuration(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError('width and height must be positive')
        if not math.isfinite(self.fps) or self.fps <= 0.0:
            raise ValueError('fps must be finite and positive')
        if not 10 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 10 and 100')
        if (
            not math.isfinite(self.reconnect_interval)
            or self.reconnect_interval <= 0.0
        ):
            raise ValueError('reconnect_interval_sec must be finite and positive')
        if not self.publish_raw and not self.publish_compressed:
            raise ValueError(
                'At least one of publish_raw or publish_compressed must be true'
            )
        if not self.get_frame_service_name.strip():
            raise ValueError('get_frame_service must not be empty')

    def _configure_publishers(self) -> None:
        if self.raw_publisher is not None:
            self.destroy_publisher(self.raw_publisher)
            self.raw_publisher = None
        if self.compressed_publisher is not None:
            self.destroy_publisher(self.compressed_publisher)
            self.compressed_publisher = None

        if self.publish_raw:
            self.raw_publisher = self.create_publisher(
                Image,
                self.image_topic,
                qos_profile_sensor_data,
            )
        if self.publish_compressed:
            self.compressed_publisher = self.create_publisher(
                CompressedImage,
                self.compressed_image_topic,
                qos_profile_sensor_data,
            )

    def _configure_timer(self) -> None:
        if self.publish_timer is not None:
            self.destroy_timer(self.publish_timer)
            self.publish_timer = None
        self.publish_timer = self.create_timer(
            max(1.0 / self.fps, 0.001),
            self._publish_latest_frame,
        )

    def _configure_services(self) -> None:
        if getattr(self, 'get_frame_service', None) is not None:
            self.destroy_service(self.get_frame_service)
        self.get_frame_service = self.create_service(
            GetFrame,
            self.get_frame_service_name,
            self._handle_get_frame,
        )

    def _handle_parameter_update(
        self,
        parameters: list[Parameter],
    ) -> SetParametersResult:
        candidate = {
            'device': self.device,
            'image_topic': self.image_topic,
            'compressed_image_topic': self.compressed_image_topic,
            'frame_id': self.frame_id,
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'use_mjpeg': self.use_mjpeg,
            'publish_raw': self.publish_raw,
            'publish_compressed': self.publish_compressed,
            'jpeg_quality': self.jpeg_quality,
            'reconnect_interval_sec': self.reconnect_interval,
            'get_frame_service': self.get_frame_service_name,
        }

        try:
            for parameter in parameters:
                if parameter.name not in candidate:
                    continue
                candidate[parameter.name] = parameter.value

            width = int(candidate['width'])
            height = int(candidate['height'])
            fps = float(candidate['fps'])
            jpeg_quality = int(candidate['jpeg_quality'])
            reconnect_interval = float(candidate['reconnect_interval_sec'])
            publish_raw = bool(candidate['publish_raw'])
            publish_compressed = bool(candidate['publish_compressed'])
            get_frame_service_name = str(candidate['get_frame_service'])

            if width <= 0 or height <= 0:
                raise ValueError('width and height must be positive')
            if not math.isfinite(fps) or fps <= 0.0:
                raise ValueError('fps must be finite and positive')
            if not 10 <= jpeg_quality <= 100:
                raise ValueError('jpeg_quality must be between 10 and 100')
            if (
                not math.isfinite(reconnect_interval)
                or reconnect_interval <= 0.0
            ):
                raise ValueError(
                    'reconnect_interval_sec must be finite and positive'
                )
            if not publish_raw and not publish_compressed:
                raise ValueError(
                    'At least one of publish_raw or publish_compressed must be true'
                )
            if not get_frame_service_name.strip():
                raise ValueError('get_frame_service must not be empty')
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        with self._config_lock:
            old_publish = (
                self.publish_raw,
                self.publish_compressed,
                self.image_topic,
                self.compressed_image_topic,
            )
            old_capture = (
                self.device,
                self.width,
                self.height,
                self.fps,
                self.use_mjpeg,
                self.jpeg_quality,
                self.frame_id,
            )
            old_service_name = self.get_frame_service_name

            self.device = str(candidate['device'])
            self.image_topic = str(candidate['image_topic'])
            self.compressed_image_topic = str(candidate['compressed_image_topic'])
            self.frame_id = str(candidate['frame_id'])
            self.width = width
            self.height = height
            self.fps = fps
            self.use_mjpeg = bool(candidate['use_mjpeg'])
            self.publish_raw = publish_raw
            self.publish_compressed = publish_compressed
            self.jpeg_quality = jpeg_quality
            self.reconnect_interval = reconnect_interval
            self.get_frame_service_name = get_frame_service_name

            if old_publish != (
                self.publish_raw,
                self.publish_compressed,
                self.image_topic,
                self.compressed_image_topic,
            ):
                self._configure_publishers()

            if old_capture != (
                self.device,
                self.width,
                self.height,
                self.fps,
                self.use_mjpeg,
                self.jpeg_quality,
                self.frame_id,
            ):
                self._configure_timer()
                self._request_reopen(force=True)

            if old_service_name != self.get_frame_service_name:
                self._configure_services()

        self.get_logger().info(
            'Camera parameters updated: '
            f'{self.width}x{self.height} @ {self.fps:.1f} fps, '
            f'MJPEG={self.use_mjpeg}, raw={self.publish_raw}, '
            f'compressed={self.publish_compressed}'
        )
        return SetParametersResult(successful=True)

    def _warn_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_warn_time >= 2.0:
            self.get_logger().warning(text)
            self.last_warn_time = now

    def _request_reopen(self, *, force: bool = False) -> None:
        with self.capture_lock:
            self.reconfigure_capture = True
            if force:
                self.last_open_attempt = 0.0
            if self.capture is not None:
                self.capture.release()
                self.capture = None

    def _open_camera(self) -> bool:
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval:
            return False

        self.last_open_attempt = now
        source = as_capture_source(self.device)
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]

        for backend in backends:
            capture = cv2.VideoCapture(source, backend)
            if not capture.isOpened():
                capture.release()
                continue

            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)
            if self.use_mjpeg:
                capture.set(
                    cv2.CAP_PROP_FOURCC,
                    float(cv2.VideoWriter_fourcc(*'MJPG')),
                )
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
            capture.set(cv2.CAP_PROP_FPS, float(self.fps))

            with self.capture_lock:
                if self.capture is not None:
                    self.capture.release()
                self.capture = capture
                self.reconfigure_capture = False

            self.actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
            fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
            self.actual_fourcc = ''.join(
                chr((fourcc >> shift) & 0xFF) for shift in (0, 8, 16, 24)
            ).strip('\x00')
            self.get_logger().info(
                f'USB camera connected: {self.device} -> '
                f'{self.actual_width}x{self.actual_height} @ '
                f'{self.actual_fps:.1f} fps, fourcc={self.actual_fourcc or "unknown"}'
            )
            return True

        self._warn_throttled(
            f'Cannot open USB camera {self.device}; retrying automatically'
        )
        return False

    def _encode_frame(self, frame: np.ndarray) -> bytes:
        ok, encoded = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError('OpenCV could not encode frame as JPEG')
        return encoded.tobytes()

    def _capture_loop(self) -> None:
        while not self.shutdown_event.is_set():
            with self.capture_lock:
                capture = self.capture
                needs_reopen = self.reconfigure_capture

            if needs_reopen or capture is None or not capture.isOpened():
                if not self._open_camera():
                    self.shutdown_event.wait(0.05)
                continue

            ok, frame = capture.read()
            if not ok or frame is None:
                self.read_failures += 1
                self._warn_throttled(
                    f'Failed to read frame from {self.device}; reconnecting'
                )
                self._request_reopen()
                self.shutdown_event.wait(0.05)
                continue

            if len(frame.shape) != 3 or frame.shape[2] != 3:
                self._warn_throttled(
                    'Camera returned a non-BGR frame; skipping this image'
                )
                self.shutdown_event.wait(0.001)
                continue

            compressed = None
            should_encode_compressed = (
                self.publish_compressed
                and self.compressed_publisher is not None
                and self.compressed_publisher.get_subscription_count() > 0
            )
            if should_encode_compressed:
                try:
                    compressed = self._encode_frame(frame)
                except Exception as exc:
                    self._warn_throttled(f'JPEG encode failed: {exc}')

            with self.frame_lock:
                self.latest_frame = frame
                self.latest_compressed = compressed
                self.latest_frame_seq += 1
                self.latest_header_stamp = self.get_clock().now().to_msg()
                self.latest_width = int(frame.shape[1])
                self.latest_height = int(frame.shape[0])
                self.latest_capture_monotonic = time.monotonic()
                self.frames_captured += 1

        self._request_reopen(force=True)

    def _publish_latest_frame(self) -> None:
        with self.frame_lock:
            if self.latest_frame_seq <= 0:
                return

            sequence = self.latest_frame_seq
            stamp = self.latest_header_stamp
            width = self.latest_width
            height = self.latest_height

            raw_frame = None
            should_publish_raw = (
                self.publish_raw
                and self.raw_publisher is not None
                and self.raw_publisher.get_subscription_count() > 0
            )
            if should_publish_raw and sequence != self.last_published_seq_raw:
                raw_frame = None if self.latest_frame is None else self.latest_frame.copy()

            compressed_frame = None
            if (
                self.publish_compressed
                and sequence != self.last_published_seq_compressed
            ):
                compressed_frame = self.latest_compressed

        if raw_frame is not None and self.raw_publisher is not None:
            message = Image()
            message.header.stamp = stamp
            message.header.frame_id = self.frame_id
            message.height = height
            message.width = width
            message.encoding = 'bgr8'
            message.is_bigendian = False
            message.step = int(width * raw_frame.shape[2])
            message.data = raw_frame.tobytes()
            self.raw_publisher.publish(message)
            self.frames_published_raw += 1
            self.last_published_seq_raw = sequence

        if (
            compressed_frame is not None
            and self.compressed_publisher is not None
        ):
            message = CompressedImage()
            message.header.stamp = stamp
            message.header.frame_id = self.frame_id
            message.format = 'jpeg'
            message.data = compressed_frame
            self.compressed_publisher.publish(message)
            self.frames_published_compressed += 1
            self.last_published_seq_compressed = sequence

    def _handle_get_frame(
        self,
        _request: GetFrame.Request,
        response: GetFrame.Response,
    ) -> GetFrame.Response:
        with self.frame_lock:
            latest_frame = None if self.latest_frame is None else self.latest_frame.copy()
            latest_compressed = self.latest_compressed
            stamp = self.latest_header_stamp
            width = self.latest_width
            height = self.latest_height
            age_sec = (
                max(0.0, time.monotonic() - self.latest_capture_monotonic)
                if self.latest_capture_monotonic > 0.0
                else float('inf')
            )

        if latest_frame is None or stamp is None or width <= 0 or height <= 0:
            response.success = False
            response.message = 'No camera frame is available yet'
            response.age_sec = float('inf')
            return response

        if latest_compressed is None:
            try:
                latest_compressed = self._encode_frame(latest_frame)
            except Exception as exc:
                response.success = False
                response.message = f'JPEG encode failed: {exc}'
                response.age_sec = float(age_sec)
                return response

        response.success = True
        response.message = 'ok'
        response.frame.header.stamp = stamp
        response.frame.header.frame_id = self.frame_id
        response.frame.format = 'jpeg'
        response.frame.data = latest_compressed
        response.width = int(width)
        response.height = int(height)
        response.age_sec = float(age_sec)
        return response

    def close(self) -> None:
        self.shutdown_event.set()
        self._request_reopen(force=True)
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[UsbCameraNode] = None
    try:
        node = UsbCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
