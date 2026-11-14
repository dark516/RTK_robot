from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
from typing import Any, Optional

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from rover_vision.model_registry import resolve_models_directory


FIXED_MODEL_ID = 'ssd_mobilenet_v1_coco_2017_11_17'
FIXED_MODEL_DISPLAY_NAME = 'SSD MobileNet v1 COCO'
FIXED_MODEL_WEIGHTS = 'frozen_inference_graph.pb'
FIXED_MODEL_CONFIG = 'ssd_mobilenet_v1_coco_2017_11_17.pbtxt'
FIXED_MODEL_LABELS = 'object_detection_classes_coco.txt'
FIXED_MODEL_INPUT_SIZE = (300, 300)


@dataclass(slots=True)
class Detection:
    class_id: int
    label: str
    confidence: float
    x: int
    y: int
    width: int
    height: int


def clamp_int(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def image_message_to_bgr(message: Image) -> np.ndarray:
    height = int(message.height)
    width = int(message.width)
    encoding = str(message.encoding).lower()
    if height <= 0 or width <= 0:
        raise ValueError('Image dimensions must be positive')

    buffer = np.frombuffer(message.data, dtype=np.uint8)
    if encoding in {'bgr8', 'rgb8'}:
        array = buffer.reshape((height, width, 3))
        if encoding == 'rgb8':
            return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        return array.copy()
    if encoding in {'bgra8', 'rgba8'}:
        array = buffer.reshape((height, width, 4))
        if encoding == 'rgba8':
            return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    if encoding in {'mono8', '8uc1'}:
        array = buffer.reshape((height, width))
        return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'Unsupported input encoding: {message.encoding}')


def encode_jpeg(frame: np.ndarray, quality: int) -> bytes:
    ok, encoded = cv2.imencode(
        '.jpg',
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )
    if not ok:
        raise RuntimeError('OpenCV could not encode annotated frame as JPEG')
    return encoded.tobytes()


def load_labels_file(path: str) -> list[str]:
    with open(path, 'r', encoding='utf-8') as handle:
        return [line.strip() for line in handle.readlines() if line.strip()]


class CameraDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_detector_node')

        self.declare_parameter('enabled', False)
        self.declare_parameter('model_name', FIXED_MODEL_ID)
        self.declare_parameter('models_directory', 'models')
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('processed_image_topic', '/image_processed')
        self.declare_parameter(
            'processed_compressed_image_topic',
            '/image_processed/compressed',
        )
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('publish_detections', True)
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('confidence_threshold', 0.30)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('max_processing_fps', 10.0)
        self.declare_parameter('annotate_labels', True)
        self.declare_parameter('annotate_confidence', True)
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('jpeg_quality', 85)

        self._config_lock = threading.RLock()
        self._frame_lock = threading.RLock()
        self._subscription = None
        self._raw_publisher = None
        self._compressed_publisher = None
        self._detections_publisher = None
        self._timer = None

        self._detector: cv2.dnn_DetectionModel | None = None
        self._labels: list[str] = []
        self._models_directory = resolve_models_directory('models')
        self._last_error = ''
        self._last_status_log: tuple[str, str] | None = None
        self._active = False

        self._latest_frame: np.ndarray | None = None
        self._latest_stamp = None
        self._latest_seq = 0
        self._last_processed_seq = 0

        self._frames_received = 0
        self._frames_processed = 0

        self._load_parameters()
        self.add_on_set_parameters_callback(self._handle_parameter_update)
        self._configure_timer()
        self._reconfigure_pipeline(initial=True)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter('enabled').value)
        self.model_name = FIXED_MODEL_ID
        self.models_directory_text = str(
            self.get_parameter('models_directory').value
        ).strip() or 'models'
        self.input_topic = str(self.get_parameter('input_topic').value).strip()
        self.processed_image_topic = str(
            self.get_parameter('processed_image_topic').value
        ).strip()
        self.processed_compressed_image_topic = str(
            self.get_parameter('processed_compressed_image_topic').value
        ).strip()
        self.frame_id = str(self.get_parameter('frame_id').value).strip()
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.publish_compressed = bool(
            self.get_parameter('publish_compressed').value
        )
        self.publish_detections = bool(
            self.get_parameter('publish_detections').value
        )
        self.detections_topic = str(
            self.get_parameter('detections_topic').value
        ).strip()
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        self.max_processing_fps = float(
            self.get_parameter('max_processing_fps').value
        )
        self.annotate_labels = bool(
            self.get_parameter('annotate_labels').value
        )
        self.annotate_confidence = bool(
            self.get_parameter('annotate_confidence').value
        )
        self.line_thickness = int(self.get_parameter('line_thickness').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._validate_configuration()
        self._models_directory = resolve_models_directory(self.models_directory_text)

    def _validate_configuration(self) -> None:
        if not self.input_topic:
            raise ValueError('input_topic must not be empty')
        if not self.processed_image_topic:
            raise ValueError('processed_image_topic must not be empty')
        if not self.processed_compressed_image_topic:
            raise ValueError('processed_compressed_image_topic must not be empty')
        if not self.detections_topic:
            raise ValueError('detections_topic must not be empty')
        if not self.frame_id:
            raise ValueError('frame_id must not be empty')
        if (
            not self.publish_raw
            and not self.publish_compressed
            and not self.publish_detections
        ):
            raise ValueError(
                'At least one of publish_raw, publish_compressed or '
                'publish_detections must be true'
            )
        if not math.isfinite(self.confidence_threshold):
            raise ValueError('confidence_threshold must be finite')
        if not math.isfinite(self.nms_threshold):
            raise ValueError('nms_threshold must be finite')
        if (
            not math.isfinite(self.max_processing_fps)
            or self.max_processing_fps <= 0.0
        ):
            raise ValueError('max_processing_fps must be finite and positive')
        if not 10 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 10 and 100')
        if self.line_thickness <= 0:
            raise ValueError('line_thickness must be positive')

    def _handle_parameter_update(
        self,
        parameters: list[Parameter],
    ) -> SetParametersResult:
        candidate = {
            'enabled': self.enabled,
            'model_name': FIXED_MODEL_ID,
            'models_directory': self.models_directory_text,
            'input_topic': self.input_topic,
            'processed_image_topic': self.processed_image_topic,
            'processed_compressed_image_topic': self.processed_compressed_image_topic,
            'frame_id': self.frame_id,
            'publish_raw': self.publish_raw,
            'publish_compressed': self.publish_compressed,
            'publish_detections': self.publish_detections,
            'detections_topic': self.detections_topic,
            'confidence_threshold': self.confidence_threshold,
            'nms_threshold': self.nms_threshold,
            'max_processing_fps': self.max_processing_fps,
            'annotate_labels': self.annotate_labels,
            'annotate_confidence': self.annotate_confidence,
            'line_thickness': self.line_thickness,
            'jpeg_quality': self.jpeg_quality,
        }
        try:
            for parameter in parameters:
                if parameter.name in candidate:
                    candidate[parameter.name] = parameter.value

            self.enabled = bool(candidate['enabled'])
            if str(candidate['model_name']).strip() != FIXED_MODEL_ID:
                raise ValueError(
                    f'Only the fixed model {FIXED_MODEL_ID} is supported right now'
                )
            self.model_name = FIXED_MODEL_ID
            self.models_directory_text = str(candidate['models_directory']).strip() or 'models'
            self.input_topic = str(candidate['input_topic']).strip()
            self.processed_image_topic = str(candidate['processed_image_topic']).strip()
            self.processed_compressed_image_topic = str(
                candidate['processed_compressed_image_topic']
            ).strip()
            self.frame_id = str(candidate['frame_id']).strip()
            self.publish_raw = bool(candidate['publish_raw'])
            self.publish_compressed = bool(candidate['publish_compressed'])
            self.publish_detections = bool(candidate['publish_detections'])
            self.detections_topic = str(candidate['detections_topic']).strip()
            self.confidence_threshold = float(candidate['confidence_threshold'])
            self.nms_threshold = float(candidate['nms_threshold'])
            self.max_processing_fps = float(candidate['max_processing_fps'])
            self.annotate_labels = bool(candidate['annotate_labels'])
            self.annotate_confidence = bool(candidate['annotate_confidence'])
            self.line_thickness = int(candidate['line_thickness'])
            self.jpeg_quality = int(candidate['jpeg_quality'])
            self._validate_configuration()
            self._models_directory = resolve_models_directory(self.models_directory_text)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        self._configure_timer()
        self._reconfigure_pipeline()
        return SetParametersResult(successful=True)

    def _configure_timer(self) -> None:
        if self._timer is not None:
            self.destroy_timer(self._timer)
        self._timer = self.create_timer(
            max(1.0 / self.max_processing_fps, 0.001),
            self._process_latest_frame,
        )

    def _destroy_io(self) -> None:
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        if self._raw_publisher is not None:
            self.destroy_publisher(self._raw_publisher)
            self._raw_publisher = None
        if self._compressed_publisher is not None:
            self.destroy_publisher(self._compressed_publisher)
            self._compressed_publisher = None
        if self._detections_publisher is not None:
            self.destroy_publisher(self._detections_publisher)
            self._detections_publisher = None

    def _resolve_model_paths(self) -> tuple[str, str, str]:
        weights_path = self._models_directory / FIXED_MODEL_ID / FIXED_MODEL_WEIGHTS
        config_path = self._models_directory / FIXED_MODEL_CONFIG
        labels_path = self._models_directory / FIXED_MODEL_LABELS
        return str(weights_path), str(config_path), str(labels_path)

    def _load_detector(self) -> tuple[cv2.dnn_DetectionModel, list[str]]:
        weights_path, config_path, labels_path = self._resolve_model_paths()
        missing = [
            path
            for path in (weights_path, config_path, labels_path)
            if not Path(path).exists()
        ]
        if missing:
            missing_text = ', '.join(Path(path).name for path in missing)
            raise RuntimeError(
                f'Fixed model files are missing in {self._models_directory}: {missing_text}'
            )

        detector = cv2.dnn_DetectionModel(weights_path, config_path)
        detector.setInputSize(*FIXED_MODEL_INPUT_SIZE)
        detector.setInputScale(1.0 / 127.5)
        detector.setInputMean((127.5, 127.5, 127.5))
        detector.setInputSwapRB(True)
        return detector, load_labels_file(labels_path)

    def _log_status(self, level: str, message: str) -> None:
        status = (level, message)
        if self._last_status_log == status:
            return
        self._last_status_log = status
        logger = self.get_logger()
        if level == 'error':
            logger.error(message)
        elif level == 'warning':
            logger.warning(message)
        else:
            logger.info(message)

    def _reconfigure_pipeline(self, *, initial: bool = False) -> None:
        with self._config_lock:
            self._destroy_io()
            self._detector = None
            self._labels = []
            self._active = False
            self._last_error = ''

            if not self.enabled:
                if not initial:
                    self._log_status('info', 'Camera detector disabled')
                return

            try:
                self._detector, self._labels = self._load_detector()
            except Exception as exc:
                self._last_error = f'Could not load model {FIXED_MODEL_DISPLAY_NAME}: {exc}'
                self._log_status('error', self._last_error)
                return

            self._subscription = self.create_subscription(
                Image,
                self.input_topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
            if self.publish_raw:
                self._raw_publisher = self.create_publisher(
                    Image,
                    self.processed_image_topic,
                    qos_profile_sensor_data,
                )
            if self.publish_compressed:
                self._compressed_publisher = self.create_publisher(
                    CompressedImage,
                    self.processed_compressed_image_topic,
                    qos_profile_sensor_data,
                )
            if self.publish_detections:
                self._detections_publisher = self.create_publisher(
                    String,
                    self.detections_topic,
                    qos_profile_sensor_data,
                )
            self._active = True
            self._log_status(
                'info',
                'Camera detector enabled: '
                f'{FIXED_MODEL_DISPLAY_NAME} -> {self.processed_image_topic}, '
                f'{self.detections_topic} via opencv_dnn',
            )

    def _image_callback(self, message: Image) -> None:
        try:
            frame = image_message_to_bgr(message)
        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().warning(f'Camera detector skipped frame: {exc}')
            return

        with self._frame_lock:
            self._latest_frame = frame
            self._latest_stamp = message.header.stamp
            self._latest_seq += 1
            self._frames_received += 1

    def _should_process_now(self) -> bool:
        if not self._active or self._detector is None:
            return False
        if (
            self._raw_publisher is not None
            and self._raw_publisher.get_subscription_count() > 0
        ):
            return True
        if (
            self._compressed_publisher is not None
            and self._compressed_publisher.get_subscription_count() > 0
        ):
            return True
        if (
            self._detections_publisher is not None
            and self._detections_publisher.get_subscription_count() > 0
        ):
            return True
        return False

    def _process_latest_frame(self) -> None:
        if not self._should_process_now():
            return

        with self._frame_lock:
            if (
                self._latest_seq <= 0
                or self._latest_seq == self._last_processed_seq
                or self._latest_frame is None
            ):
                return
            frame = self._latest_frame.copy()
            stamp = self._latest_stamp
            sequence = self._latest_seq

        try:
            annotated, detections = self._run_detection(frame)
        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().warning(f'Inference failed: {exc}')
            return

        self._publish_processed_frame(annotated, stamp)
        self._publish_detections(detections, frame.shape, stamp)
        self._last_processed_seq = sequence
        self._frames_processed += 1
        self._last_error = ''

    def _run_detection(self, frame: np.ndarray) -> tuple[np.ndarray, list[Detection]]:
        assert self._detector is not None

        class_ids, confidences, boxes = self._detector.detect(
            frame,
            confThreshold=float(self.confidence_threshold),
            nmsThreshold=float(self.nms_threshold),
        )

        detections: list[Detection] = []
        if class_ids is not None and len(class_ids) > 0:
            ids = np.array(class_ids).reshape(-1).tolist()
            scores = np.array(confidences).reshape(-1).tolist()
            box_list = np.array(boxes).reshape(-1, 4).tolist()
            frame_height, frame_width = frame.shape[:2]
            for class_id, score, box in zip(ids, scores, box_list):
                x, y, width, height = [int(value) for value in box]
                x = clamp_int(x, 0, max(0, frame_width - 1))
                y = clamp_int(y, 0, max(0, frame_height - 1))
                width = clamp_int(width, 1, frame_width)
                height = clamp_int(height, 1, frame_height)
                if x + width > frame_width:
                    width = max(1, frame_width - x)
                if y + height > frame_height:
                    height = max(1, frame_height - y)
                detections.append(Detection(
                    class_id=int(class_id),
                    label=self._label_for_class(int(class_id)),
                    confidence=float(score),
                    x=x,
                    y=y,
                    width=width,
                    height=height,
                ))

        annotated = self._annotate_detections(frame, detections)
        return annotated, detections

    def _label_for_class(self, class_id: int) -> str:
        if 1 <= class_id <= len(self._labels):
            return self._labels[class_id - 1]
        return f'class_{class_id}'

    def _annotate_detections(
        self,
        frame: np.ndarray,
        detections: list[Detection],
    ) -> np.ndarray:
        annotated = frame.copy()
        for detection in detections:
            color = self._color_for_class(detection.class_id)
            x1 = detection.x
            y1 = detection.y
            x2 = detection.x + detection.width
            y2 = detection.y + detection.height
            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                color,
                int(self.line_thickness),
            )

            text_parts: list[str] = []
            if self.annotate_labels:
                text_parts.append(detection.label)
            if self.annotate_confidence:
                text_parts.append(f'{detection.confidence:.2f}')
            if text_parts:
                text = ' '.join(text_parts)
                (text_width, text_height), baseline = cv2.getTextSize(
                    text,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    1,
                )
                text_top = max(0, y1 - text_height - baseline - 6)
                cv2.rectangle(
                    annotated,
                    (x1, text_top),
                    (x1 + text_width + 8, text_top + text_height + baseline + 6),
                    color,
                    -1,
                )
                cv2.putText(
                    annotated,
                    text,
                    (x1 + 4, text_top + text_height + 1),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
        return annotated

    def _color_for_class(self, class_id: int) -> tuple[int, int, int]:
        hue = (class_id * 37) % 180
        hsv = np.uint8([[[hue, 220, 255]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])

    def _publish_detections(
        self,
        detections: list[Detection],
        frame_shape: tuple[int, ...],
        stamp: Any,
    ) -> None:
        if self._detections_publisher is None:
            return

        height = int(frame_shape[0])
        width = int(frame_shape[1])
        payload = {
            'stamp': {
                'sec': int(getattr(stamp, 'sec', 0)),
                'nanosec': int(getattr(stamp, 'nanosec', 0)),
            },
            'frame_id': self.frame_id,
            'model': {
                'id': FIXED_MODEL_ID,
                'name': FIXED_MODEL_DISPLAY_NAME,
            },
            'image': {
                'width': width,
                'height': height,
            },
            'count': len(detections),
            'detections': [
                {
                    'class_id': detection.class_id,
                    'label': detection.label,
                    'confidence': detection.confidence,
                    'bbox': {
                        'x': detection.x,
                        'y': detection.y,
                        'width': detection.width,
                        'height': detection.height,
                    },
                    'center': {
                        'x': detection.x + detection.width / 2.0,
                        'y': detection.y + detection.height / 2.0,
                    },
                }
                for detection in detections
            ],
        }

        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self._detections_publisher.publish(message)

    def _publish_processed_frame(self, frame: np.ndarray, stamp: Any) -> None:
        height, width = frame.shape[:2]
        if self._raw_publisher is not None:
            message = Image()
            message.header.stamp = stamp
            message.header.frame_id = self.frame_id
            message.height = int(height)
            message.width = int(width)
            message.encoding = 'bgr8'
            message.is_bigendian = False
            message.step = int(width * 3)
            message.data = frame.tobytes()
            self._raw_publisher.publish(message)

        if self._compressed_publisher is not None:
            data = encode_jpeg(frame, self.jpeg_quality)
            message = CompressedImage()
            message.header.stamp = stamp
            message.header.frame_id = self.frame_id
            message.format = 'jpeg'
            message.data = data
            self._compressed_publisher.publish(message)

    def close(self) -> None:
        self._destroy_io()
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: CameraDetectorNode | None = None
    try:
        node = CameraDetectorNode()
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
