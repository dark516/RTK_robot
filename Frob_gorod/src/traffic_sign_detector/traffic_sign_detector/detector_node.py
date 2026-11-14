"""YOLO traffic-sign detection as a ROS 2 node."""

from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from traffic_sign_interfaces.msg import (
    TrafficSignDetection,
    TrafficSignDetectionArray,
)
from ultralytics import YOLO

from traffic_sign_detector.core import (
    DistanceEstimator,
    SignSpec,
    required_confidence,
)


SIGN_DEFAULTS = {
    "straight": SignSpec(0.100, "height"),
    "left": SignSpec(0.100, "height"),
    "right": SignSpec(0.100, "height"),
    "no_left": SignSpec(0.100, "height"),
    "no_right": SignSpec(0.100, "height"),
    "bus_stop": SignSpec(0.130, "height"),
    "parking": SignSpec(0.090, "mean"),
    "obstacle": SignSpec(0.130, "width"),
}


class TrafficSignDetectorNode(Node):
    """Detect signs, estimate their distance, and publish one array per frame."""

    def __init__(self) -> None:
        super().__init__("traffic_sign_detector")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self.bridge = CvBridge()
        self.capture: Optional[cv2.VideoCapture] = None
        self.capture_timer = None
        self.previous_frame_time = time.monotonic()
        self.failed_capture_count = 0

        self.distance_estimator = DistanceEstimator(
            focal_length_px=self.focal_length_px,
            parking_distance_scale=self.parking_distance_scale,
            filter_size=self.filter_size,
            sign_specs=self.sign_specs,
        )

        model_path = self._resolve_model_path(self.model_path)
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(str(model_path))
        self.get_logger().info(f"YOLO classes: {self.model.names}")

        self.model_min_confidence = min(
            self.default_confidence,
            self.bus_stop_confidence,
            self.parking_confidence,
        )

        self.detections_publisher = self.create_publisher(
            TrafficSignDetectionArray,
            self.detections_topic,
            10,
        )

        self.annotated_image_publisher = None
        if self.publish_annotated_image:
            self.annotated_image_publisher = self.create_publisher(
                Image,
                self.annotated_image_topic,
                qos_profile_sensor_data,
            )

        self.annotated_compressed_publisher = None
        if self.publish_annotated_compressed:
            self.annotated_compressed_publisher = self.create_publisher(
                CompressedImage,
                self.annotated_compressed_topic,
                qos_profile_sensor_data,
            )

        if self.input_source == "camera":
            self._start_camera()
        elif self.input_source == "topic":
            self.image_subscription = self.create_subscription(
                Image,
                self.image_topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"Reading ROS images from '{self.image_topic}'"
            )
        else:
            latest_frame_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.image_subscription = self.create_subscription(
                CompressedImage,
                self.compressed_image_topic,
                self._compressed_image_callback,
                latest_frame_qos,
            )
            self.get_logger().info(
                f"Reading compressed ROS images from '{self.compressed_image_topic}'"
            )

        self.get_logger().info(
            f"Publishing detections on '{self.detections_topic}' "
            f"(YOLO minimum confidence: {self.model_min_confidence:.2f})"
        )

    def _declare_parameters(self) -> None:
        # Original script settings.
        self.declare_parameter("model_path", "models/best.pt")
        self.declare_parameter("camera_index", 48)
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("default_confidence", 0.80)
        self.declare_parameter("bus_stop_confidence", 0.40)
        self.declare_parameter("parking_confidence", 0.60)
        self.declare_parameter("focal_length_px", 900.0)
        self.declare_parameter("parking_distance_scale", 1.54)
        self.declare_parameter("filter_size", 5)

        for class_name, spec in SIGN_DEFAULTS.items():
            self.declare_parameter(
                f"signs.{class_name}.dimension_m",
                spec.dimension_m,
            )
            self.declare_parameter(
                f"signs.{class_name}.axis",
                spec.axis,
            )

        # ROS/video integration settings.
        self.declare_parameter("input_source", "camera")
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("camera_frame_id", "camera")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter(
            "compressed_image_topic",
            "/camera/image_raw/compressed",
        )
        self.declare_parameter("detections_topic", "/traffic_sign/detections")
        self.declare_parameter(
            "annotated_image_topic",
            "/traffic_sign/annotated_image",
        )
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter(
            "annotated_compressed_topic",
            "/traffic_sign/annotated_image/compressed",
        )
        self.declare_parameter("publish_annotated_compressed", False)
        self.declare_parameter("annotated_jpeg_quality", 70)
        self.declare_parameter("display_window", True)
        self.declare_parameter("draw_debug_info", True)
        self.declare_parameter("window_name", "YOLO Traffic Sign Detection")
        self.declare_parameter("yolo_device", "")
        self.declare_parameter("yolo_image_size", 0)

    def _read_parameters(self) -> None:
        self.model_path = self._param("model_path")
        self.camera_index = self._param("camera_index")
        self.frame_width = self._param("frame_width")
        self.frame_height = self._param("frame_height")
        self.default_confidence = self._param("default_confidence")
        self.bus_stop_confidence = self._param("bus_stop_confidence")
        self.parking_confidence = self._param("parking_confidence")
        self.focal_length_px = self._param("focal_length_px")
        self.parking_distance_scale = self._param("parking_distance_scale")
        self.filter_size = self._param("filter_size")

        self.sign_specs: Dict[str, SignSpec] = {}
        for class_name in SIGN_DEFAULTS:
            self.sign_specs[class_name] = SignSpec(
                dimension_m=float(
                    self._param(f"signs.{class_name}.dimension_m")
                ),
                axis=str(self._param(f"signs.{class_name}.axis")),
            )

        self.input_source = str(self._param("input_source")).lower()
        self.camera_fps = self._param("camera_fps")
        self.camera_frame_id = self._param("camera_frame_id")
        self.image_topic = self._param("image_topic")
        self.compressed_image_topic = self._param("compressed_image_topic")
        self.detections_topic = self._param("detections_topic")
        self.annotated_image_topic = self._param("annotated_image_topic")
        self.publish_annotated_image = self._param("publish_annotated_image")
        self.annotated_compressed_topic = self._param("annotated_compressed_topic")
        self.publish_annotated_compressed = self._param(
            "publish_annotated_compressed"
        )
        self.annotated_jpeg_quality = self._param("annotated_jpeg_quality")
        self.display_window = self._param("display_window")
        self.draw_debug_info = self._param("draw_debug_info")
        self.window_name = self._param("window_name")
        self.yolo_device = str(self._param("yolo_device"))
        self.yolo_image_size = self._param("yolo_image_size")

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _validate_parameters(self) -> None:
        if self.input_source not in {"camera", "topic", "compressed_topic"}:
            raise ValueError(
                "input_source must be 'camera', 'topic', or 'compressed_topic'"
            )
        if self.frame_width < 1 or self.frame_height < 1:
            raise ValueError("frame_width and frame_height must be positive")
        if self.camera_fps <= 0.0:
            raise ValueError("camera_fps must be greater than zero")
        if self.yolo_image_size < 0:
            raise ValueError("yolo_image_size cannot be negative")
        if not 1 <= self.annotated_jpeg_quality <= 100:
            raise ValueError("annotated_jpeg_quality must be between 1 and 100")

        for name, value in (
            ("default_confidence", self.default_confidence),
            ("bus_stop_confidence", self.bus_stop_confidence),
            ("parking_confidence", self.parking_confidence),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be between 0.0 and 1.0")

    def _resolve_model_path(self, configured_path: str) -> Path:
        expanded_path = Path(
            os.path.expandvars(os.path.expanduser(configured_path))
        )
        candidates = []

        if expanded_path.is_absolute():
            candidates.append(expanded_path)
        else:
            candidates.append(Path.cwd() / expanded_path)
            try:
                package_share = Path(
                    get_package_share_directory("traffic_sign_detector")
                )
                candidates.append(package_share / expanded_path)
            except PackageNotFoundError:
                pass

        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()

        searched = ", ".join(str(path) for path in candidates)
        raise FileNotFoundError(
            f"YOLO model '{configured_path}' was not found. Searched: {searched}"
        )

    def _start_camera(self) -> None:
        self.capture = cv2.VideoCapture(int(self.camera_index))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.frame_width))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.frame_height))

        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open camera {self.camera_index}")

        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"Camera {self.camera_index} opened at "
            f"{actual_width}x{actual_height}"
        )

        self.capture_timer = self.create_timer(
            1.0 / float(self.camera_fps),
            self._capture_frame,
        )

    def _capture_frame(self) -> None:
        if self.capture is None:
            return

        success, frame = self.capture.read()
        if not success:
            self.failed_capture_count += 1
            if self.failed_capture_count == 1 or self.failed_capture_count % 30 == 0:
                self.get_logger().error(
                    f"Failed to receive camera frame "
                    f"({self.failed_capture_count} consecutive failures)"
                )
            return

        self.failed_capture_count = 0
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.camera_frame_id)
        self._process_frame(frame, header)

    def _image_callback(self, image_message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding="bgr8",
            )
        except Exception as exc:  # cv_bridge exception types differ by ROS release.
            self.get_logger().error(f"Could not convert input image: {exc}")
            return

        self._process_frame(frame, image_message.header)

    def _compressed_image_callback(self, image_message: CompressedImage) -> None:
        try:
            encoded = np.frombuffer(image_message.data, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("OpenCV returned an empty image")
        except Exception as exc:
            self.get_logger().error(f"Could not decode compressed input image: {exc}")
            return

        # Keep the robot capture timestamp.  The map planner uses it to select
        # the robot pose that existed before Wi-Fi transfer and laptop inference.
        self._process_frame(frame, image_message.header)

    def _process_frame(self, frame, header: Header) -> None:
        predict_arguments = {
            "source": frame,
            "conf": float(self.model_min_confidence),
            "verbose": False,
        }
        if self.yolo_device:
            predict_arguments["device"] = self.yolo_device
        if self.yolo_image_size > 0:
            predict_arguments["imgsz"] = int(self.yolo_image_size)

        try:
            result = self.model.predict(**predict_arguments)[0]
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        output_message = TrafficSignDetectionArray()
        output_message.header = header

        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = str(self.model.names[class_id])
            confidence = float(box.conf[0])

            threshold = required_confidence(
                class_name,
                self.default_confidence,
                self.bus_stop_confidence,
                self.parking_confidence,
            )
            if confidence < threshold:
                continue

            x_min, y_min, x_max, y_max = map(int, box.xyxy[0].tolist())
            box_width = x_max - x_min
            box_height = y_max - y_min
            center_x = int((x_min + x_max) / 2)
            center_y = int((y_min + y_max) / 2)

            distance_cm = self.distance_estimator.estimate_filtered_cm(
                class_name,
                box_width,
                box_height,
            )

            detection_message = TrafficSignDetection()
            detection_message.class_id = class_id
            detection_message.class_name = class_name
            detection_message.confidence = confidence
            detection_message.has_distance = distance_cm is not None
            detection_message.distance_cm = (
                float(distance_cm) if distance_cm is not None else math.nan
            )
            detection_message.x_min = x_min
            detection_message.y_min = y_min
            detection_message.x_max = x_max
            detection_message.y_max = y_max
            detection_message.center_x = center_x
            detection_message.center_y = center_y
            output_message.detections.append(detection_message)

            self._draw_detection(
                frame=frame,
                class_name=class_name,
                confidence=confidence,
                distance_cm=distance_cm,
                threshold=threshold,
                box=(x_min, y_min, x_max, y_max),
                center=(center_x, center_y),
            )

        self.detections_publisher.publish(output_message)

        current_time = time.monotonic()
        elapsed = current_time - self.previous_frame_time
        self.previous_frame_time = current_time
        fps = 1.0 / elapsed if elapsed > 0.0 else 0.0
        self._draw_status(frame, fps)

        if self.annotated_image_publisher is not None:
            annotated_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated_message.header = header
            self.annotated_image_publisher.publish(annotated_message)

        if self.annotated_compressed_publisher is not None:
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.annotated_jpeg_quality)],
            )
            if ok:
                annotated_compressed = CompressedImage()
                annotated_compressed.header = header
                annotated_compressed.format = "jpeg"
                annotated_compressed.data = encoded.tobytes()
                self.annotated_compressed_publisher.publish(annotated_compressed)
            else:
                self.get_logger().warning("Could not encode annotated JPEG")

        if self.display_window:
            cv2.imshow(str(self.window_name), frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("Q pressed; stopping detector")
                rclpy.shutdown()

    @staticmethod
    def _box_color(class_name: str) -> Tuple[int, int, int]:
        if class_name == "bus_stop":
            return (0, 165, 255)
        if class_name == "parking":
            return (255, 100, 0)
        return (0, 255, 0)

    def _draw_detection(
        self,
        frame,
        class_name: str,
        confidence: float,
        distance_cm: Optional[float],
        threshold: float,
        box: Tuple[int, int, int, int],
        center: Tuple[int, int],
    ) -> None:
        x_min, y_min, x_max, y_max = box
        box_width = x_max - x_min
        box_height = y_max - y_min
        color = self._box_color(class_name)

        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)

        label = f"{class_name} {confidence:.2f}"
        if distance_cm is not None:
            label += f" {distance_cm:.1f} cm"
        cv2.putText(
            frame,
            label,
            (x_min, max(y_min - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
        )

        if not self.draw_debug_info:
            return

        debug_text = f"{box_width}x{box_height}px thr:{threshold:.2f}"
        if class_name == "parking":
            pixel_mean = (box_width + box_height) / 2.0
            debug_text = (
                f"{box_width}x{box_height}px avg:{pixel_mean:.1f}px "
                f"scale:{self.parking_distance_scale:.2f}x "
                f"thr:{threshold:.2f}"
            )
        cv2.putText(
            frame,
            debug_text,
            (x_min, y_max + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
        )

    def _draw_status(self, frame, fps: float) -> None:
        lines = (
            (f"FPS: {fps:.1f}", (255, 255, 255), 0.8, 35),
            (
                f"Default conf: {self.default_confidence:.2f}",
                (255, 255, 255),
                0.55,
                65,
            ),
            (
                f"Bus stop conf: {self.bus_stop_confidence:.2f}",
                (0, 165, 255),
                0.55,
                90,
            ),
            (
                f"Parking conf: {self.parking_confidence:.2f}",
                (255, 100, 0),
                0.55,
                115,
            ),
            (
                f"Parking scale: {self.parking_distance_scale:.2f}x",
                (255, 100, 0),
                0.55,
                140,
            ),
        )

        for text, color, scale, y_position in lines:
            cv2.putText(
                frame,
                text,
                (20, y_position),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale,
                color,
                2 if y_position == 35 else 1,
            )

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        if self.display_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrafficSignDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"traffic_sign_detector failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
