"""Fail-closed adapter from the legacy detector message to the city contract."""
from __future__ import annotations

import math
import uuid

import rclpy
from rclpy.node import Node

from frob_city_interfaces.msg import SignDetection, SignDetectionArray
from traffic_sign_interfaces.msg import TrafficSignDetectionArray


_CLASS_BY_NAME = {
    "straight": SignDetection.STRAIGHT_ONLY,
    "straight_only": SignDetection.STRAIGHT_ONLY,
    "left": SignDetection.LEFT_ONLY,
    "left_only": SignDetection.LEFT_ONLY,
    "right": SignDetection.RIGHT_ONLY,
    "right_only": SignDetection.RIGHT_ONLY,
    "no_left": SignDetection.NO_LEFT,
    "no_right": SignDetection.NO_RIGHT,
    "bus_stop": SignDetection.BUS_STOP,
    "parking": SignDetection.PARKING,
    "hazard": SignDetection.HAZARD_MARKER,
    "hazard_marker": SignDetection.HAZARD_MARKER,
    "obstacle": SignDetection.HAZARD_MARKER,
}


def _normalized_label(value: str) -> str:
    return str(value).strip().lower().replace("-", "_").replace(" ", "_")


def adapt_legacy_frame(
    legacy: TrafficSignDetectionArray,
    *,
    detector_instance_id: str,
    frame_sequence: int,
    image_width_px: int,
    image_height_px: int,
    calibration_id: str,
    output_frame_id: str = "",
    allow_qualified_empty_frames: bool = False,
    trust_legacy_range: bool = False,
    range_stddev_m: float = 0.10,
) -> SignDetectionArray:
    """Convert one frame while preserving conservative negative evidence.

    Legacy messages have no frame-quality flag.  By default an empty legacy
    frame is therefore *not* qualified absence, while accepted positive
    detections remain usable.  Range is also disabled until its uncertainty
    has been measured and explicitly commissioned.
    """
    if image_width_px <= 0 or image_height_px <= 0:
        raise ValueError("image dimensions must be positive")
    if trust_legacy_range and (not math.isfinite(range_stddev_m) or range_stddev_m <= 0.0):
        raise ValueError("trusted legacy range requires positive finite uncertainty")

    out = SignDetectionArray()
    out.header = legacy.header
    if output_frame_id:
        out.header.frame_id = str(output_frame_id)
    out.detector_instance_id = str(detector_instance_id)
    out.frame_sequence = int(frame_sequence)
    out.image_width_px = int(image_width_px)
    out.image_height_px = int(image_height_px)
    out.calibration_id = str(calibration_id)

    rejected = 0
    for index, item in enumerate(legacy.detections):
        sign_class = _CLASS_BY_NAME.get(_normalized_label(item.class_name))
        confidence = float(item.confidence)
        if sign_class is None or not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            rejected += 1
            continue
        x0 = max(0, min(int(item.x_min), image_width_px))
        y0 = max(0, min(int(item.y_min), image_height_px))
        x1 = max(0, min(int(item.x_max), image_width_px))
        y1 = max(0, min(int(item.y_max), image_height_px))
        if x1 <= x0 or y1 <= y0:
            rejected += 1
            continue

        converted = SignDetection()
        converted.sign_class = int(sign_class)
        converted.confidence = confidence
        converted.x_min_px = x0
        converted.y_min_px = y0
        converted.width_px = x1 - x0
        converted.height_px = y1 - y0
        distance_cm = float(item.distance_cm)
        range_ok = (
            trust_legacy_range
            and bool(item.has_distance)
            and math.isfinite(distance_cm)
            and distance_cm > 0.0
        )
        converted.range_valid = range_ok
        converted.range_convention = (
            SignDetection.OPTICAL_Z if range_ok else SignDetection.RANGE_UNKNOWN
        )
        converted.range_m = distance_cm * 0.01 if range_ok else 0.0
        converted.range_stddev_m = float(range_stddev_m) if range_ok else 0.0
        converted.detector_track_id = f"legacy:{frame_sequence}:{index}"
        out.detections.append(converted)

    # A positive accepted frame is useful evidence.  Empty legacy frames only
    # become negative evidence after an explicit camera/detector qualification.
    out.frame_valid = bool(out.detections) or bool(allow_qualified_empty_frames)
    if out.frame_valid:
        out.invalid_reason = ""
    elif rejected:
        out.invalid_reason = f"legacy_frame_has_no_accepted_detections:{rejected}_rejected"
    else:
        out.invalid_reason = "legacy_empty_frame_not_qualified"
    return out


class DetectionAdapterNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("detection_adapter", parameter_overrides=parameter_overrides)
        defaults = {
            "input_topic": "/traffic_sign/detections_legacy",
            "output_topic": "/traffic_sign/detections",
            "image_width_px": 1280,
            "image_height_px": 720,
            "output_frame_id": "",
            "calibration_id": "LEGACY_UNCOMMISSIONED",
            "allow_qualified_empty_frames": False,
            "trust_legacy_range": False,
            "range_stddev_m": 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._sequence = 0
        self._detector_instance_id = f"legacy-adapter-{uuid.uuid4().hex[:12]}"
        self._publisher = self.create_publisher(
            SignDetectionArray, str(self.get_parameter("output_topic").value), 10
        )
        self._subscription = self.create_subscription(
            TrafficSignDetectionArray,
            str(self.get_parameter("input_topic").value),
            self._on_frame,
            10,
        )
        self.get_logger().info(
            "Legacy detector adapter active; empty frames and range remain "
            "unqualified unless explicitly commissioned."
        )

    def _on_frame(self, msg: TrafficSignDetectionArray) -> None:
        self._sequence += 1
        try:
            converted = adapt_legacy_frame(
                msg,
                detector_instance_id=self._detector_instance_id,
                frame_sequence=self._sequence,
                image_width_px=int(self.get_parameter("image_width_px").value),
                image_height_px=int(self.get_parameter("image_height_px").value),
                calibration_id=str(self.get_parameter("calibration_id").value),
                output_frame_id=str(self.get_parameter("output_frame_id").value),
                allow_qualified_empty_frames=bool(
                    self.get_parameter("allow_qualified_empty_frames").value
                ),
                trust_legacy_range=bool(self.get_parameter("trust_legacy_range").value),
                range_stddev_m=float(self.get_parameter("range_stddev_m").value),
            )
        except ValueError as exc:
            self.get_logger().error(f"Legacy detection frame rejected: {exc}")
            return
        self._publisher.publish(converted)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
