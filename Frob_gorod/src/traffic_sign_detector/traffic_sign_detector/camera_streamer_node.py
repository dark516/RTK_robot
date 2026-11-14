"""Capture the robot camera and publish bandwidth-friendly JPEG frames."""

from __future__ import annotations

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CameraStreamerNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_sign_camera_streamer")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("camera_fps", 5.0)
        self.declare_parameter("camera_frame_id", "camera")
        self.declare_parameter("jpeg_quality", 72)
        self.declare_parameter(
            "compressed_image_topic", "/camera/image_raw/compressed"
        )

        camera_index = int(self.get_parameter("camera_index").value)
        width = int(self.get_parameter("frame_width").value)
        height = int(self.get_parameter("frame_height").value)
        fps = float(self.get_parameter("camera_fps").value)
        self._frame_id = str(self.get_parameter("camera_frame_id").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        topic = str(self.get_parameter("compressed_image_topic").value)
        if width < 1 or height < 1 or fps <= 0.0:
            raise ValueError("frame dimensions and camera_fps must be positive")
        if not 1 <= self._jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")

        self._publisher = self.create_publisher(
            CompressedImage, topic, qos_profile_sensor_data
        )
        self._capture = cv2.VideoCapture(camera_index)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # A one-frame backend buffer minimizes stale frames when encoding is late.
        self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self._capture.isOpened():
            raise RuntimeError(f"Could not open camera {camera_index}")

        actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._failures = 0
        self._timer = self.create_timer(1.0 / fps, self._capture_frame)
        self.get_logger().info(
            f"Streaming camera {camera_index} at {actual_width}x{actual_height}, "
            f"up to {fps:.1f} FPS, JPEG quality {self._jpeg_quality}, topic '{topic}'"
        )

    def _capture_frame(self) -> None:
        success, frame = self._capture.read()
        if not success:
            self._failures += 1
            if self._failures == 1 or self._failures % 30 == 0:
                self.get_logger().error(
                    f"Failed to receive camera frame ({self._failures} failures)"
                )
            return
        self._failures = 0

        # Stamp immediately after capture and before JPEG work.  This is the
        # timestamp that returns unchanged with the laptop detections.
        stamp = self.get_clock().now().to_msg()
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not ok:
            self.get_logger().error("OpenCV JPEG encoding failed")
            return
        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self._publisher.publish(message)

    def destroy_node(self) -> bool:
        self._capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraStreamerNode()
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
