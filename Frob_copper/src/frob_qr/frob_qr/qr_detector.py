import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
import cv2
import numpy as np


class QRDetector(Node):
    """Reads camera directly via OpenCV, detects QR codes and publishes offset + data."""

    def __init__(self):
        super().__init__('qr_detector')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('show_window', False)
        self.declare_parameter('verbose', False)
        self.declare_parameter('queue_size', 10)

        camera_id = self.get_parameter('camera_id').value
        show_window = self.get_parameter('show_window').value
        self._verbose = self.get_parameter('verbose').value
        queue = self.get_parameter('queue_size').value

        self._window_name = 'QR Detector'

        # Open camera directly
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera with ID {camera_id}')
            raise RuntimeError(f'Cannot open camera {camera_id}')

        self._log(f'Camera opened with ID {camera_id}')

        # QR code detector
        self.qr_detector = cv2.QRCodeDetector()

        # Publishers
        self.error_pub = self.create_publisher(Float32, '/qr/error', queue)
        self.detected_pub = self.create_publisher(Bool, '/qr/detected', queue)
        self.data_pub = self.create_publisher(String, '/qr/data', queue)

        # Show debug window
        if show_window:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window_name, 640, 480)

        # Timer: grab and process frames at ~15 FPS
        self.timer = self.create_timer(1.0 / 15.0, self._process_frame)

        self.get_logger().info(
            f'QRDetector started — looking for QR codes '
            f'on camera {camera_id} | show_window={show_window} verbose={self._verbose}'
        )

    def _log(self, msg):
        if self._verbose:
            self.get_logger().info(msg)

    def _warn(self, msg):
        if self._verbose:
            self.get_logger().warn(msg)

    def _process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self._warn('Failed to grab frame from camera')
            return

        height, width = frame.shape[:2]
        frame_center_x = width / 2.0

        # Detect QR code
        data, points, _ = self.qr_detector.detectAndDecode(frame)

        detected = False
        error = 0.0
        decoded_data = ""

        if data and points is not None:
            detected = True
            decoded_data = data

            # points shape: (4, 2) — four corners of the QR code
            # Compute center of the bounding quadrilateral
            pts = points[0]
            center_x = float(np.mean(pts[:, 0]))

            # Normalized error: -1 (far left) to +1 (far right)
            error = (center_x - frame_center_x) / (width / 2.0)

            self._log(
                f'QR detected — data="{data}" '
                f'center_x={center_x:.1f}, error={error:.3f}'
            )

        self.error_pub.publish(Float32(data=error))
        self.detected_pub.publish(Bool(data=detected))
        self.data_pub.publish(String(data=decoded_data))

        # --- Window visualization ---
        show = self.get_parameter('show_window').value
        if not show:
            return

        display = frame.copy()

        # Vertical center line
        cv2.line(display, (width // 2, 0), (width // 2, height), (0, 255, 0), 1)

        if detected and points is not None:
            # Draw QR code bounding box
            pts = points[0].astype(np.int32)
            cv2.polylines(display, [pts], True, (0, 255, 0), 2)

            # Center crosshair
            center_x = int(np.mean(pts[:, 0]))
            center_y = int(np.mean(pts[:, 1]))
            cv2.line(display, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 2)
            cv2.line(display, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 2)

            # Line from QR center to frame center
            cv2.line(display, (center_x, center_y), (width // 2, height // 2), (255, 0, 0), 2)

            # Error value
            color = (0, 255, 0) if abs(error) < 0.1 else (0, 255, 255) if abs(error) < 0.3 else (0, 0, 255)
            cv2.putText(display, f'Error: {error:+.3f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(display, f'Data: {decoded_data[:20]}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        else:
            cv2.putText(display, 'QR NOT found', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow(self._window_name, display)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
