import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from traffic_sign_interfaces.msg import TrafficSignDetectionArray

from .tcp_relay import FrameDecoder


class DetectionTcpReceiver(Node):
    def __init__(self):
        super().__init__("traffic_sign_detection_tcp_receiver")
        self.declare_parameter("server_host", "10.79.241.111")
        self.declare_parameter("server_port", 18451)
        self.declare_parameter("output_topic", "/traffic_sign/detections_legacy")
        self._publisher = self.create_publisher(
            TrafficSignDetectionArray,
            str(self.get_parameter("output_topic").value),
            10,
        )
        threading.Thread(target=self._receive_loop, daemon=True).start()

    def _receive_loop(self):
        host = str(self.get_parameter("server_host").value)
        port = int(self.get_parameter("server_port").value)
        while rclpy.ok():
            try:
                with socket.create_connection((host, port), timeout=3.0) as connection:
                    connection.settimeout(None)
                    decoder = FrameDecoder()
                    self.get_logger().info(f"Connected to detection relay at {host}:{port}")
                    while rclpy.ok():
                        data = connection.recv(65536)
                        if not data:
                            raise ConnectionError("relay closed connection")
                        for payload in decoder.feed(data):
                            self._publisher.publish(
                                deserialize_message(payload, TrafficSignDetectionArray)
                            )
            except (OSError, ConnectionError, ValueError) as exc:
                self.get_logger().warning(f"Detection relay unavailable: {exc}")
                time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionTcpReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
