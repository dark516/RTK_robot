import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from traffic_sign_interfaces.msg import TrafficSignDetectionArray

from .tcp_relay import pack_frame


class DetectionTcpServer(Node):
    def __init__(self):
        super().__init__("traffic_sign_detection_tcp_server")
        self.declare_parameter("input_topic", "/traffic_sign/detections_legacy")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 18451)
        self._tcp_clients = set()
        self._lock = threading.Lock()
        self.create_subscription(
            TrafficSignDetectionArray,
            str(self.get_parameter("input_topic").value),
            self._on_message,
            10,
        )
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((str(self.get_parameter("listen_host").value), int(self.get_parameter("listen_port").value)))
        server.listen(4)
        self.get_logger().info(f"Detection TCP relay listening on {server.getsockname()}")
        while rclpy.ok():
            client, _ = server.accept()
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._tcp_clients.add(client)

    def _on_message(self, msg):
        frame = pack_frame(serialize_message(msg))
        dead = []
        with self._lock:
            for client in self._tcp_clients:
                try:
                    client.sendall(frame)
                except OSError:
                    dead.append(client)
            for client in dead:
                self._tcp_clients.remove(client)
                client.close()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionTcpServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
