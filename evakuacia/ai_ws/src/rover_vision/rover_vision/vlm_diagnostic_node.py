#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
import socket
import ssl
import subprocess
import threading
from http.server import ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rover_vision.vlm_diagnostic import (
    EVENT_TYPES,
    DashboardHandler,
    VlmDiagnosticEngine,
)


def ensure_certificate(certfile: Path, keyfile: Path):
    certfile.parent.mkdir(parents=True, exist_ok=True)
    if certfile.exists() and keyfile.exists():
        return
    certfile.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
            "-keyout", str(keyfile), "-out", str(certfile), "-days", "30",
            "-subj", "/CN=rover-vlm.local",
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


class VlmDiagnosticNode(Node):
    def __init__(self):
        super().__init__("vlm_diagnostic_node")
        self.declare_parameter("robot_url", "http://192.168.1.70:8091/stream?mode=live")
        self.declare_parameter("vlm_server_url", "http://127.0.0.1:8091/v1/chat/completions")
        self.declare_parameter("bind", "0.0.0.0")
        self.declare_parameter("port", 8081)
        self.declare_parameter("mode", "service")
        self.declare_parameter("capture_interval_sec", 0.5)
        self.declare_parameter("camera_timeout_sec", 2.0)
        self.declare_parameter("certfile", "~/.cache/rover-vlm/cert.pem")
        self.declare_parameter("keyfile", "~/.cache/rover-vlm/key.pem")
        self.declare_parameter("event_types", EVENT_TYPES)

        self.engine = VlmDiagnosticEngine(
            robot_url=self.get_parameter("robot_url").value,
            vlm_server_url=self.get_parameter("vlm_server_url").value,
            mode=self.get_parameter("mode").value,
            capture_interval=float(self.get_parameter("capture_interval_sec").value),
            camera_timeout=float(self.get_parameter("camera_timeout_sec").value),
            event_types=self.get_parameter("event_types").value,
        )
        self.service = self.create_service(Trigger, "/vlm/analyze_frame", self._analyze_service)
        self.server = self._start_server()
        self.get_logger().info(
            f"VLM dashboard: https://{self._local_ip()}:{self.server.server_port}/"
        )
        self.get_logger().info("ROS service: /vlm/analyze_frame")

    def _analyze_service(self, _request, response):
        response.success, response.message = self.engine.request_analysis_sync(timeout=125.0)
        return response

    def _start_server(self):
        bind = str(self.get_parameter("bind").value)
        port = int(self.get_parameter("port").value)
        server = ThreadingHTTPServer((bind, port), DashboardHandler(self.engine))
        certfile = Path(str(self.get_parameter("certfile").value)).expanduser()
        keyfile = Path(str(self.get_parameter("keyfile").value)).expanduser()
        ensure_certificate(certfile, keyfile)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=str(certfile), keyfile=str(keyfile))
        server.socket = context.wrap_socket(server.socket, server_side=True)
        threading.Thread(target=server.serve_forever, name="vlm-https", daemon=True).start()
        return server

    @staticmethod
    def _local_ip():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(("192.168.1.1", 9))
            ip = sock.getsockname()[0]
            sock.close()
            return ip
        except OSError:
            return "<IP ноутбука>"

    def destroy_node(self):
        self.engine.stop()
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VlmDiagnosticNode()
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
