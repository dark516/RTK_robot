#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import socket
import ssl
import threading

import rclpy
from rclpy.node import Node

from rover_vision.local_llm_gateway import (
    ModelManager,
    ReusableThreadingHTTPServer,
    ensure_api_key,
    make_handler,
)
from rover_vision.vlm_diagnostic_node import ensure_certificate


class LocalLlmGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("local_llm_gateway_node")
        self.declare_parameter(
            "llama_server",
            "~/.local/bin/llama-server",
        )
        self.declare_parameter("models_dir", "~/.local/share/rtk-evacuation/models/llm")
        self.declare_parameter("log_dir", "~/.cache/rover-llm/logs")
        self.declare_parameter("api_key_file", "~/.config/sverk-rover/local_llm_api_key")
        self.declare_parameter("https_bind", "0.0.0.0")
        self.declare_parameter("https_port", 8082)
        self.declare_parameter("http_api_bind", "0.0.0.0")
        self.declare_parameter("http_api_port", 8083)
        self.declare_parameter("backend_host", "127.0.0.1")
        self.declare_parameter("backend_port", 8092)
        self.declare_parameter("device", "Vulkan0")
        self.declare_parameter("context_size", 8192)
        self.declare_parameter("threads", 8)
        self.declare_parameter("startup_timeout_s", 360.0)
        self.declare_parameter("default_model", "qwen3.5-9b")
        self.declare_parameter("autostart", True)
        self.declare_parameter("certfile", "~/.cache/rover-vlm/cert.pem")
        self.declare_parameter("keyfile", "~/.cache/rover-vlm/key.pem")

        key_path = Path(str(self.get_parameter("api_key_file").value)).expanduser()
        self.api_key = ensure_api_key(key_path)
        self.manager = ModelManager(
            llama_server=Path(str(self.get_parameter("llama_server").value)),
            models_dir=Path(str(self.get_parameter("models_dir").value)),
            api_key=self.api_key,
            log_dir=Path(str(self.get_parameter("log_dir").value)),
            api_key_file=key_path,
            backend_host=str(self.get_parameter("backend_host").value),
            backend_port=int(self.get_parameter("backend_port").value),
            device=str(self.get_parameter("device").value),
            context_size=int(self.get_parameter("context_size").value),
            threads=int(self.get_parameter("threads").value),
            startup_timeout_s=float(self.get_parameter("startup_timeout_s").value),
        )

        handler = make_handler(self.manager, self.api_key)
        https_port = int(self.get_parameter("https_port").value)
        http_api_port = int(self.get_parameter("http_api_port").value)
        self.https_server = ReusableThreadingHTTPServer(
            (str(self.get_parameter("https_bind").value), https_port),
            handler,
        )
        certfile = Path(str(self.get_parameter("certfile").value)).expanduser()
        keyfile = Path(str(self.get_parameter("keyfile").value)).expanduser()
        ensure_certificate(certfile, keyfile)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=str(certfile), keyfile=str(keyfile))
        self.https_server.socket = context.wrap_socket(self.https_server.socket, server_side=True)
        self.http_api_server = ReusableThreadingHTTPServer(
            (str(self.get_parameter("http_api_bind").value), http_api_port),
            handler,
        )
        self._server_threads = [
            threading.Thread(target=self.https_server.serve_forever, name="llm-gateway-https", daemon=True),
            threading.Thread(target=self.http_api_server.serve_forever, name="llm-gateway-http", daemon=True),
        ]
        for thread in self._server_threads:
            thread.start()

        ip = self._local_ip()
        self.get_logger().info(f"LLM dashboard and HTTPS API: https://{ip}:{https_port}/")
        self.get_logger().info(f"Robot HTTP API: http://{ip}:{http_api_port}/v1")
        self.get_logger().info(f"Shared API key file: {key_path}")

        if bool(self.get_parameter("autostart").value):
            model_id = str(self.get_parameter("default_model").value)
            threading.Thread(
                target=self._autostart,
                args=(model_id,),
                name="llm-autostart",
                daemon=True,
            ).start()

    def _autostart(self, model_id: str) -> None:
        try:
            with self.manager.request_lock:
                self.manager.ensure_model(model_id)
            self.get_logger().info(f"Default LLM ready: {model_id}")
        except Exception as error:
            self.get_logger().error(f"Unable to autostart {model_id}: {error}")

    @staticmethod
    def _local_ip() -> str:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(("192.168.1.1", 9))
            ip = sock.getsockname()[0]
            sock.close()
            return ip
        except OSError:
            return "<IP ноутбука>"

    def destroy_node(self):
        self.https_server.shutdown()
        self.http_api_server.shutdown()
        self.https_server.server_close()
        self.http_api_server.server_close()
        self.manager.stop()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalLlmGatewayNode()
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
