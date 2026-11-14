from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import traceback
import threading
from typing import Any
from urllib.parse import urlparse

import rclpy
from rclpy.executors import MultiThreadedExecutor

from rover_agent_mcp.ros_bridge import RoverRosBridge
from rover_agent_mcp.tool_schemas import mcp_tools
from rover_agent_mcp.utils import json_dumps


class _McpHttpHandler(BaseHTTPRequestHandler):
    server_version = 'RoverMCP/0.1'

    def _send_json(self, status_code: int, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlparse(self.path)
        if parsed.path in {'/', '/health', '/mcp'}:
            self._send_json(200, {
                'ok': True,
                'name': 'rover_mcp_server',
                'message': 'Use POST /mcp with JSON-RPC methods initialize, tools/list, tools/call.',
            })
            return
        self._send_json(404, {'ok': False, 'error': 'Not found'})

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlparse(self.path)
        if parsed.path != '/mcp':
            self._send_json(404, {'jsonrpc': '2.0', 'id': None, 'error': {'code': -32004, 'message': 'Not found'}})
            return

        try:
            length = int(self.headers.get('Content-Length', '0'))
            raw_body = self.rfile.read(length).decode('utf-8')
            request = json.loads(raw_body) if raw_body else {}
        except Exception as exc:
            self._send_json(400, {'jsonrpc': '2.0', 'id': None, 'error': {'code': -32700, 'message': f'Parse error: {exc}'}})
            return

        response = self.server.handle_json_rpc(request)  # type: ignore[attr-defined]
        self._send_json(200, response)

    def log_message(self, fmt: str, *args: Any) -> None:
        node = getattr(self.server, 'ros_node', None)  # type: ignore[attr-defined]
        if node is not None:
            node.get_logger().debug(fmt % args)


class RoverMcpHttpServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], ros_node: RoverRosBridge):
        super().__init__(server_address, _McpHttpHandler)
        self.ros_node = ros_node

    def handle_json_rpc(self, request: Any) -> dict[str, Any]:
        request_id = None
        try:
            if not isinstance(request, dict):
                raise ValueError('JSON-RPC request must be an object')
            request_id = request.get('id')
            method = str(request.get('method', ''))
            params = request.get('params') or {}

            if method == 'initialize':
                result = {
                    'protocolVersion': '2024-11-05',
                    'serverInfo': {'name': 'rover_mcp_server', 'version': '0.1.0'},
                    'capabilities': {'tools': {}},
                }
            elif method == 'tools/list':
                result = {'tools': mcp_tools()}
            elif method == 'tools/call':
                if not isinstance(params, dict):
                    raise ValueError('tools/call params must be an object')
                tool_name = str(params.get('name', ''))
                arguments = params.get('arguments') or {}
                if not isinstance(arguments, dict):
                    raise ValueError('tools/call arguments must be an object')
                self.ros_node.get_logger().info(f'MCP tool call: {tool_name} {json_dumps(arguments)}')
                tool_result = self.ros_node.call_tool(tool_name, arguments)
                result = {
                    'content': [
                        {
                            'type': 'text',
                            'text': json.dumps(tool_result, ensure_ascii=False),
                        }
                    ],
                    'isError': not bool(tool_result.get('success', False)),
                    'structuredContent': tool_result,
                }
            else:
                return {
                    'jsonrpc': '2.0',
                    'id': request_id,
                    'error': {'code': -32601, 'message': f'Method not found: {method}'},
                }
            return {'jsonrpc': '2.0', 'id': request_id, 'result': result}
        except Exception as exc:
            self.ros_node.get_logger().error(f'MCP request failed: {exc}\n{traceback.format_exc()}')
            return {
                'jsonrpc': '2.0',
                'id': request_id,
                'error': {'code': -32603, 'message': str(exc)},
            }


class RoverMcpServerNode(RoverRosBridge):
    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('mcp_host', os.getenv('MCP_HOST', '127.0.0.1'))
        self.declare_parameter('mcp_port', int(os.getenv('MCP_PORT', '8765')))
        self._http_server: RoverMcpHttpServer | None = None
        self._http_thread: threading.Thread | None = None

    def start_http_server(self) -> None:
        host = str(self.get_parameter('mcp_host').value)
        port = int(self.get_parameter('mcp_port').value)
        self._http_server = RoverMcpHttpServer((host, port), self)
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()
        self.get_logger().info(f'Rover MCP JSON-RPC server listening at http://{host}:{port}/mcp')

    def stop_http_server(self) -> None:
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None


def main() -> None:
    rclpy.init()
    node = RoverMcpServerNode()
    node.start_http_server()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_http_server()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
