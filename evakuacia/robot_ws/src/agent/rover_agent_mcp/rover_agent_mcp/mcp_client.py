from __future__ import annotations

import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class McpClientError(RuntimeError):
    pass


class McpJsonRpcClient:
    def __init__(self, url: str, timeout_s: float = 30.0):
        self.url = url
        self.timeout_s = float(timeout_s)
        self._next_id = 1

    def request(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        request_id = self._next_id
        self._next_id += 1
        payload = {
            'jsonrpc': '2.0',
            'id': request_id,
            'method': method,
            'params': params or {},
        }
        encoded = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        req = Request(
            self.url,
            data=encoded,
            headers={'Content-Type': 'application/json; charset=utf-8'},
            method='POST',
        )
        try:
            with urlopen(req, timeout=self.timeout_s) as response:
                body = response.read().decode('utf-8')
        except HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            raise McpClientError(f'MCP HTTP {exc.code}: {body}') from exc
        except URLError as exc:
            raise McpClientError(f'MCP connection error: {exc}') from exc

        try:
            data = json.loads(body)
        except json.JSONDecodeError as exc:
            raise McpClientError(f'MCP returned non-JSON response: {body[:300]}') from exc

        if 'error' in data and data['error']:
            raise McpClientError(f'MCP error: {data["error"]}')
        return data.get('result', {})

    def initialize(self) -> dict[str, Any]:
        return self.request('initialize', {})

    def list_tools(self) -> list[dict[str, Any]]:
        result = self.request('tools/list', {})
        tools = result.get('tools', [])
        if not isinstance(tools, list):
            raise McpClientError('MCP tools/list returned invalid tools list')
        return tools

    def call_tool(self, name: str, arguments: dict[str, Any] | None = None) -> dict[str, Any]:
        result = self.request('tools/call', {'name': name, 'arguments': arguments or {}})
        structured = result.get('structuredContent')
        if isinstance(structured, dict):
            return structured
        content = result.get('content') or []
        if isinstance(content, list) and content:
            text = content[0].get('text', '') if isinstance(content[0], dict) else ''
            try:
                parsed = json.loads(text)
                if isinstance(parsed, dict):
                    return parsed
            except Exception:
                return {'success': not result.get('isError', False), 'text': text}
        return {'success': not result.get('isError', False), 'result': result}
