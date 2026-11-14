"""Small same-origin HTTP/SSE/MJPEG server for the dashboard."""

from __future__ import annotations

from dataclasses import dataclass
import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from typing import Any, Callable
from urllib.parse import urlparse

from .calibration_service import CalibrationRequestError
from .calibration_store import ActivationRejected, RevisionConflict
from .map_catalog import MapCatalogError
from .state_store import StateStore


_CONTROL_COMMANDS = frozenset({"START", "PAUSE", "RESUME", "ABORT", "RESET"})
_ASSETS = {
    "/": "index.html",
    "/settings/calibration": "calibration.html",
    "/assets/app.css": "app.css",
    "/assets/app.js": "app.js",
    "/assets/calibration.css": "calibration.css",
    "/assets/calibration.js": "calibration.js",
}
_MAX_REQUEST_BYTES = 4096
_MAX_CALIBRATION_REQUEST_BYTES = 1024 * 1024


@dataclass(frozen=True, slots=True)
class ControlResult:
    status: int
    payload: dict[str, Any]


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, dashboard):
        self.dashboard = dashboard
        super().__init__(address, handler)


class DashboardServer:
    def __init__(
        self,
        store: StateStore,
        static_root: Path,
        control: Callable[[str, str, float], ControlResult],
        *,
        host: str = "0.0.0.0",
        port: int = 8080,
        control_timeout_s: float = 2.0,
        calibration=None,
    ) -> None:
        self.store = store
        self.static_root = Path(static_root).resolve()
        self.control = control
        self.control_timeout_s = float(control_timeout_s)
        self.calibration = calibration
        self._active = threading.Event()
        self._viewer_lock = threading.Lock()
        self._camera_viewers = 0
        self._httpd = _Server((host, int(port)), _Handler, self)
        self._thread: threading.Thread | None = None

    @property
    def url(self) -> str:
        host, port = self._httpd.server_address[:2]
        return f"http://{host}:{port}"

    @property
    def camera_viewers(self) -> int:
        with self._viewer_lock:
            return self._camera_viewers

    def _change_viewers(self, delta: int) -> None:
        with self._viewer_lock:
            self._camera_viewers = max(0, self._camera_viewers + delta)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._active.set()
        self._thread = threading.Thread(target=self.serve_forever, name="frob-web-http", daemon=True)
        self._thread.start()

    def serve_forever(self) -> None:
        self._active.set()
        self._httpd.serve_forever(poll_interval=0.1)

    def shutdown(self) -> None:
        was_active = self._active.is_set()
        self._active.clear()
        if was_active:
            try:
                self._httpd.shutdown()
            except KeyboardInterrupt:
                # ros2 launch can forward the same SIGINT that interrupted spin.
                # The HTTP worker is a daemon; cleanup must remain idempotent.
                pass
        try:
            self._httpd.server_close()
        except KeyboardInterrupt:
            pass
        if self._thread and self._thread is not threading.current_thread():
            try:
                self._thread.join(timeout=2.0)
            except KeyboardInterrupt:
                pass


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "FrobCityWeb/1.0"

    @property
    def dashboard(self) -> DashboardServer:
        return self.server.dashboard  # type: ignore[attr-defined]

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802 - stdlib handler contract
        parsed = urlparse(self.path)
        path = parsed.path
        if path in _ASSETS:
            self._serve_asset(path)
        elif path == "/api/snapshot":
            self._send_json(200, self.dashboard.store.snapshot())
        elif path == "/api/health":
            snapshot = self.dashboard.store.snapshot()
            self._send_json(200, {
                "ok": True,
                "revision": snapshot["revision"],
                "camera_viewers": self.dashboard.camera_viewers,
                "server_time_s": time.time(),
            })
        elif path == "/api/events":
            self._serve_events()
        elif path == "/api/camera.mjpg":
            self._serve_camera()
        elif path == "/api/calibration/maps":
            self._calibration_call(200, self.dashboard.calibration.maps if self.dashboard.calibration else None)
        elif path == "/api/calibration/drafts":
            self._calibration_call(200, self.dashboard.calibration.list_drafts if self.dashboard.calibration else None)
        elif path.startswith("/api/calibration/drafts/"):
            draft_id = path.removeprefix("/api/calibration/drafts/")
            callback = None if not self.dashboard.calibration else lambda: self.dashboard.calibration.get_draft(draft_id)
            self._calibration_call(200, callback)
        elif path == "/api/calibration/map-preview":
            from urllib.parse import parse_qs
            map_ids = parse_qs(parsed.query).get("id", [])
            callback = None if not self.dashboard.calibration or len(map_ids) != 1 else lambda: self.dashboard.calibration.map_preview(map_ids[0])
            self._calibration_call(200, callback)
        else:
            self._send_json(404, {"error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802 - stdlib handler contract
        path = urlparse(self.path).path
        if path == "/api/control":
            self._handle_control()
            return
        if path not in {
            "/api/calibration/drafts",
            "/api/calibration/validate",
            "/api/calibration/activate",
        } or self.dashboard.calibration is None:
            self._send_json(404, {"error": "not_found"})
            return
        body = self._read_calibration_json()
        if body is None:
            return
        if path == "/api/calibration/drafts":
            self._calibration_call(201, lambda: self.dashboard.calibration.create(body))
        elif path == "/api/calibration/validate":
            self._calibration_call(200, lambda: self.dashboard.calibration.validate(body))
        else:
            self._calibration_call(200, lambda: self.dashboard.calibration.activate(body))

    def do_PUT(self) -> None:  # noqa: N802 - stdlib handler contract
        path = urlparse(self.path).path
        prefix = "/api/calibration/drafts/"
        if not path.startswith(prefix) or self.dashboard.calibration is None:
            self._send_json(404, {"error": "not_found"})
            return
        body = self._read_calibration_json()
        if body is None:
            return
        draft_id = path.removeprefix(prefix)
        self._calibration_call(200, lambda: self.dashboard.calibration.update(draft_id, body))

    def _handle_control(self) -> None:
        if self.headers.get_content_type() != "application/json":
            self._send_json(415, {"error": "content_type_must_be_json"})
            return
        try:
            content_length = int(self.headers.get("Content-Length", "-1"))
        except ValueError:
            content_length = -1
        if content_length < 0:
            self._send_json(411, {"error": "content_length_required"})
            return
        if content_length > _MAX_REQUEST_BYTES:
            self._send_json(413, {"error": "request_too_large"})
            return
        try:
            body = json.loads(self.rfile.read(content_length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json(400, {"error": "invalid_json"})
            return
        if not isinstance(body, dict):
            self._send_json(400, {"error": "json_object_required"})
            return
        command = str(body.get("command", "")).upper()
        if command not in _CONTROL_COMMANDS:
            self._send_json(400, {"error": "invalid_command"})
            return
        reason = str(body.get("reason", "web_operator"))[:160]
        try:
            result = self.dashboard.control(command, reason, self.dashboard.control_timeout_s)
        except TimeoutError:
            result = ControlResult(504, {"accepted": False, "reason": "control_timeout"})
        except Exception:
            result = ControlResult(503, {"accepted": False, "reason": "control_unavailable"})
        self._send_json(result.status, result.payload)

    def _read_calibration_json(self) -> dict[str, Any] | None:
        if self.headers.get_content_type() != "application/json":
            self._send_calibration_error(415, "CONTENT_TYPE", "content type must be application/json")
            return None
        try:
            content_length = int(self.headers.get("Content-Length", "-1"))
        except ValueError:
            content_length = -1
        if content_length < 0:
            self._send_calibration_error(411, "CONTENT_LENGTH", "content length is required")
            return None
        if content_length > _MAX_CALIBRATION_REQUEST_BYTES:
            self._send_calibration_error(413, "REQUEST_TOO_LARGE", "request exceeds 1 MiB")
            return None
        try:
            body = json.loads(self.rfile.read(content_length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_calibration_error(400, "INVALID_JSON", "request body is not valid JSON")
            return None
        if not isinstance(body, dict):
            self._send_calibration_error(400, "JSON_OBJECT", "request body must be a JSON object")
            return None
        return body

    def _send_calibration_error(self, status: int, code: str, message: str) -> None:
        self._send_json(status, {
            "ok": False,
            "errors": [{"code": code, "path": "request", "message": message}],
        })

    def _calibration_call(self, status: int, callback) -> None:
        if callback is None:
            self._send_calibration_error(404, "NOT_FOUND", "calibration resource not found")
            return
        try:
            result = callback()
        except RevisionConflict as exc:
            self._send_calibration_error(409, "REVISION_CONFLICT", str(exc))
        except ActivationRejected as exc:
            self._send_calibration_error(422, "ACTIVATION_REJECTED", str(exc))
        except KeyError as exc:
            self._send_calibration_error(404, "NOT_FOUND", str(exc))
        except MapCatalogError as exc:
            self._send_calibration_error(422, "MAP_ERROR", str(exc))
        except CalibrationRequestError as exc:
            self._send_calibration_error(400, "BAD_REQUEST", str(exc))
        except (OSError, json.JSONDecodeError) as exc:
            self._send_calibration_error(503, "STORAGE_ERROR", str(exc))
        except (TypeError, ValueError) as exc:
            self._send_calibration_error(400, "BAD_REQUEST", str(exc))
        else:
            self._send_json(status, {"ok": True, "data": result})

    def _serve_asset(self, path: str) -> None:
        file_path = self.dashboard.static_root / _ASSETS[path]
        if not file_path.is_file() or file_path.parent != self.dashboard.static_root:
            self._send_json(404, {"error": "asset_not_found"})
            return
        data = file_path.read_bytes()
        content_type = mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        # The dashboard is served from the robot and updated in place. Always
        # revalidate assets so a browser cannot mix a new HTML page with an
        # older calibration editor after a deployment.
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self'; connect-src 'self'; style-src 'self'; script-src 'self'")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.send_response(int(status))
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def _serve_events(self) -> None:
        try:
            revision = int(self.headers.get("Last-Event-ID", "-1"))
        except ValueError:
            revision = -1
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while self.dashboard._active.is_set():
                delta = self.dashboard.store.delta_after(revision, timeout_s=5.0)
                if delta is None:
                    self.wfile.write(b": heartbeat\n\n")
                else:
                    revision = int(delta["revision"])
                    data = json.dumps(delta, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
                    self.wfile.write(f"id: {revision}\nevent: state\ndata: {data}\n\n".encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return

    def _serve_camera(self) -> None:
        boundary = "frobframe"
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        sequence = 0
        last: tuple[int, float, bytes] | None = None
        self.dashboard._change_viewers(1)
        try:
            while self.dashboard._active.is_set():
                current = self.dashboard.store.camera_after(sequence, timeout_s=0.05)
                if current is not None:
                    last = current
                    sequence = current[0]
                if last is None:
                    continue
                frame = last[2]
                header = (
                    f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\nX-Frame-Sequence: {last[0]}\r\n\r\n"
                ).encode("ascii")
                self.wfile.write(header + frame + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        finally:
            self.dashboard._change_viewers(-1)
