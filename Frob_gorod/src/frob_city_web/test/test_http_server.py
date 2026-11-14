import json
from pathlib import Path
import urllib.error
import urllib.request

import pytest

from frob_city_web.http_server import ControlResult, DashboardServer
from frob_city_web.state_store import StateStore


@pytest.fixture
def server(tmp_path: Path):
    (tmp_path / "index.html").write_text("<h1>Frob</h1>", encoding="utf-8")
    (tmp_path / "app.css").write_text("body{}", encoding="utf-8")
    (tmp_path / "app.js").write_text("'use strict';", encoding="utf-8")
    store = StateStore()
    store.update("mission", {"phase": "IDLE"})
    calls = []

    def control(command, reason, timeout_s):
        calls.append((command, reason, timeout_s))
        return ControlResult(200, {"accepted": True, "command": command})

    instance = DashboardServer(store, tmp_path, control, host="127.0.0.1", port=0)
    instance.start()
    try:
        yield instance, store, calls
    finally:
        instance.shutdown()


def get(url):
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return response.status, dict(response.headers), response.read()


def test_static_snapshot_health_and_no_cors(server):
    instance, _, _ = server
    status, headers, body = get(instance.url + "/")
    assert status == 200 and b"Frob" in body
    assert "Access-Control-Allow-Origin" not in headers
    asset_headers = get(instance.url + "/assets/app.css")[1]
    assert asset_headers["Content-Type"].startswith("text/css")
    assert asset_headers["Cache-Control"] == "no-cache"
    snapshot = json.loads(get(instance.url + "/api/snapshot")[2])
    health = json.loads(get(instance.url + "/api/health")[2])
    assert snapshot["sections"]["mission"]["phase"] == "IDLE"
    assert health["ok"] is True


def test_unknown_and_traversal_paths_are_rejected(server):
    instance, _, _ = server
    for path in ("/missing", "/assets/../index.html"):
        with pytest.raises(urllib.error.HTTPError) as error:
            get(instance.url + path)
        assert error.value.code == 404


def test_sse_returns_full_snapshot_with_event_id(server):
    instance, _, _ = server
    request = urllib.request.Request(instance.url + "/api/events", headers={"Last-Event-ID": "-1"})
    with urllib.request.urlopen(request, timeout=2.0) as response:
        lines = []
        while True:
            line = response.readline().decode("utf-8")
            if line == "\n":
                break
            lines.append(line)
    assert "event: state\n" in lines
    assert any(line.startswith("id: 1") for line in lines)
    payload = json.loads(next(line[6:] for line in lines if line.startswith("data: ")))
    assert payload["full"] is True


def test_mjpeg_streams_latest_camera_and_releases_viewer(server):
    instance, store, _ = server
    store.set_camera(b"\xff\xd8jpeg\xff\xd9", 4.0)
    with urllib.request.urlopen(instance.url + "/api/camera.mjpg", timeout=2.0) as response:
        data = response.read(100)
        assert response.headers["Content-Type"].startswith("multipart/x-mixed-replace")
        assert b"Content-Type: image/jpeg" in data
        assert b"\xff\xd8jpeg\xff\xd9" in data
    for _ in range(20):
        if instance.camera_viewers == 0:
            break
        import time
        time.sleep(0.01)
    assert instance.camera_viewers == 0


def test_shutdown_finishes_when_a_second_sigint_interrupts_http_wait(tmp_path, monkeypatch):
    (tmp_path / "index.html").write_text("ok", encoding="utf-8")
    instance = DashboardServer(
        StateStore(), tmp_path, lambda *_args: ControlResult(200, {}),
        host="127.0.0.1", port=0,
    )
    instance.start()
    original_shutdown = instance._httpd.shutdown
    original_close = instance._httpd.server_close
    closed = []

    def interrupted_shutdown():
        original_shutdown()
        raise KeyboardInterrupt

    def tracked_close():
        closed.append(True)
        original_close()

    monkeypatch.setattr(instance._httpd, "shutdown", interrupted_shutdown)
    monkeypatch.setattr(instance._httpd, "server_close", tracked_close)

    instance.shutdown()

    assert closed == [True]
    assert not instance._active.is_set()
