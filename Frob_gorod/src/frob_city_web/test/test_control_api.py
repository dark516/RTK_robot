import json
from pathlib import Path
import urllib.error
import urllib.request

import pytest

from frob_city_web.http_server import ControlResult, DashboardServer
from frob_city_web.state_store import StateStore


@pytest.fixture
def endpoint(tmp_path: Path):
    for name in ("index.html", "app.css", "app.js"):
        (tmp_path / name).write_text(name, encoding="utf-8")
    calls = []

    def callback(command, reason, timeout_s):
        calls.append((command, reason, timeout_s))
        if command == "PAUSE":
            return ControlResult(409, {"accepted": False, "reason": "wrong phase"})
        return ControlResult(200, {"accepted": True})

    server = DashboardServer(StateStore(), tmp_path, callback, host="127.0.0.1", port=0)
    server.start()
    try:
        yield server.url, calls
    finally:
        server.shutdown()


def post(url, payload, content_type="application/json"):
    body = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
    request = urllib.request.Request(
        url + "/api/control", data=body, method="POST",
        headers={"Content-Type": content_type},
    )
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def test_only_five_commands_reach_callback(endpoint):
    url, calls = endpoint
    for command in ("START", "PAUSE", "RESUME", "ABORT", "RESET"):
        status, _ = post(url, {"command": command, "reason": "operator"})
        assert status in {200, 409}
    assert [item[0] for item in calls] == ["START", "PAUSE", "RESUME", "ABORT", "RESET"]

    status, body = post(url, {"command": "FORWARD"})
    assert status == 400
    assert body["error"] == "invalid_command"
    assert len(calls) == 5


def test_invalid_content_and_oversized_body_are_rejected(endpoint):
    url, calls = endpoint
    assert post(url, b"not json")[0] == 400
    assert post(url, {"command": "START"}, "text/plain")[0] == 415
    assert post(url, b"x" * 4097)[0] == 413
    assert calls == []


def test_service_rejection_is_preserved(endpoint):
    status, body = post(endpoint[0], {"command": "PAUSE"})
    assert status == 409
    assert body == {"accepted": False, "reason": "wrong phase"}
