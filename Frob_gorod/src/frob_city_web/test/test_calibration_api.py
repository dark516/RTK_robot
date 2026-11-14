import json
import http.client
from pathlib import Path
import urllib.error
import urllib.request

import pytest

from frob_city_web.calibration_service import CalibrationService
from frob_city_web.calibration_store import CalibrationStore
from frob_city_web.http_server import ControlResult, DashboardServer
from frob_city_web.map_catalog import MapCatalog
from frob_city_web.state_store import StateStore


def write_map(root: Path):
    root.mkdir(parents=True)
    (root / "field.pgm").write_text("P2\n2 2\n255\n254 254 254 254\n", encoding="ascii")
    (root / "field.yaml").write_text(
        "\n".join((
            "image: field.pgm",
            "resolution: 2.5",
            "origin: [0.0, 0.0, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
        )),
        encoding="utf-8",
    )


@pytest.fixture
def calibration_server(tmp_path: Path):
    static = tmp_path / "static"
    static.mkdir()
    for name in ("index.html", "app.css", "app.js", "calibration.html", "calibration.css", "calibration.js"):
        (static / name).write_text(name, encoding="utf-8")
    maps = tmp_path / "maps"
    write_map(maps)
    service = CalibrationService(
        MapCatalog([maps]),
        CalibrationStore(tmp_path / "calibration"),
        live_map_snapshot=lambda: None,
        live_map_max_age_s=2.0,
    )
    server = DashboardServer(
        StateStore(), static, lambda *_args: ControlResult(200, {}),
        host="127.0.0.1", port=0, calibration=service,
    )
    server.start()
    try:
        yield server.url
    finally:
        server.shutdown()


def request_json(url, *, method="GET", payload=None, content_type="application/json"):
    data = None if payload is None else (
        payload if isinstance(payload, bytes) else json.dumps(payload).encode("utf-8")
    )
    request = urllib.request.Request(
        url, data=data, method=method,
        headers={"Content-Type": content_type} if data is not None else {},
    )
    try:
        with urllib.request.urlopen(request, timeout=3.0) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def test_calibration_page_assets_are_served(calibration_server):
    with urllib.request.urlopen(calibration_server + "/settings/calibration") as response:
        assert response.status == 200
        assert response.read() == b"calibration.html"
    with urllib.request.urlopen(calibration_server + "/assets/calibration.js") as response:
        assert response.status == 200


def test_map_list_reports_file_fallback_when_live_map_missing(calibration_server):
    status, payload = request_json(calibration_server + "/api/calibration/maps")

    assert status == 200
    assert payload["ok"] is True
    assert payload["data"]["live"]["available"] is False
    assert payload["data"]["files"][0]["name"] == "field"


def test_file_map_preview_is_returned_by_opaque_id(calibration_server):
    maps = request_json(calibration_server + "/api/calibration/maps")[1]["data"]["files"]

    status, payload = request_json(
        calibration_server + f"/api/calibration/map-preview?id={maps[0]['id']}"
    )

    assert status == 200
    assert payload["data"]["width"] == 2
    assert payload["data"]["source"]["kind"] == "file"


def test_create_load_update_and_revision_conflict(calibration_server):
    maps = request_json(calibration_server + "/api/calibration/maps")[1]["data"]["files"]
    status, created = request_json(
        calibration_server + "/api/calibration/drafts",
        method="POST",
        payload={"name": "Поле", "map_source": {"kind": "file", "id": maps[0]["id"]}},
    )
    document = created["data"]
    assert status == 201
    assert document["field"]["grid_size"] == 5
    assert document["field"]["field_to_map"] == {
        "x_m": 0.5, "y_m": 0.5, "yaw_rad": 0.0,
    }
    loaded = request_json(calibration_server + f"/api/calibration/drafts/{document['id']}")[1]
    assert loaded["data"]["map_source"]["fingerprint"]

    changed = {**document, "name": "Новое имя"}
    status, updated = request_json(
        calibration_server + f"/api/calibration/drafts/{document['id']}",
        method="PUT", payload={"revision": document["revision"], "document": changed},
    )
    assert status == 200
    assert updated["data"]["revision"] == 2

    status, conflict = request_json(
        calibration_server + f"/api/calibration/drafts/{document['id']}",
        method="PUT", payload={"revision": document["revision"], "document": changed},
    )
    assert status == 409
    assert conflict["errors"][0]["code"] == "REVISION_CONFLICT"


def test_validation_reports_readiness_and_invalid_draft_cannot_activate(calibration_server):
    maps = request_json(calibration_server + "/api/calibration/maps")[1]["data"]["files"]
    created = request_json(
        calibration_server + "/api/calibration/drafts", method="POST",
        payload={"name": "Поле", "map_source": {"kind": "file", "id": maps[0]["id"]}},
    )[1]["data"]

    status, checked = request_json(
        calibration_server + "/api/calibration/validate", method="POST",
        payload={"id": created["id"], "revision": created["revision"]},
    )
    assert status == 200
    assert checked["data"]["valid"] is True
    assert checked["data"]["activation_ready"] is False

    status, rejected = request_json(
        calibration_server + "/api/calibration/activate", method="POST",
        payload={"id": created["id"], "revision": created["revision"]},
    )
    assert status == 422
    assert rejected["errors"][0]["code"] == "ACTIVATION_REJECTED"


def test_calibration_mutations_require_json_and_reject_large_body(calibration_server):
    status, wrong_type = request_json(
        calibration_server + "/api/calibration/drafts",
        method="POST", payload=b"x", content_type="text/plain",
    )
    assert status == 415
    assert wrong_type["errors"][0]["code"] == "CONTENT_TYPE"

    host_port = calibration_server.removeprefix("http://")
    host, port = host_port.split(":")
    connection = http.client.HTTPConnection(host, int(port), timeout=3.0)
    connection.putrequest("POST", "/api/calibration/drafts")
    connection.putheader("Content-Type", "application/json")
    connection.putheader("Content-Length", str(1024 * 1024 + 1))
    connection.endheaders()
    response = connection.getresponse()
    too_large = json.loads(response.read())
    connection.close()
    assert response.status == 413
    assert too_large["errors"][0]["code"] == "REQUEST_TOO_LARGE"


def test_fresh_live_map_is_available_and_stale_live_map_is_not(tmp_path):
    map_data = {
        "width": 1,
        "height": 1,
        "resolution_m": 0.05,
        "origin": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "rle": [[0, 1]],
    }
    current = {"map": map_data, "age_s": 0.2}
    service = CalibrationService(
        MapCatalog([]), CalibrationStore(tmp_path / "store"),
        live_map_snapshot=lambda: current, live_map_max_age_s=2.0,
    )

    assert service.maps()["live"]["available"] is True
    assert service.map_preview("live")["source"]["kind"] == "live"
    current["age_s"] = 2.1
    assert service.maps()["live"]["available"] is False
    assert service.maps()["live"]["stale"] is True
