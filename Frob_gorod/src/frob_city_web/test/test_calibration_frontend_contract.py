from html.parser import HTMLParser
from pathlib import Path
import shutil
import subprocess

import pytest


STATIC = Path(__file__).parents[1] / "frob_city_web" / "static"


class SurfaceParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()
        self.data = []
        self.grid_size = None

    def handle_starttag(self, _tag, attrs):
        values = dict(attrs)
        if values.get("id"):
            self.ids.add(values["id"])
        if values.get("data-grid-size"):
            self.grid_size = values["data-grid-size"]

    def handle_data(self, data):
        if data.strip():
            self.data.append(data.strip())


def test_calibration_page_contains_the_complete_operator_workspace():
    parser = SurfaceParser()
    parser.feed((STATIC / "calibration.html").read_text(encoding="utf-8"))

    assert parser.grid_size == "5"
    assert {
        "calibration-canvas", "map-picker", "draft-picker", "draft-name",
        "save-draft", "validate-draft", "activate-draft", "calibration-tabs",
        "calibration-inspector", "validation-list", "source-badge", "layer-controls",
    } <= parser.ids
    content = " ".join(parser.data)
    for label in (
        "Поле", "Робот и безопасность", "Камера и знаки", "Локализация",
        "Миссия", "ROS / Nav2", "Готовность",
    ):
        assert label in content
    assert "Количество клеток" not in content


def test_calibration_frontend_has_no_external_assets_or_motion_commands():
    joined = "".join(
        (STATIC / name).read_text(encoding="utf-8")
        for name in ("calibration.html", "calibration.css", "calibration.js")
    )
    assert "https://" not in joined and "http://" not in joined
    assert "/api/calibration/maps" in joined
    assert "/api/calibration/validate" in joined
    assert "/api/control" not in joined
    assert "data-command" not in (STATIC / "calibration.html").read_text(encoding="utf-8")


def test_calibration_js_geometry_and_local_edit_behaviour():
    if shutil.which("node") is None:
        pytest.skip("Node.js is only needed for the source-level frontend contract test")
    script = r"""
const assert = require('node:assert/strict');
const core = require(process.argv[1]);
const draft = core.createFixedPreset('test', {kind: 'file', id: 'map'});
assert.equal(draft.field.grid_size, 5);
assert.equal(draft.geometry.cells.length, 25);
assert.equal(core.mapSelectionMode(null, 'map'), 'load');
assert.equal(core.mapSelectionMode(draft, 'map'), 'load');
assert.equal(core.mapSelectionMode(draft, 'other-map'), 'new-draft');
const transformed = core.fieldToMap([1, 0], {x_m: 2, y_m: 3, yaw_rad: Math.PI});
assert.ok(Math.abs(transformed[0] - 1) < 1e-9);
assert.ok(Math.abs(transformed[1] - 3) < 1e-9);
draft.field.field_to_map = {x_m: -4.1, y_m: -1.7, yaw_rad: 0};
const centreBefore = core.fieldToMap([2, 2], draft.field.field_to_map);
core.setYawAroundFieldCenter(draft, Math.PI);
const centreAfter = core.fieldToMap([2, 2], draft.field.field_to_map);
assert.ok(Math.abs(centreAfter[0] - centreBefore[0]) < 1e-9);
assert.ok(Math.abs(centreAfter[1] - centreBefore[1]) < 1e-9);
const before = JSON.stringify(draft.field.field_to_map);
core.applyLocalPointEdit(draft, 'cell:A1', [1.25, 2.5]);
assert.equal(JSON.stringify(draft.field.field_to_map), before);
assert.deepEqual(draft.geometry.cells[0].navigation_point.active, [1.25, 2.5]);
assert.equal(draft.geometry.cells[0].navigation_point.provenance, 'drag');
draft.geometry.roads = [{id: 'ROAD_A1_B1', points: [
  {nominal: [0.4, 0.4], active: [0.4, 0.4], provenance: 'preset'},
  {nominal: [1.2, 0.4], active: [1.2, 0.4], provenance: 'preset'},
]}];
const inserted = core.insertRoadPoint(draft, 'road:ROAD_A1_B1:0');
assert.equal(inserted, 'road:ROAD_A1_B1:1');
assert.equal(draft.geometry.roads[0].points.length, 3);
assert.deepEqual(draft.geometry.roads[0].points[1].nominal, [0.8, 0.4]);
assert.equal(draft.geometry.roads[0].points[1].provenance, 'drag');
core.removeRoadPoint(draft, inserted);
assert.equal(draft.geometry.roads[0].points.length, 2);
assert.throws(() => core.removeRoadPoint(draft, 'road:ROAD_A1_B1:0'), /endpoint/);
assert.deepEqual(core.decodeRle([[0, 2], [100, 1], [-1, 1]], 4), [0, 0, 100, -1]);
assert.deepEqual(core.validateDraftClientSide(draft), []);
draft.field.grid_size = 4;
assert.equal(core.validateDraftClientSide(draft)[0].code, 'FIXED_GRID');
"""
    result = subprocess.run(
        ["node", "-e", script, str(STATIC / "calibration.js")],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def test_existing_dashboard_links_to_calibration_settings():
    html = (STATIC / "index.html").read_text(encoding="utf-8")

    assert 'href="/settings/calibration"' in html
