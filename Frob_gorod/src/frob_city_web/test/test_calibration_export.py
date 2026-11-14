from pathlib import Path

import pytest

from frob_city_navigation.config_loader import load_project_config
from frob_city_navigation.models import ConfigPaths
from frob_city_web.calibration_export import export_field_yaml
from frob_city_web.calibration_model import new_calibration


NAV_CONFIG = Path(__file__).parents[2] / "frob_city_navigation" / "config"


def calibrated_document():
    document = new_calibration(
        "field", {"kind": "file", "id": "map", "fingerprint": "a" * 64}
    )
    document["field"]["field_to_map"] = {"x_m": 1.25, "y_m": -0.5, "yaw_rad": 0.3}
    document["calibrations"]["robot_safety"]["footprint"]["value"] = [
        [-0.15, -0.12], [0.18, -0.12], [0.18, 0.12], [-0.15, 0.12],
    ]
    return document


def test_export_is_accepted_by_existing_strict_project_loader(tmp_path):
    document = calibrated_document()
    field_path = tmp_path / "field.yaml"
    field_path.write_text(export_field_yaml(document), encoding="utf-8")

    loaded = load_project_config(ConfigPaths(
        field_path,
        NAV_CONFIG / "wp13_test_event.yaml",
        NAV_CONFIG / "wp13_test_perception.yaml",
        NAV_CONFIG / "wp13_test_navigation.yaml",
    ))

    assert loaded.field.field_to_map.x_m == pytest.approx(1.25)
    assert loaded.field.field_to_map.yaw_rad == pytest.approx(0.3)
    assert loaded.field.field_to_map.valid is True
    assert loaded.field.geometry_calibrated is False
    assert len(loaded.field.anchors) == 21
    assert len(loaded.field.edges) == 48
    assert len(loaded.field.platforms) == 4
    assert len(loaded.field.sign_sites) == 16
    assert {
        face.face_id
        for site in loaded.field.sign_sites
        for face in site.faces
    } == {"FACE_E", "FACE_N", "FACE_W", "FACE_S"}


def test_local_map_point_is_exported_back_into_field_coordinates(tmp_path):
    document = calibrated_document()
    cell = next(item for item in document["geometry"]["cells"] if item["id"] == "A1")
    cell["navigation_point"]["active"] = [2.0, 1.0]
    cell["navigation_point"]["provenance"] = "numeric"

    text = export_field_yaml(document)
    field_path = tmp_path / "field.yaml"
    field_path.write_text(text, encoding="utf-8")
    loaded = load_project_config(ConfigPaths(
        field_path,
        NAV_CONFIG / "wp13_test_event.yaml",
        NAV_CONFIG / "wp13_test_perception.yaml",
        NAV_CONFIG / "wp13_test_navigation.yaml",
    ))
    anchor = next(item for item in loaded.field.anchors if item.cell_id == "C0_0")
    c, s = __import__("math").cos(0.3), __import__("math").sin(0.3)
    expected_x = c * (2.0 - 1.25) + s * (1.0 + 0.5)
    expected_y = -s * (2.0 - 1.25) + c * (1.0 + 0.5)

    assert anchor.pose.x_m == pytest.approx(expected_x)
    assert anchor.pose.y_m == pytest.approx(expected_y)
