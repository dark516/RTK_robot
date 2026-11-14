import math

import pytest

from frob_city_web.calibration_model import (
    materialize_overlay,
    new_calibration,
    normalize_angle,
    transform_point,
)


def test_new_calibration_is_always_fixed_5_by_5():
    document = new_calibration("Поле", {"kind": "file", "id": "map"})

    assert document["field"]["grid_size"] == 5
    assert document["field"]["size_m"] == [4.0, 4.0]
    assert len(document["geometry"]["cells"]) == 25
    assert document["geometry"]["cells"][0]["id"] == "A1"
    assert document["geometry"]["cells"][-1]["id"] == "E5"
    assert all(
        cell["navigation_point"]["nominal"] == cell["navigation_point"]["active"]
        for cell in document["geometry"]["cells"]
    )


def test_fixed_preset_contains_nominal_platforms_roads_and_sign_holes():
    document = new_calibration("Поле", {"kind": "file", "id": "map"})
    cells = {cell["id"]: cell for cell in document["geometry"]["cells"]}

    assert {cell_id for cell_id, cell in cells.items() if cell["kind"] == "platform"} == {
        "B2", "D2", "B4", "D4",
    }
    assert sum(cell["navigable"] for cell in cells.values()) == 21
    assert len(document["geometry"]["platforms"]) == 4
    assert len(document["geometry"]["roads"]) == 24
    assert len(document["geometry"]["sign_sites"]) == 16
    assert {site["platform_id"] for site in document["geometry"]["sign_sites"]} == {
        "PLATFORM_B2", "PLATFORM_D2", "PLATFORM_B4", "PLATFORM_D4",
    }


def test_overlay_materializes_all_preset_layers_in_map_frame():
    document = new_calibration("Поле", {"kind": "file", "id": "map"})
    document["field"]["field_to_map"] = {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0}

    overlay = materialize_overlay(document)

    assert len(overlay["roads"]) == 24
    assert len(overlay["platforms"]) == 4
    assert len(overlay["sign_sites"]) == 16
    assert overlay["platforms"][0]["points"][0] == pytest.approx([1.8, 2.8])
    assert overlay["sign_sites"][0]["active"][0] > 1.8


@pytest.mark.parametrize(
    ("degrees", "expected"),
    ((0, [1.0, 0.0]), (90, [0.0, 1.0]), (180, [-1.0, 0.0]), (270, [0.0, -1.0])),
)
def test_transform_handles_orientation_shortcuts(degrees, expected):
    actual = transform_point(
        [1.0, 0.0],
        {"x_m": 0.0, "y_m": 0.0, "yaw_rad": math.radians(degrees)},
    )

    assert actual == pytest.approx(expected)


def test_materialized_local_override_does_not_change_global_transform():
    document = new_calibration("x", {"kind": "file", "id": "map"})
    document["field"]["field_to_map"] = {"x_m": 2.0, "y_m": -1.0, "yaw_rad": math.pi / 2}
    point = document["geometry"]["cells"][0]["navigation_point"]
    point["active"] = [2.1, 3.2]
    point["provenance"] = "drag"
    before = dict(document["field"]["field_to_map"])

    overlay = materialize_overlay(document)

    assert document["field"]["field_to_map"] == before
    assert overlay["cells"][0]["active"] == [2.1, 3.2]
    assert overlay["cells"][1]["active"] == pytest.approx([1.6, 0.2])


def test_overlay_reports_local_residual_from_rigid_preset():
    document = new_calibration("x", {"kind": "file", "id": "map"})
    point = document["geometry"]["cells"][0]["navigation_point"]
    point["active"] = [0.7, 0.8]
    point["provenance"] = "numeric"

    overlay = materialize_overlay(document)

    assert overlay["cells"][0]["nominal_map"] == pytest.approx([0.4, 0.4])
    assert overlay["cells"][0]["residual_m"] == pytest.approx(0.5)


def test_new_calibration_contains_future_calibration_groups_as_missing_values():
    document = new_calibration("x", {"kind": "live", "topic": "/map"})

    assert set(document["calibrations"]) == {
        "robot_safety",
        "camera_signs",
        "localization",
        "mission",
        "ros_nav2",
    }
    assert document["calibrations"]["robot_safety"]["footprint"]["status"] == "missing"
    assert document["calibrations"]["ros_nav2"]["cmd_vel_topic"]["value"] == "/cmd_vel"


@pytest.mark.parametrize(
    ("raw", "expected"),
    ((0.0, 0.0), (2 * math.pi, 0.0), (-math.pi / 2, 3 * math.pi / 2)),
)
def test_normalize_angle_uses_zero_to_two_pi(raw, expected):
    assert normalize_angle(raw) == pytest.approx(expected)
