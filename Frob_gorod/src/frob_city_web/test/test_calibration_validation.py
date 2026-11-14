import math

import pytest

from frob_city_web.calibration_model import new_calibration
from frob_city_web.calibration_validation import validate_calibration


def map_preview(*, occupied_first=False):
    values = [100 if occupied_first else 0] + [0] * 99
    runs = []
    for value in values:
        if runs and runs[-1][0] == value:
            runs[-1][1] += 1
        else:
            runs.append([value, 1])
    return {
        "width": 10,
        "height": 10,
        "resolution_m": 0.5,
        "origin": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "rle": runs,
        "fingerprint": "a" * 64,
    }


def document():
    result = new_calibration(
        "x", {"kind": "file", "id": "map", "fingerprint": "a" * 64}
    )
    return result


def test_validator_rejects_non_finite_points_and_wrong_grid_size():
    draft = document()
    draft["field"]["grid_size"] = 4
    draft["geometry"]["cells"][0]["navigation_point"]["active"][0] = float("nan")

    result = validate_calibration(draft, map_preview())

    assert result["valid"] is False
    assert {item["code"] for item in result["errors"]} >= {"FIXED_GRID", "NON_FINITE"}


def test_validator_warns_about_large_local_residual_without_bending_transform():
    draft = document()
    before = dict(draft["field"]["field_to_map"])
    point = draft["geometry"]["cells"][0]["navigation_point"]
    point["active"] = [point["nominal"][0] + 0.75, point["nominal"][1]]
    point["provenance"] = "drag"

    result = validate_calibration(draft, map_preview())

    assert result["valid"] is True
    assert "LARGE_LOCAL_CORRECTION" in {item["code"] for item in result["warnings"]}
    assert draft["field"]["field_to_map"] == before
    assert result["metrics"]["maximum_residual_m"] == pytest.approx(0.75)


def test_validator_rejects_changed_map_fingerprint():
    draft = document()

    result = validate_calibration(draft, {**map_preview(), "fingerprint": "b" * 64})

    assert result["valid"] is False
    assert "MAP_CHANGED" in {item["code"] for item in result["errors"]}


def test_validator_rejects_navigation_point_outside_map():
    draft = document()
    point = draft["geometry"]["cells"][0]["navigation_point"]
    point["active"] = [8.0, 8.0]
    point["provenance"] = "numeric"

    result = validate_calibration(draft, map_preview())

    assert "OUTSIDE_MAP" in {item["code"] for item in result["errors"]}


def test_validator_rejects_navigation_point_in_occupied_map_cell():
    draft = document()
    point = draft["geometry"]["cells"][0]["navigation_point"]
    point["active"] = [0.1, 0.1]
    point["provenance"] = "numeric"

    result = validate_calibration(draft, map_preview(occupied_first=True))

    assert "NAV_POINT_OCCUPIED" in {item["code"] for item in result["errors"]}


def test_platform_cell_centres_are_not_validated_as_navigation_targets():
    draft = document()
    platform_cell = next(cell for cell in draft["geometry"]["cells"] if cell["id"] == "B2")
    platform_cell["navigation_point"]["active"] = [0.1, 0.1]
    platform_cell["navigation_point"]["provenance"] = "numeric"

    result = validate_calibration(draft, map_preview(occupied_first=True))

    occupied = [item for item in result["errors"] if item["code"] == "NAV_POINT_OCCUPIED"]
    assert all(item.get("object_id") != "B2" for item in occupied)


def test_readiness_lists_missing_physical_calibrations_separately_from_geometry():
    result = validate_calibration(document(), map_preview())

    assert result["valid"] is True
    assert result["activation_ready"] is False
    blocker_paths = {item["path"] for item in result["blockers"]}
    assert "calibrations.robot_safety.footprint" in blocker_paths
    assert "calibrations.ros_nav2.sole_cmd_vel_authority" in blocker_paths


def test_rotated_map_origin_is_used_for_occupancy_lookup():
    draft = document()
    point = draft["geometry"]["cells"][0]["navigation_point"]
    point["active"] = [-0.1, 0.1]
    point["provenance"] = "numeric"
    preview = map_preview(occupied_first=True)
    preview["origin"]["yaw_rad"] = math.pi / 2

    result = validate_calibration(draft, preview)

    assert "NAV_POINT_OCCUPIED" in {item["code"] for item in result["errors"]}
