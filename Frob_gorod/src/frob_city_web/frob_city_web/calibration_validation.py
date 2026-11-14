"""Pure validation for field calibration documents."""

from __future__ import annotations

import math
from typing import Any, Mapping

from .calibration_model import GRID_SIZE, SCHEMA_VERSION, materialize_overlay


_REQUIRED_FOR_ACTIVATION = (
    "robot_safety.footprint",
    "robot_safety.stopping_distance_m",
    "robot_safety.maximum_speed_mps",
    "robot_safety.watchdog_timeout_s",
    "robot_safety.clearance_margin_m",
    "camera_signs.camera_to_base",
    "camera_signs.image_width_px",
    "camera_signs.image_height_px",
    "camera_signs.intrinsics",
    "localization.camera_frame",
    "localization.maximum_pose_covariance",
    "localization.maximum_jump_m",
    "localization.map_resolution_m",
    "mission.start_cell",
    "mission.start_heading_rad",
    "mission.completion_stop_count",
    "mission.stop_dwell_s",
    "mission.parking_prior",
    "mission.route_tolerance_m",
    "mission.observation_tolerance_rad",
    "ros_nav2.sole_cmd_vel_authority",
)


def _issue(code: str, path: str, message: str, object_id: str | None = None) -> dict[str, Any]:
    result = {"code": code, "path": path, "message": message}
    if object_id is not None:
        result["object_id"] = object_id
    return result


def _scan_finite(value: Any, path: str, errors: list[dict[str, Any]]) -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            errors.append(_issue("NON_FINITE", path, "value must be finite"))
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _scan_finite(item, f"{path}[{index}]", errors)
        return
    if isinstance(value, dict):
        for key, item in value.items():
            _scan_finite(item, f"{path}.{key}" if path else str(key), errors)


def _decode_rle(preview: Mapping[str, Any]) -> list[int] | None:
    values: list[int] = []
    try:
        for value, count in preview["rle"]:
            numeric_value, numeric_count = int(value), int(count)
            if numeric_value not in (-1, 0, 100) or numeric_count <= 0:
                return None
            values.extend([numeric_value] * numeric_count)
        expected = int(preview["width"]) * int(preview["height"])
    except (KeyError, TypeError, ValueError):
        return None
    return values if len(values) == expected else None


def _map_cell(point: list[float], preview: Mapping[str, Any]) -> tuple[int, int] | None:
    try:
        origin = preview["origin"]
        dx = float(point[0]) - float(origin["x_m"])
        dy = float(point[1]) - float(origin["y_m"])
        yaw = float(origin.get("yaw_rad", 0.0))
        resolution = float(preview["resolution_m"])
        width, height = int(preview["width"]), int(preview["height"])
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        column, row = math.floor(local_x / resolution), math.floor(local_y / resolution)
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        return None
    if 0 <= column < width and 0 <= row < height:
        return column, row
    return None


def _point_active(point: Mapping[str, Any], transform: Mapping[str, Any]) -> list[float]:
    if point.get("provenance") != "preset":
        return [float(point["active"][0]), float(point["active"][1])]
    from .calibration_model import transform_point
    return transform_point(point["nominal"], transform)


def _geometry_points(document: Mapping[str, Any]) -> list[tuple[str, str, list[float]]]:
    geometry = document["geometry"]
    transform = document["field"]["field_to_map"]
    points: list[tuple[str, str, list[float]]] = []
    for index, cell in enumerate(geometry["cells"]):
        if not cell.get("navigable", True):
            continue
        points.append((
            f"geometry.cells[{index}].navigation_point",
            str(cell.get("id", f"cell-{index}")),
            _point_active(cell["navigation_point"], transform),
        ))
    for collection, point_key in (("observation_points", "position"), ("sign_sites", "position")):
        for index, item in enumerate(geometry.get(collection, [])):
            points.append((
                f"geometry.{collection}[{index}].{point_key}",
                str(item.get("id", f"{collection}-{index}")),
                _point_active(item[point_key], transform),
            ))
    for road_index, road in enumerate(geometry.get("roads", [])):
        for point_index, point in enumerate(road.get("points", [])):
            points.append((
                f"geometry.roads[{road_index}].points[{point_index}]",
                str(road.get("id", f"road-{road_index}")),
                _point_active(point, transform),
            ))
    return points


def _validate_ids(document: Mapping[str, Any], errors: list[dict[str, Any]]) -> None:
    for collection in ("cells", "roads", "platforms", "observation_points", "sign_sites"):
        seen: set[str] = set()
        for index, item in enumerate(document["geometry"].get(collection, [])):
            item_id = item.get("id")
            path = f"geometry.{collection}[{index}].id"
            if not isinstance(item_id, str) or not item_id.strip():
                errors.append(_issue("MISSING_ID", path, "stable identifier is required"))
            elif item_id in seen:
                errors.append(_issue("DUPLICATE_ID", path, "stable identifier must be unique", item_id))
            else:
                seen.add(item_id)


def _readiness(document: Mapping[str, Any]) -> list[dict[str, Any]]:
    groups = document.get("calibrations", {})
    blockers = []
    for dotted in _REQUIRED_FOR_ACTIVATION:
        group_name, field_name = dotted.split(".", 1)
        setting = groups.get(group_name, {}).get(field_name)
        if not isinstance(setting, dict) or setting.get("value") is None:
            blockers.append(_issue(
                "MISSING_CALIBRATION",
                f"calibrations.{dotted}",
                "value is required before activation",
            ))
    return blockers


def validate_calibration(
    document: Mapping[str, Any],
    map_preview: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Validate geometry and report physical-readiness blockers separately."""

    errors: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    metrics = {"mean_residual_m": 0.0, "maximum_residual_m": 0.0}
    if not isinstance(document, Mapping):
        return {
            "valid": False,
            "activation_ready": False,
            "errors": [_issue("TYPE", "document", "calibration must be an object")],
            "warnings": [],
            "blockers": [],
            "metrics": metrics,
        }

    _scan_finite(document, "", errors)
    try:
        if document.get("schema_version") != SCHEMA_VERSION:
            errors.append(_issue("SCHEMA", "schema_version", f"expected schema {SCHEMA_VERSION}"))
        field = document["field"]
        geometry = document["geometry"]
        cells = geometry["cells"]
        if field.get("grid_size") != GRID_SIZE or field.get("size_m") != [4.0, 4.0] or len(cells) != 25:
            errors.append(_issue("FIXED_GRID", "field", "field must remain the fixed 5x5, 4.0 m preset"))
        expected_ids = {f"{chr(ord('A') + column)}{row}" for row in range(1, 6) for column in range(5)}
        actual_ids = {str(cell.get("id")) for cell in cells}
        if actual_ids != expected_ids:
            errors.append(_issue("CELL_IDS", "geometry.cells", "cells must be exactly A1 through E5"))
        _validate_ids(document, errors)
        overlay = materialize_overlay(document)
        residuals = [float(cell["residual_m"]) for cell in overlay["cells"]]
        if residuals:
            metrics["mean_residual_m"] = sum(residuals) / len(residuals)
            metrics["maximum_residual_m"] = max(residuals)
        for cell in overlay["cells"]:
            if cell["residual_m"] > 0.5:
                warnings.append(_issue(
                    "LARGE_LOCAL_CORRECTION",
                    "geometry.cells",
                    f"local correction is {cell['residual_m']:.3f} m",
                    cell["id"],
                ))
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        errors.append(_issue("STRUCTURE", "document", f"invalid calibration structure: {exc}"))

    preview_values = _decode_rle(map_preview) if map_preview is not None else None
    if map_preview is None:
        errors.append(_issue("MAP_MISSING", "map_source", "a usable map is required"))
    elif preview_values is None:
        errors.append(_issue("MAP_INVALID", "map_source", "map preview is malformed"))
    else:
        expected_fingerprint = document.get("map_source", {}).get("fingerprint")
        actual_fingerprint = map_preview.get("fingerprint")
        if expected_fingerprint and actual_fingerprint and expected_fingerprint != actual_fingerprint:
            errors.append(_issue("MAP_CHANGED", "map_source.fingerprint", "selected map changed after the draft was created"))
        try:
            for path, object_id, point in _geometry_points(document):
                cell = _map_cell(point, map_preview)
                if cell is None:
                    errors.append(_issue("OUTSIDE_MAP", path, "point is outside the occupancy map", object_id))
                    continue
                index = cell[1] * int(map_preview["width"]) + cell[0]
                if preview_values[index] == 100 and "navigation_point" in path:
                    errors.append(_issue("NAV_POINT_OCCUPIED", path, "navigation point lies in occupied map space", object_id))
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            errors.append(_issue("GEOMETRY", "geometry", f"cannot evaluate geometry: {exc}"))

    blockers = _readiness(document)
    return {
        "valid": not errors,
        "activation_ready": not errors and not blockers,
        "errors": errors,
        "warnings": warnings,
        "blockers": blockers,
        "metrics": metrics,
    }
