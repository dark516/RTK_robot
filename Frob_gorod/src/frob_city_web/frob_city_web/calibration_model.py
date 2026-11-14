"""ROS-independent calibration document and fixed field geometry."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import math
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = 1
FIELD_SIZE_M = 4.0
GRID_SIZE = 5
CELL_SIZE_M = FIELD_SIZE_M / GRID_SIZE
PLATFORM_CELLS = frozenset({"B2", "D2", "B4", "D4"})


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def normalize_angle(value: float) -> float:
    """Return a finite angle in the half-open range [0, 2π)."""

    result = float(value) % (2.0 * math.pi)
    return 0.0 if math.isclose(result, 2.0 * math.pi) else result


def transform_point(
    point: Sequence[float], transform: Mapping[str, float]
) -> list[float]:
    """Apply the rigid field-to-map transform to one metric point."""

    x_m, y_m = (float(value) for value in point)
    yaw = float(transform["yaw_rad"])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        float(transform["x_m"]) + cosine * x_m - sine * y_m,
        float(transform["y_m"]) + sine * x_m + cosine * y_m,
    ]


def _setting(value: Any, units: str = "", *, status: str | None = None) -> dict[str, Any]:
    return {
        "value": value,
        "units": units,
        "status": status or ("missing" if value is None else "draft"),
        "provenance": "preset" if value is not None else "unmeasured",
        "evidence": "",
        "measured_at": None,
    }


def _cells() -> list[dict[str, Any]]:
    cells = []
    for row in range(GRID_SIZE):
        for column in range(GRID_SIZE):
            cell_id = f"{chr(ord('A') + column)}{row + 1}"
            centre = [
                round((column + 0.5) * CELL_SIZE_M, 6),
                round((row + 0.5) * CELL_SIZE_M, 6),
            ]
            cells.append({
                "id": cell_id,
                "column": column,
                "row": row,
                "kind": "platform" if cell_id in PLATFORM_CELLS else "road",
                "navigable": cell_id not in PLATFORM_CELLS,
                "bounds": [
                    round(column * CELL_SIZE_M, 6),
                    round(row * CELL_SIZE_M, 6),
                    round((column + 1) * CELL_SIZE_M, 6),
                    round((row + 1) * CELL_SIZE_M, 6),
                ],
                "navigation_point": {
                    "nominal": list(centre),
                    "active": list(centre),
                    "provenance": "preset",
                },
            })
    return cells


def _editable_point(values: Sequence[float]) -> dict[str, Any]:
    point = [round(float(values[0]), 6), round(float(values[1]), 6)]
    return {"nominal": list(point), "active": list(point), "provenance": "preset"}


def _platforms(cells: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for cell in cells:
        if cell["id"] not in PLATFORM_CELLS:
            continue
        x0, y0, x1, y1 = cell["bounds"]
        points = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
        result.append({
            "id": f"PLATFORM_{cell['id']}",
            "cell_id": cell["id"],
            "polygon": {"nominal": points, "active": deepcopy(points), "provenance": "preset"},
        })
    return result


def _roads(cells: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    by_grid = {(int(cell["column"]), int(cell["row"])): cell for cell in cells}
    result = []
    for (column, row), cell in by_grid.items():
        if not cell["navigable"]:
            continue
        for neighbor_key in ((column + 1, row), (column, row + 1)):
            neighbor = by_grid.get(neighbor_key)
            if neighbor is None or not neighbor["navigable"]:
                continue
            result.append({
                "id": f"ROAD_{cell['id']}_{neighbor['id']}",
                "from_cell": cell["id"],
                "to_cell": neighbor["id"],
                "points": [
                    _editable_point(cell["navigation_point"]["nominal"]),
                    _editable_point(neighbor["navigation_point"]["nominal"]),
                ],
            })
    return result


def _sign_sites(platforms: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    result = []
    inset = 0.06
    names = ("SW", "SE", "NE", "NW")
    for platform in platforms:
        nominal = platform["polygon"]["nominal"]
        x0, y0 = nominal[0]
        x1, y1 = nominal[2]
        positions = (
            [x0 + inset, y0 + inset],
            [x1 - inset, y0 + inset],
            [x1 - inset, y1 - inset],
            [x0 + inset, y1 - inset],
        )
        for corner, position in zip(names, positions):
            result.append({
                "id": f"SIGN_{platform['cell_id']}_{corner}",
                "platform_id": platform["id"],
                "corner": corner,
                "position": _editable_point(position),
                "heading_rad": 0.0,
                "allowed_headings_rad": [0.0, math.pi / 2, math.pi, 3 * math.pi / 2],
                "sign_class": None,
            })
    return result


def _observation_points(
    cells: Sequence[Mapping[str, Any]],
    platforms: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    by_grid = {(int(cell["column"]), int(cell["row"])): cell for cell in cells}
    result = []
    for platform in platforms:
        platform_cell = next(cell for cell in cells if cell["id"] == platform["cell_id"])
        pc, pr = int(platform_cell["column"]), int(platform_cell["row"])
        for dc, dr in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            road_cell = by_grid.get((pc + dc, pr + dr))
            if road_cell is None or not road_cell["navigable"]:
                continue
            heading = math.atan2(pr - int(road_cell["row"]), pc - int(road_cell["column"]))
            result.append({
                "id": f"OBS_{platform_cell['id']}_FROM_{road_cell['id']}",
                "platform_id": platform["id"],
                "cell_id": road_cell["id"],
                "position": _editable_point(road_cell["navigation_point"]["nominal"]),
                "heading_rad": normalize_angle(heading),
            })
    return result


def _calibration_groups() -> dict[str, Any]:
    return {
        "robot_safety": {
            "footprint": _setting(None, "m"),
            "robot_radius_m": _setting(None, "m"),
            "stopping_distance_m": _setting(None, "m"),
            "command_latency_s": _setting(None, "s"),
            "perception_latency_s": _setting(None, "s"),
            "normal_speed_mps": _setting(None, "m/s"),
            "cautious_speed_mps": _setting(None, "m/s"),
            "maximum_speed_mps": _setting(None, "m/s"),
            "watchdog_timeout_s": _setting(None, "s"),
            "clearance_margin_m": _setting(None, "m"),
        },
        "camera_signs": {
            "camera_to_base": _setting(None, "m,rad"),
            "image_width_px": _setting(None, "px"),
            "image_height_px": _setting(None, "px"),
            "intrinsics": _setting(None, "px"),
            "distortion": _setting(None, ""),
            "sign_dimensions": _setting(None, "m"),
            "confidence_threshold": _setting(None, "ratio"),
            "range_uncertainty_m": _setting(None, "m"),
            "bearing_uncertainty_rad": _setting(None, "rad"),
        },
        "localization": {
            "map_frame": _setting("map"),
            "odom_frame": _setting("odom"),
            "base_frame": _setting("base_footprint"),
            "camera_frame": _setting(None),
            "maximum_pose_covariance": _setting(None),
            "maximum_jump_m": _setting(None, "m"),
            "map_resolution_m": _setting(None, "m/cell"),
            "stale_timeout_s": _setting(2.0, "s"),
        },
        "mission": {
            "start_cell": _setting(None),
            "start_heading_rad": _setting(None, "rad"),
            "completion_stop_count": _setting(None, "count"),
            "stop_dwell_s": _setting(None, "s"),
            "parking_prior": _setting(None),
            "route_tolerance_m": _setting(None, "m"),
            "observation_tolerance_rad": _setting(None, "rad"),
        },
        "ros_nav2": {
            "map_topic": _setting("/map"),
            "odom_topic": _setting("/odom"),
            "detections_topic": _setting("/traffic_sign/detections"),
            "nav2_action": _setting("/navigate_to_pose"),
            "motion_permit_topic": _setting("/city/motion_permit"),
            "cmd_vel_raw_topic": _setting("/city/cmd_vel_raw"),
            "cmd_vel_topic": _setting("/cmd_vel"),
            "sole_cmd_vel_authority": _setting(None),
            "critical_topic_timeout_s": _setting(2.0, "s"),
        },
    }


def new_calibration(name: str, map_source: Mapping[str, Any]) -> dict[str, Any]:
    """Create a draft around the only supported hackathon field preset."""

    now = _now_iso()
    cells = _cells()
    platforms = _platforms(cells)
    return {
        "schema_version": SCHEMA_VERSION,
        "id": None,
        "revision": 0,
        "name": str(name).strip() or "Новая калибровка",
        "lifecycle": "draft",
        "created_at": now,
        "updated_at": now,
        "map_source": deepcopy(dict(map_source)),
        "field": {
            "preset": "frob-city-5x5-v1",
            "grid_size": GRID_SIZE,
            "size_m": [FIELD_SIZE_M, FIELD_SIZE_M],
            "cell_size_m": CELL_SIZE_M,
            "field_to_map": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        },
        "geometry": {
            "cells": cells,
            "roads": _roads(cells),
            "platforms": platforms,
            "observation_points": _observation_points(cells, platforms),
            "sign_sites": _sign_sites(platforms),
        },
        "calibrations": _calibration_groups(),
        "validation": None,
        "activation": None,
    }


def materialize_overlay(document: Mapping[str, Any]) -> dict[str, Any]:
    """Return map-frame geometry without mutating the stored document."""

    transform = document["field"]["field_to_map"]
    cells = []
    for cell in document["geometry"]["cells"]:
        point = cell["navigation_point"]
        nominal_map = transform_point(point["nominal"], transform)
        active = nominal_map if point.get("provenance") == "preset" else [float(v) for v in point["active"]]
        cells.append({
            "id": cell["id"],
            "nominal_map": nominal_map,
            "active": active,
            "residual_m": math.hypot(active[0] - nominal_map[0], active[1] - nominal_map[1]),
        })
    def mapped_point(point: Mapping[str, Any]) -> dict[str, Any]:
        nominal_map = transform_point(point["nominal"], transform)
        active = nominal_map if point.get("provenance") == "preset" else [float(v) for v in point["active"]]
        return {
            "nominal_map": nominal_map,
            "active": active,
            "residual_m": math.hypot(active[0] - nominal_map[0], active[1] - nominal_map[1]),
        }

    roads = []
    for road in document["geometry"].get("roads", []):
        points = [mapped_point(point) for point in road["points"]]
        roads.append({"id": road["id"], "points": [point["active"] for point in points], "handles": points})

    platforms = []
    for platform in document["geometry"].get("platforms", []):
        polygon = platform["polygon"]
        nominal_points = [transform_point(point, transform) for point in polygon["nominal"]]
        points = nominal_points if polygon.get("provenance") == "preset" else [
            [float(value) for value in point] for point in polygon["active"]
        ]
        platforms.append({"id": platform["id"], "cell_id": platform["cell_id"], "points": points,
                          "nominal_points": nominal_points})

    def point_collection(name: str) -> list[dict[str, Any]]:
        result = []
        for item in document["geometry"].get(name, []):
            point = mapped_point(item["position"])
            result.append({**{key: value for key, value in item.items() if key != "position"}, **point})
        return result

    return {
        "cells": cells,
        "roads": roads,
        "platforms": platforms,
        "observation_points": point_collection("observation_points"),
        "sign_sites": point_collection("sign_sites"),
    }
