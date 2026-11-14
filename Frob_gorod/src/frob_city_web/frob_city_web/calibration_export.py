"""Export active web calibration into the strict city field YAML contract."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import yaml


def _cell_name(ui_id: str) -> str:
    column = ord(ui_id[0].upper()) - ord("A")
    row = int(ui_id[1:]) - 1
    return f"C{column}_{row}"


def _inverse_point(point: Sequence[float], transform: Mapping[str, Any]) -> list[float]:
    dx = float(point[0]) - float(transform["x_m"])
    dy = float(point[1]) - float(transform["y_m"])
    yaw = float(transform["yaw_rad"])
    return [math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy]


def _field_point(record: Mapping[str, Any], transform: Mapping[str, Any]) -> list[float]:
    if record.get("provenance") == "preset":
        return [float(record["nominal"][0]), float(record["nominal"][1])]
    return _inverse_point(record["active"], transform)


def _field_polygon(record: Mapping[str, Any], transform: Mapping[str, Any]) -> list[list[float]]:
    if record.get("provenance") == "preset":
        return [[float(value) for value in point] for point in record["nominal"]]
    return [_inverse_point(point, transform) for point in record["active"]]


def _poses(points: Sequence[Sequence[float]]) -> list[dict[str, float]]:
    result = []
    for index, point in enumerate(points):
        other = points[index + 1] if index + 1 < len(points) else points[index - 1]
        if index + 1 < len(points):
            yaw = math.atan2(float(other[1]) - float(point[1]), float(other[0]) - float(point[0]))
        else:
            yaw = math.atan2(float(point[1]) - float(other[1]), float(point[0]) - float(other[0]))
        result.append({"x_m": float(point[0]), "y_m": float(point[1]), "yaw_rad": yaw})
    return result


def _approach(from_cell: Mapping[str, Any], to_cell: Mapping[str, Any]) -> str:
    dc = int(to_cell["column"]) - int(from_cell["column"])
    dr = int(to_cell["row"]) - int(from_cell["row"])
    incoming = {(1, 0): "FROM_WEST", (-1, 0): "FROM_EAST", (0, 1): "FROM_SOUTH", (0, -1): "FROM_NORTH"}[(dc, dr)]
    return f"{_cell_name(to_cell['id'])}/{incoming}"


def _face_id(heading_rad: float) -> str:
    quarter_turn = int(round(float(heading_rad) / (math.pi / 2.0))) % 4
    return ("FACE_E", "FACE_N", "FACE_W", "FACE_S")[quarter_turn]


def export_field_yaml(document: Mapping[str, Any]) -> str:
    """Return deterministic YAML accepted by `load_project_config`."""

    transform = document["field"]["field_to_map"]
    geometry = document["geometry"]
    footprint = document["calibrations"]["robot_safety"]["footprint"].get("value")
    if not isinstance(footprint, list) or len(footprint) < 3:
        raise ValueError("robot footprint is required for field export")

    cells = {cell["id"]: cell for cell in geometry["cells"]}
    road_cells = [cell for cell in geometry["cells"] if cell.get("navigable")]
    regions = []
    anchors = []
    for cell in road_cells:
        x0, y0, x1, y1 = (float(value) for value in cell["bounds"])
        runtime_cell = _cell_name(cell["id"])
        regions.append({"id": f"REGION_{runtime_cell}", "polygon": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]})
        point = _field_point(cell["navigation_point"], transform)
        anchors.append({
            "id": f"ANCHOR_{runtime_cell}",
            "cell_id": runtime_cell,
            "pose": {"x_m": point[0], "y_m": point[1], "yaw_rad": 0.0},
        })

    edges = []
    for road in geometry["roads"]:
        first_cell, second_cell = cells[road["from_cell"]], cells[road["to_cell"]]
        points = [_field_point(point, transform) for point in road["points"]]
        for source, target, path in (
            (first_cell, second_cell, points),
            (second_cell, first_cell, list(reversed(points))),
        ):
            source_id, target_id = _cell_name(source["id"]), _cell_name(target["id"])
            edges.append({
                "id": f"{source_id}->{target_id}",
                "from_anchor_id": f"ANCHOR_{source_id}",
                "to_anchor_id": f"ANCHOR_{target_id}",
                "movement": "STRAIGHT",
                "path": _poses(path),
                "permitted_region_ids": [f"REGION_{source_id}", f"REGION_{target_id}"],
                "approach_id": _approach(source, target),
            })

    platforms = [
        {"id": item["id"], "polygon": _field_polygon(item["polygon"], transform)}
        for item in geometry["platforms"]
    ]
    observations = []
    for item in geometry["observation_points"]:
        point = _field_point(item["position"], transform)
        half_size = 0.08
        observations.append({
            "id": item["id"],
            "polygon": [
                [point[0] - half_size, point[1] - half_size],
                [point[0] + half_size, point[1] - half_size],
                [point[0] + half_size, point[1] + half_size],
                [point[0] - half_size, point[1] + half_size],
            ],
            "anchor_id": f"ANCHOR_{_cell_name(item['cell_id'])}",
            "nominal_heading_rad": float(item.get("heading_rad", 0.0)),
            "heading_tolerance_rad": 0.35,
        })

    sites = []
    for item in geometry["sign_sites"]:
        headings = item.get("allowed_headings_rad") or [item.get("heading_rad", 0.0)]
        sites.append({
            "id": item["id"],
            "position": _field_point(item["position"], transform),
            "faces": [
                {"id": _face_id(heading), "yaw_rad": float(heading), "controlled_approach_id": ""}
                for heading in headings
            ],
        })

    payload = {
        "schema_version": 2,
        "frame_id": "map",
        "field_size_m": [4.0, 4.0],
        "geometry_calibrated": False,
        "field_to_map": {
            "x_m": float(transform["x_m"]),
            "y_m": float(transform["y_m"]),
            "yaw_rad": float(transform["yaw_rad"]),
            "valid": True,
        },
        "footprint": [[float(value) for value in point] for point in footprint],
        "drivable_regions": regions,
        "platforms": platforms,
        "anchors": anchors,
        "edges": edges,
        "observation_regions": observations,
        "sign_sites": sites,
        "gates": [],
    }
    return yaml.safe_dump(payload, allow_unicode=True, sort_keys=False)
