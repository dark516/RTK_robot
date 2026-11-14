"""Deterministic autonomous topology derived from the calibrated 5x5 field.

The Web editor remains the source of surveyed geometry.  This module only
replaces its display-oriented adjacent-cell edges with directed, atomic road
maneuvers suitable for the mission planner.
"""
from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path

import yaml


def _cell(cell_id: str) -> tuple[int, int]:
    x_text, y_text = cell_id.removeprefix("C").split("_", 1)
    return int(x_text), int(y_text)


def _angle(vector: tuple[int, int]) -> float:
    return math.atan2(vector[1], vector[0])


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _from_name(vector_from_intersection: tuple[int, int]) -> str:
    return {
        (-1, 0): "WEST",
        (1, 0): "EAST",
        (0, -1): "SOUTH",
        (0, 1): "NORTH",
    }[vector_from_intersection]


def _pose(x: float, y: float, yaw: float) -> dict:
    return {"x_m": float(x), "y_m": float(y), "yaw_rad": float(yaw)}


def _maneuver_path(source, intersection, destination, radius_m: float = 0.28) -> list[dict]:
    sx, sy = source
    ix, iy = intersection
    dx, dy = destination
    vin = ((ix - sx) / math.hypot(ix - sx, iy - sy), (iy - sy) / math.hypot(ix - sx, iy - sy))
    vout = ((dx - ix) / math.hypot(dx - ix, dy - iy), (dy - iy) / math.hypot(dx - ix, dy - iy))
    incoming_yaw = math.atan2(vin[1], vin[0])
    outgoing_yaw = math.atan2(vout[1], vout[0])
    if vin == vout:
        return [_pose(sx, sy, incoming_yaw), _pose(ix, iy, incoming_yaw), _pose(dx, dy, incoming_yaw)]

    before = (ix - vin[0] * radius_m, iy - vin[1] * radius_m)
    after = (ix + vout[0] * radius_m, iy + vout[1] * radius_m)
    points = [(sx, sy), before]
    # Quadratic Bezier gives a smooth, tangent-continuous turn wholly inside
    # the surveyed intersection cell.
    for step in range(1, 8):
        t = step / 7.0
        omt = 1.0 - t
        x = omt * omt * before[0] + 2.0 * omt * t * ix + t * t * after[0]
        y = omt * omt * before[1] + 2.0 * omt * t * iy + t * t * after[1]
        points.append((x, y))
    points.append((dx, dy))
    poses = []
    for index, (x, y) in enumerate(points):
        if index + 1 < len(points):
            nx, ny = points[index + 1]
            yaw = math.atan2(ny - y, nx - x)
        else:
            yaw = outgoing_yaw
        poses.append(_pose(x, y, yaw))
    poses[0]["yaw_rad"] = incoming_yaw
    poses[-1]["yaw_rad"] = outgoing_yaw
    return poses


def build_competition_field(manual: dict) -> dict:
    field = deepcopy(manual)
    anchors = {item["id"]: item for item in field["anchors"]}
    by_cell = {_cell(item["cell_id"]): item for item in field["anchors"]}

    intersections = {
        cell: item for cell, item in by_cell.items()
        if cell[0] % 2 == 0 and cell[1] % 2 == 0
    }
    neighbor_vectors = ((-1, 0), (1, 0), (0, -1), (0, 1))
    edges: list[dict] = []
    gates: list[dict] = []
    observations: dict[str, dict] = {}
    approaches: list[tuple[str, float, tuple[float, float]]] = []

    for intersection_cell in sorted(intersections):
        intersection_anchor = intersections[intersection_cell]
        ix = float(intersection_anchor["pose"]["x_m"])
        iy = float(intersection_anchor["pose"]["y_m"])
        neighbors = []
        for vx, vy in neighbor_vectors:
            item = by_cell.get((intersection_cell[0] + vx, intersection_cell[1] + vy))
            if item is not None:
                neighbors.append(((vx, vy), item))

        for (vx, vy), neighbor in neighbors:
            nx = float(neighbor["pose"]["x_m"])
            ny = float(neighbor["pose"]["y_m"])
            yaw = math.atan2(ny - iy, nx - ix)
            edge_id = f"START_{intersection_anchor['cell_id']}_TO_{neighbor['cell_id']}"
            edges.append({
                "id": edge_id,
                "from_anchor_id": intersection_anchor["id"],
                "to_anchor_id": neighbor["id"],
                "movement": "STRAIGHT",
                "path": [_pose(ix, iy, yaw), _pose(nx, ny, yaw)],
                "permitted_region_ids": [
                    f"REGION_{intersection_anchor['cell_id']}",
                    f"REGION_{neighbor['cell_id']}",
                ],
                "approach_id": "",
            })

        for (svx, svy), source in neighbors:
            sx = float(source["pose"]["x_m"])
            sy = float(source["pose"]["y_m"])
            travel = (-svx, -svy)
            incoming_yaw = _angle(travel)
            approach_id = f"{intersection_anchor['cell_id']}/FROM_{_from_name((svx, svy))}"
            observation_id = f"OBS_{approach_id.replace('/', '_')}"
            half = 0.22
            observations[approach_id] = {
                "id": observation_id,
                "polygon": [
                    [sx - half, sy - half], [sx + half, sy - half],
                    [sx + half, sy + half], [sx - half, sy + half],
                ],
                "anchor_id": source["id"],
                "nominal_heading_rad": incoming_yaw,
                "heading_tolerance_rad": 0.40,
            }
            approaches.append((approach_id, incoming_yaw, (sx, sy)))
            gate_id = f"GATE_{approach_id.replace('/', '_')}"
            gates.append({
                "id": gate_id,
                "pose": _pose(ix + svx * 0.35, iy + svy * 0.35, incoming_yaw),
            })

            for (dvx, dvy), destination in neighbors:
                if (dvx, dvy) == (svx, svy):
                    continue  # no U-turn
                dx = float(destination["pose"]["x_m"])
                dy = float(destination["pose"]["y_m"])
                cross = travel[0] * dvy - travel[1] * dvx
                dot = travel[0] * dvx + travel[1] * dvy
                movement = "STRAIGHT" if dot > 0 else ("LEFT" if cross > 0 else "RIGHT")
                edge_id = (
                    f"MANEUVER_{source['cell_id']}_VIA_{intersection_anchor['cell_id']}"
                    f"_TO_{destination['cell_id']}"
                )
                edges.append({
                    "id": edge_id,
                    "from_anchor_id": source["id"],
                    "to_anchor_id": destination["id"],
                    "movement": movement,
                    "path": _maneuver_path((sx, sy), (ix, iy), (dx, dy)),
                    "permitted_region_ids": [
                        f"REGION_{source['cell_id']}",
                        f"REGION_{intersection_anchor['cell_id']}",
                        f"REGION_{destination['cell_id']}",
                    ],
                    "approach_id": approach_id,
                    "commitment_gate_id": gate_id,
                })

    for site in field["sign_sites"]:
        px, py = map(float, site["position"])
        for face in site["faces"]:
            travel_yaw = float(face["yaw_rad"]) - math.pi / 2.0
            candidates = []
            for approach_id, yaw, (sx, sy) in approaches:
                if abs(_angle_delta(yaw, travel_yaw)) > 0.05:
                    continue
                rx, ry = px - sx, py - sy
                forward = math.cos(yaw) * rx + math.sin(yaw) * ry
                right = math.sin(yaw) * rx - math.cos(yaw) * ry
                if 0.20 <= right <= 0.70 and -0.50 <= forward <= 0.75:
                    score = abs(right - 0.46) + 0.20 * abs(forward - 0.34)
                    candidates.append((score, approach_id))
            face["controlled_approach_id"] = min(candidates)[1] if candidates else ""

    field["edges"] = sorted(edges, key=lambda item: item["id"])
    field["gates"] = sorted(gates, key=lambda item: item["id"])
    field["observation_regions"] = [observations[key] for key in sorted(observations)]
    return field


def write_competition_profile(manual_field: Path, output_field: Path) -> None:
    source = yaml.safe_load(manual_field.read_text())
    output_field.write_text(yaml.safe_dump(build_competition_field(source), sort_keys=False))


def build_competition_nav2(source: dict, start_pose: dict) -> dict:
    params = deepcopy(source)
    amcl = params["amcl"]["ros__parameters"]
    amcl["set_initial_pose"] = True
    amcl["always_reset_initial_pose"] = True
    amcl["initial_pose"] = {
        "x": float(start_pose["x_m"]),
        "y": float(start_pose["y_m"]),
        "z": 0.0,
        "yaw": float(start_pose["yaw_rad"]),
    }
    controller = params["controller_server"]["ros__parameters"]["FollowPath"]
    controller["desired_linear_vel"] = 0.12
    controller["rotate_to_heading_angular_vel"] = 0.35
    controller["max_angular_accel"] = 1.0
    controller["rotate_to_heading_once"] = True
    controller["allow_reversing"] = False
    smoother = params["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [0.12, 0.0, 0.45]
    smoother["min_velocity"] = [0.0, 0.0, -0.45]
    smoother["max_accel"] = [0.25, 0.0, 1.0]
    smoother["max_decel"] = [-0.35, 0.0, -1.0]
    smoother["velocity_timeout"] = 0.30
    behavior = params.get("behavior_server", {}).get("ros__parameters", {})
    if behavior:
        # bt_navigator's stock XML requires Spin to exist during activation.
        # Its velocity still traverses collision_monitor and the fail-closed
        # motion guard; reverse/drive-on-heading/assisted teleop stay absent.
        behavior["behavior_plugins"] = ["spin", "wait"]
        for key in ("backup", "drive_on_heading", "assisted_teleop"):
            behavior.pop(key, None)
    collision = params["collision_monitor"]["ros__parameters"]
    collision["cmd_vel_out_topic"] = "/city/cmd_vel_raw"
    collision["source_timeout"] = 0.40
    return params


def write_competition_nav2(source_file: Path, event_file: Path, output_file: Path) -> None:
    source = yaml.safe_load(source_file.read_text())
    event = yaml.safe_load(event_file.read_text())
    output_file.write_text(
        yaml.safe_dump(build_competition_nav2(source, event["start"]["map_pose"]), sort_keys=False)
    )
