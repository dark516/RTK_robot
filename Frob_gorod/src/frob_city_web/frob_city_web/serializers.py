"""Deterministic, JSON-safe conversion at the web boundary."""

from __future__ import annotations

import math
from typing import Any, Iterable


_SIGN_CLASS_LABELS = {
    0: "UNKNOWN", 1: "STRAIGHT_ONLY", 2: "LEFT_ONLY", 3: "RIGHT_ONLY",
    4: "NO_LEFT", 5: "NO_RIGHT", 6: "BUS_STOP", 7: "PARKING",
    8: "HAZARD_MARKER",
}
_RANGE_LABELS = {0: "UNKNOWN", 1: "OPTICAL_Z", 2: "RAY_RANGE"}


def _sign_class_json(value: Any) -> dict[str, Any]:
    numeric = int(value)
    return {"value": numeric, "label": _SIGN_CLASS_LABELS.get(numeric, f"UNKNOWN_{numeric}")}


def _range_json(value: Any) -> dict[str, Any]:
    numeric = int(value)
    return {"value": numeric, "label": _RANGE_LABELS.get(numeric, f"UNKNOWN_{numeric}")}


def _finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def stamp_seconds(stamp: Any) -> float:
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("invalid ROS timestamp")
    return value


def _header_json(header: Any) -> dict[str, Any]:
    return {
        "stamp_s": stamp_seconds(header.stamp),
        "frame_id": str(header.frame_id),
    }


def _yaw(orientation: Any) -> float | None:
    values = [_finite(getattr(orientation, name, None)) for name in ("x", "y", "z", "w")]
    if any(value is None for value in values):
        return None
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_json(value: Any) -> dict[str, Any]:
    header = getattr(value, "header", None)
    pose = getattr(value, "pose", value)
    position = pose.position
    result = {
        "x_m": _finite(position.x),
        "y_m": _finite(position.y),
        "z_m": _finite(getattr(position, "z", 0.0)),
        "yaw_rad": _yaw(pose.orientation),
    }
    if header is not None:
        result.update(_header_json(header))
    return result


def twist_json(twist: Any) -> dict[str, Any]:
    return {
        "linear_x_mps": _finite(twist.linear.x),
        "linear_y_mps": _finite(twist.linear.y),
        "angular_z_radps": _finite(twist.angular.z),
    }


def _as_integer(value: Any) -> int:
    # ROS 2 Jazzy may expose uint8 fields/constants as a one-byte ``bytes``
    # value (DiagnosticStatus.level is one example).
    if isinstance(value, (bytes, bytearray)) and len(value) == 1:
        return value[0]
    return int(value)


def _constant_items(source: Any) -> Iterable[tuple[str, int]]:
    for name in dir(source):
        if name.isupper():
            value = getattr(source, name, None)
            if isinstance(value, int):
                yield name, value
            elif isinstance(value, (bytes, bytearray)) and len(value) == 1:
                yield name, value[0]


def _enum_json(
    sources: Any | Iterable[Any],
    value: Any,
    *,
    prefix: str = "",
    exclude_prefix: str = "",
) -> dict[str, Any]:
    numeric = _as_integer(value)
    if not isinstance(sources, (tuple, list)):
        sources = (sources,)
    for source in sources:
        for name, constant in _constant_items(source):
            if prefix and not name.startswith(prefix):
                continue
            if exclude_prefix and name.startswith(exclude_prefix):
                continue
            if constant == numeric:
                label = name[len(prefix):] if prefix else name
                return {"value": numeric, "label": label}
    return {"value": numeric, "label": f"UNKNOWN_{numeric}"}


def _rle(values: list[int]) -> list[list[int]]:
    if not values:
        return []
    output: list[list[int]] = []
    current = values[0]
    count = 1
    for value in values[1:]:
        if value == current:
            count += 1
        else:
            output.append([current, count])
            current, count = value, 1
    output.append([current, count])
    return output


def occupancy_json(msg: Any) -> dict[str, Any]:
    width = int(msg.info.width)
    height = int(msg.info.height)
    if width <= 0 or height <= 0:
        raise ValueError("occupancy dimensions must be positive")
    values = [int(value) for value in msg.data]
    if len(values) != width * height:
        raise ValueError("occupancy data length does not match dimensions")
    if any(value < -1 or value > 100 for value in values):
        raise ValueError("occupancy value outside range -1..100")
    origin = msg.info.origin
    return {
        **_header_json(msg.header),
        "map_load_time_s": stamp_seconds(msg.info.map_load_time),
        "width": width,
        "height": height,
        "resolution_m": _finite(msg.info.resolution),
        "origin": {
            "x_m": _finite(origin.position.x),
            "y_m": _finite(origin.position.y),
            "yaw_rad": _yaw(origin.orientation),
        },
        "rle": _rle(values),
    }


def path_json(msg: Any) -> dict[str, Any]:
    return {**_header_json(msg.header), "poses": [pose_json(item) for item in msg.poses]}


def mission_json(msg: Any) -> dict[str, Any]:
    return {
        **_header_json(msg.header),
        "run_id": str(msg.run_id),
        "navigator_instance_id": str(msg.navigator_instance_id),
        "phase": _enum_json(msg, msg.phase, prefix="PHASE_"),
        "execution_state": _enum_json(msg, msg.execution_state, prefix="EXEC_"),
        "objective_id": str(msg.objective_id),
        "route_id": str(msg.route_id),
        "active_segment_id": str(msg.active_segment_id),
        "goal_generation": int(msg.goal_generation),
        "required_unique_bus_stops": int(msg.required_unique_bus_stops),
        "serviced_stop_ids": [str(item) for item in msg.serviced_stop_ids],
        "current_dwell_s": _finite(msg.current_dwell_s),
        "parking_prior_state": _enum_json(msg, msg.parking_prior_state, prefix="PARKING_PRIOR_"),
        "attempt_tainted": bool(msg.attempt_tainted),
        "reason": str(msg.reason),
    }


def guard_json(msg: Any) -> dict[str, Any]:
    return {
        **_header_json(msg.header),
        "run_id": str(msg.run_id),
        "map_instance_id": str(msg.map_instance_id),
        "navigator_instance_id": str(msg.navigator_instance_id),
        "goal_generation": int(msg.goal_generation),
        "last_permission_sequence": int(msg.last_permission_sequence),
        "active_segment_id": str(msg.active_segment_id),
        "motion_allowed": bool(msg.motion_allowed),
        "stopped": bool(msg.stopped),
        "fault_latched": bool(msg.fault_latched),
        "reason": str(msg.reason),
    }


def _polygon_msg_json(msg: Any) -> list[list[float | None]]:
    return [[_finite(point.x), _finite(point.y)] for point in msg.polygon.points]


def route_segment_json(msg: Any) -> dict[str, Any]:
    dependencies = []
    for index, dependency_id in enumerate(msg.dependency_ids):
        version = msg.dependency_versions[index] if index < len(msg.dependency_versions) else None
        dependencies.append({"id": str(dependency_id), "version": int(version) if version is not None else None})
    return {
        "segment_id": str(msg.segment_id),
        "kind": int(msg.kind),
        "from_anchor_id": str(msg.from_anchor_id),
        "to_anchor_id": str(msg.to_anchor_id),
        "approach_id": str(msg.approach_id),
        "selected_movement": int(msg.selected_movement),
        "path": path_json(msg.path),
        "allowed_regions": [_polygon_msg_json(item) for item in msg.allowed_regions],
        "dependencies": dependencies,
        "traversed_edge_ids": [str(value) for value in msg.traversed_edge_ids],
        "terminal_kind": int(msg.terminal_kind),
        "terminal_id": str(msg.terminal_id),
        "terminal_limit": pose_json(msg.terminal_limit),
        "commitment_gate": pose_json(msg.commitment_gate) if msg.commitment_gate_valid else None,
        "max_speed_mps": _finite(msg.max_speed_mps),
        "max_yaw_rate_radps": _finite(msg.max_yaw_rate_radps),
        "xy_tolerance_m": _finite(msg.xy_tolerance_m),
        "yaw_tolerance_rad": _finite(msg.yaw_tolerance_rad),
        "estimated_duration_s": _finite(msg.estimated_duration_s),
    }


def motion_permit_json(msg: Any) -> dict[str, Any]:
    return {
        **_header_json(msg.header),
        "run_id": str(msg.run_id),
        "map_instance_id": str(msg.map_instance_id),
        "navigator_instance_id": str(msg.navigator_instance_id),
        "field_id": str(msg.field_id),
        "goal_generation": int(msg.goal_generation),
        "sequence": int(msg.sequence),
        "enabled": bool(msg.enabled),
        "valid_for_s": _finite(msg.valid_for_s),
        "segment": route_segment_json(msg.segment),
    }


def _time_or_none(value: Any) -> float | None:
    try:
        result = stamp_seconds(value)
    except (AttributeError, TypeError, ValueError):
        return None
    return result if result > 0.0 else None


def map_state_json(msg: Any) -> dict[str, Any]:
    approaches = []
    for item in msg.approaches:
        approaches.append({
            "approach_id": str(item.approach_id),
            "version": int(item.version),
            "state": _enum_json(item, item.state),
            "confirmed_sign_classes": [
                _sign_class_json(value)
                for value in item.confirmed_sign_classes
            ],
            "allowed_movement_mask": int(item.allowed_movement_mask),
            "last_qualified_observation_s": _time_or_none(item.last_qualified_observation),
            "precautionary_hold": bool(item.precautionary_hold),
            "reason": str(item.reason),
        })
    targets = []
    for item in msg.targets:
        targets.append({
            "target_id": str(item.target_id),
            "version": int(item.version),
            "kind": _enum_json(item, item.kind),
            "state": _enum_json(item, item.state),
            "site_id": str(item.site_id),
            "face_id": str(item.face_id),
            "directed_road_id": str(item.directed_road_id),
            "service_anchor_id": str(item.service_anchor_id),
            "sign_pose": pose_json(item.sign_pose),
            "geometry_valid": bool(item.geometry_valid),
            "service_pose": pose_json(item.service_pose) if item.geometry_valid else None,
            "terminal_limit": pose_json(item.terminal_limit) if item.geometry_valid else None,
            "xy_tolerance_m": _finite(item.xy_tolerance_m),
            "yaw_tolerance_rad": _finite(item.yaw_tolerance_rad),
            "front_clearance_min_m": _finite(item.front_clearance_min_m),
            "last_seen_s": _time_or_none(item.last_seen),
        })
    known_signs = [{
        "site_id": str(item.site_id),
        "face_id": str(item.face_id),
        "approach_id": str(item.approach_id),
        "sign_class": _sign_class_json(item.sign_class),
        "version": int(item.version),
        "last_seen_s": _time_or_none(item.last_seen),
        "confidence": _finite(item.confidence),
    } for item in getattr(msg, "known_signs", [])]
    return {
        **_header_json(msg.header),
        "run_id": str(msg.run_id),
        "map_instance_id": str(msg.map_instance_id),
        "field_id": str(msg.field_id),
        "revision": int(msg.revision),
        "ready": bool(msg.ready),
        "perception_healthy": bool(msg.perception_healthy),
        "localization_healthy": bool(msg.localization_healthy),
        "precautionary_hold_ids": [str(value) for value in msg.precautionary_hold_ids],
        "approaches": approaches,
        "known_signs": known_signs,
        "targets": targets,
        "edges": [{
            "edge_id": str(item.edge_id), "version": int(item.version),
            "state": _enum_json(item, item.state),
            "completed_traversals": int(item.completed_traversals),
            "next_recheck_after_s": _time_or_none(item.next_recheck_after),
            "reason": str(item.reason),
        } for item in msg.edges],
        "coverage": [{
            "coverage_id": str(item.coverage_id), "version": int(item.version),
            "state": _enum_json(item, item.state), "failed_attempts": int(item.failed_attempts),
            "last_qualified_observation_s": _time_or_none(item.last_qualified_observation),
            "reason": str(item.reason),
        } for item in msg.coverage],
        "reason": str(msg.reason),
    }


def detections_json(msg: Any) -> dict[str, Any]:
    detections = []
    for item in msg.detections:
        detections.append({
            "class": _sign_class_json(item.sign_class),
            "confidence": _finite(item.confidence),
            "bbox": {"x": int(item.x_min_px), "y": int(item.y_min_px),
                     "width": int(item.width_px), "height": int(item.height_px)},
            "range_valid": bool(item.range_valid),
            "range_convention": _range_json(item.range_convention),
            "range_m": _finite(item.range_m) if item.range_valid else None,
            "range_stddev_m": _finite(item.range_stddev_m) if item.range_valid else None,
            "detector_track_id": str(item.detector_track_id),
        })
    return {
        **_header_json(msg.header),
        "detector_instance_id": str(msg.detector_instance_id),
        "frame_sequence": int(msg.frame_sequence),
        "image_width_px": int(msg.image_width_px),
        "image_height_px": int(msg.image_height_px),
        "calibration_id": str(msg.calibration_id),
        "frame_valid": bool(msg.frame_valid),
        "invalid_reason": str(msg.invalid_reason),
        "detections": detections,
    }


def diagnostics_json(msg: Any) -> dict[str, Any]:
    return {
        **_header_json(msg.header),
        "status": [{
            "level": _enum_json(item, item.level),
            "name": str(item.name), "message": str(item.message),
            "hardware_id": str(item.hardware_id),
            "values": {str(pair.key): str(pair.value) for pair in item.values},
        } for item in msg.status],
    }


def odom_json(msg: Any) -> dict[str, Any]:
    return {
        **_header_json(msg.header),
        "child_frame_id": str(msg.child_frame_id),
        "pose": pose_json(msg.pose.pose),
        "twist": twist_json(msg.twist.twist),
    }


def _point_pair(point: Any) -> list[float | None]:
    return [_finite(point.x_m), _finite(point.y_m)]


def _core_pose(value: Any) -> dict[str, float | None]:
    return {"x_m": _finite(value.x_m), "y_m": _finite(value.y_m), "yaw_rad": _finite(value.yaw_rad)}


def _core_polygon(value: Any) -> list[list[float | None]]:
    return [_point_pair(point) for point in value.points]


def field_layout_json(project_config: Any) -> dict[str, Any]:
    field = project_config.field
    return {
        "frame_id": str(field.frame_id),
        "size_m": [_finite(field.width_m), _finite(field.height_m)],
        "geometry_calibrated": bool(field.geometry_calibrated),
        "field_to_map": {
            "x_m": _finite(field.field_to_map.x_m),
            "y_m": _finite(field.field_to_map.y_m),
            "yaw_rad": _finite(field.field_to_map.yaw_rad),
            "valid": bool(field.field_to_map.valid),
        },
        "footprint": _core_polygon(field.footprint),
        "drivable_regions": [{"id": item.polygon_id, "points": _core_polygon(item.polygon)} for item in field.drivable_regions],
        "platforms": [{"id": item.polygon_id, "points": _core_polygon(item.polygon)} for item in field.platforms],
        "anchors": [{"id": item.anchor_id, "cell_id": item.cell_id, "pose": _core_pose(item.pose)} for item in field.anchors],
        "gates": [{"id": item.gate_id, "pose": _core_pose(item.pose)} for item in field.gates],
        "observation_regions": [{
            "id": item.observation_id, "anchor_id": item.anchor_id,
            "points": _core_polygon(item.polygon),
            "nominal_heading_rad": _finite(item.nominal_heading_rad),
            "heading_tolerance_rad": _finite(item.heading_tolerance_rad),
        } for item in field.observation_regions],
        "edges": [{
            "id": item.edge_id, "from_anchor_id": item.from_anchor_id,
            "to_anchor_id": item.to_anchor_id, "movement": int(item.movement),
            "path": [_core_pose(pose) for pose in item.path.poses],
            "permitted_region_ids": list(item.permitted_region_ids),
            "approach_id": item.approach_id,
            "commitment_gate_id": item.commitment_gate_id,
        } for item in field.edges],
        "sign_sites": [{
            "id": item.site_id, "position": _point_pair(item.position),
            "faces": [{"id": face.face_id, "yaw_rad": _finite(face.yaw_rad),
                       "controlled_approach_id": face.controlled_approach_id} for face in item.faces],
        } for item in field.sign_sites],
    }
