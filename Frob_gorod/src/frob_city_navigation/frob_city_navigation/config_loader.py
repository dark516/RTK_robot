"""Strict YAML loader and validator for the Frob City pure-domain configuration."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict
from enum import IntEnum, IntFlag
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence, Type, TypeVar

import yaml

from .models import (
    CONTRACT_VERSION,
    ConfigPaths,
    DirectedAnchorConfig,
    DirectedEdgeConfig,
    EventConfig,
    FieldConfig,
    FieldTransform,
    GateConfig,
    MetricPath,
    MovementMask,
    NamedPolygon,
    NavigationConfig,
    NavigationTimeouts,
    ObservationRegionConfig,
    ParkingPriorConfig,
    PerceptionConfig,
    Point2D,
    Polygon2D,
    Pose2D,
    ProjectConfig,
    SignClass,
    SignFaceConfig,
    SignSiteConfig,
    StartConfig,
    ValidationIssue,
)


TEnum = TypeVar("TEnum", bound=IntEnum)


class ConfigValidationError(ValueError):
    """Raised when project YAML cannot be parsed into a valid ProjectConfig."""

    def __init__(self, issues: Iterable[ValidationIssue]):
        self.issues = tuple(issues)
        summary = "; ".join(f"{i.path}: {i.message}" for i in self.issues)
        super().__init__(summary or "invalid project configuration")


def _issue(code: str, path: str, message: str) -> ConfigValidationError:
    return ConfigValidationError((ValidationIssue(code, path, message),))


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise _issue("TYPE", path, "expected mapping")
    return value


def _sequence(value: Any, path: str) -> Sequence[Any]:
    if not isinstance(value, list):
        raise _issue("TYPE", path, "expected list")
    return value


def _keys(data: Mapping[str, Any], allowed: set[str], required: set[str], path: str) -> None:
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise _issue("UNKNOWN_KEY", path, f"unexpected key(s): {', '.join(unknown)}")
    missing = sorted(required - set(data))
    if missing:
        raise _issue("MISSING_KEY", path, f"missing key(s): {', '.join(missing)}")


def _str(value: Any, path: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise _issue("TYPE", path, "expected string")
    if not allow_empty and not value.strip():
        raise _issue("VALUE", path, "must not be empty")
    return value


def _bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise _issue("TYPE", path, "expected boolean")
    return value


def _int(value: Any, path: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise _issue("TYPE", path, "expected integer")
    if minimum is not None and value < minimum:
        raise _issue("RANGE", path, f"must be >= {minimum}")
    return value


def _float(
    value: Any,
    path: str,
    *,
    minimum: float | None = None,
    strictly_positive: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _issue("TYPE", path, "expected number")
    result = float(value)
    if not math.isfinite(result):
        raise _issue("NON_FINITE", path, "must be finite")
    if strictly_positive and result <= 0.0:
        raise _issue("RANGE", path, "must be > 0")
    if minimum is not None and result < minimum:
        raise _issue("RANGE", path, f"must be >= {minimum}")
    return result


def _optional_float(value: Any, path: str, *, minimum: float | None = None) -> float | None:
    if value is None:
        return None
    return _float(value, path, minimum=minimum)


def _optional_str(value: Any, path: str) -> str | None:
    if value is None:
        return None
    return _str(value, path)


def _enum(enum_type: Type[TEnum], value: Any, path: str) -> TEnum:
    if isinstance(value, str):
        try:
            return enum_type[value]
        except KeyError as exc:
            raise _issue("BAD_ENUM", path, f"unsupported {enum_type.__name__}: {value}") from exc
    if isinstance(value, bool) or not isinstance(value, int):
        raise _issue("BAD_ENUM", path, f"expected {enum_type.__name__} name or numeric value")
    try:
        return enum_type(value)
    except ValueError as exc:
        raise _issue("BAD_ENUM", path, f"unsupported {enum_type.__name__} numeric value: {value}") from exc


def _movement(value: Any, path: str) -> MovementMask:
    if isinstance(value, str):
        if value == "NONE":
            return MovementMask.NONE
        parts = value.split("|")
        mask = MovementMask.NONE
        for raw in parts:
            name = raw.strip()
            if name not in {"STRAIGHT", "LEFT", "RIGHT"}:
                raise _issue("BAD_ENUM", path, f"unsupported movement: {name}")
            mask |= MovementMask[name]
        return mask
    if isinstance(value, bool) or not isinstance(value, int):
        raise _issue("BAD_ENUM", path, "expected movement name or numeric mask")
    if value < 0 or value & ~int(MovementMask.STRAIGHT | MovementMask.LEFT | MovementMask.RIGHT):
        raise _issue("BAD_ENUM", path, f"unsupported movement mask: {value}")
    return MovementMask(value)


def _pose(data: Any, path: str) -> Pose2D:
    d = _mapping(data, path)
    allowed = required = {"x_m", "y_m", "yaw_rad"}
    _keys(d, allowed, required, path)
    return Pose2D(
        _float(d["x_m"], f"{path}.x_m"),
        _float(d["y_m"], f"{path}.y_m"),
        _float(d["yaw_rad"], f"{path}.yaw_rad"),
    )


def _point(data: Any, path: str) -> Point2D:
    values = _sequence(data, path)
    if len(values) != 2:
        raise _issue("POINT", path, "point must have exactly [x_m, y_m]")
    return Point2D(_float(values[0], f"{path}[0]"), _float(values[1], f"{path}[1]"))


def _signed_area(points: tuple[Point2D, ...]) -> float:
    return 0.5 * sum(
        a.x_m * b.y_m - b.x_m * a.y_m
        for a, b in zip(points, points[1:] + points[:1])
    )


def _polygon(data: Any, path: str) -> Polygon2D:
    values = _sequence(data, path)
    points = tuple(_point(v, f"{path}[{i}]") for i, v in enumerate(values))
    if len(points) < 3:
        raise _issue("MALFORMED_POLYGON", path, "polygon requires at least three points")
    if len(set(points)) < 3:
        raise _issue("MALFORMED_POLYGON", path, "polygon requires at least three distinct points")
    if abs(_signed_area(points)) <= 1e-12:
        raise _issue("MALFORMED_POLYGON", path, "polygon area must be non-zero")
    return Polygon2D(points)


def _named_polygons(data: Any, path: str) -> tuple[NamedPolygon, ...]:
    result: list[NamedPolygon] = []
    for i, raw in enumerate(_sequence(data, path)):
        p = f"{path}[{i}]"
        d = _mapping(raw, p)
        _keys(d, {"id", "polygon"}, {"id", "polygon"}, p)
        result.append(NamedPolygon(_str(d["id"], f"{p}.id"), _polygon(d["polygon"], f"{p}.polygon")))
    return tuple(result)


def _load_yaml(path: Path, label: str) -> Mapping[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream)
    except OSError as exc:
        raise _issue("FILE", label, f"cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise _issue("YAML", label, f"invalid YAML: {exc}") from exc
    if loaded is None:
        raise _issue("YAML", label, "file is empty")
    return _mapping(loaded, label)


def _parse_field(data: Mapping[str, Any]) -> FieldConfig:
    path = "field"
    allowed = required = {
        "schema_version", "frame_id", "field_size_m", "geometry_calibrated",
        "field_to_map", "footprint", "drivable_regions", "platforms", "anchors",
        "edges", "observation_regions", "sign_sites", "gates",
    }
    _keys(data, allowed, required, path)

    size = _sequence(data["field_size_m"], "field.field_size_m")
    if len(size) != 2:
        raise _issue("FIELD_SIZE", "field.field_size_m", "must contain [width_m, height_m]")

    transform_d = _mapping(data["field_to_map"], "field.field_to_map")
    _keys(transform_d, {"x_m", "y_m", "yaw_rad", "valid"}, {"x_m", "y_m", "yaw_rad", "valid"}, "field.field_to_map")
    transform = FieldTransform(
        _float(transform_d["x_m"], "field.field_to_map.x_m"),
        _float(transform_d["y_m"], "field.field_to_map.y_m"),
        _float(transform_d["yaw_rad"], "field.field_to_map.yaw_rad"),
        _bool(transform_d["valid"], "field.field_to_map.valid"),
    )

    anchors: list[DirectedAnchorConfig] = []
    for i, raw in enumerate(_sequence(data["anchors"], "field.anchors")):
        p = f"field.anchors[{i}]"
        d = _mapping(raw, p)
        _keys(d, {"id", "cell_id", "pose"}, {"id", "cell_id", "pose"}, p)
        anchors.append(DirectedAnchorConfig(_str(d["id"], f"{p}.id"), _str(d["cell_id"], f"{p}.cell_id"), _pose(d["pose"], f"{p}.pose")))

    edges: list[DirectedEdgeConfig] = []
    for i, raw in enumerate(_sequence(data["edges"], "field.edges")):
        p = f"field.edges[{i}]"
        d = _mapping(raw, p)
        allowed_edge = {"id", "from_anchor_id", "to_anchor_id", "movement", "path", "permitted_region_ids", "approach_id", "commitment_gate_id"}
        required_edge = {"id", "from_anchor_id", "to_anchor_id", "movement", "path", "permitted_region_ids"}
        _keys(d, allowed_edge, required_edge, p)
        path_values = tuple(_pose(v, f"{p}.path[{j}]") for j, v in enumerate(_sequence(d["path"], f"{p}.path")))
        if len(path_values) < 2:
            raise _issue("PATH", f"{p}.path", "motion path requires at least two poses")
        region_ids = tuple(_str(v, f"{p}.permitted_region_ids[{j}]") for j, v in enumerate(_sequence(d["permitted_region_ids"], f"{p}.permitted_region_ids")))
        if not region_ids:
            raise _issue("PATH", f"{p}.permitted_region_ids", "must contain at least one permitted region")
        edges.append(DirectedEdgeConfig(
            edge_id=_str(d["id"], f"{p}.id"),
            from_anchor_id=_str(d["from_anchor_id"], f"{p}.from_anchor_id"),
            to_anchor_id=_str(d["to_anchor_id"], f"{p}.to_anchor_id"),
            movement=_movement(d["movement"], f"{p}.movement"),
            path=MetricPath(path_values),
            permitted_region_ids=region_ids,
            approach_id=_str(d.get("approach_id", ""), f"{p}.approach_id", allow_empty=True),
            commitment_gate_id=_optional_str(d.get("commitment_gate_id"), f"{p}.commitment_gate_id"),
        ))

    observations: list[ObservationRegionConfig] = []
    for i, raw in enumerate(_sequence(data["observation_regions"], "field.observation_regions")):
        p = f"field.observation_regions[{i}]"
        d = _mapping(raw, p)
        keys = {"id", "polygon", "anchor_id", "nominal_heading_rad", "heading_tolerance_rad"}
        _keys(d, keys, keys, p)
        observations.append(ObservationRegionConfig(
            _str(d["id"], f"{p}.id"),
            _polygon(d["polygon"], f"{p}.polygon"),
            _str(d["anchor_id"], f"{p}.anchor_id"),
            _float(d["nominal_heading_rad"], f"{p}.nominal_heading_rad"),
            _float(d["heading_tolerance_rad"], f"{p}.heading_tolerance_rad", minimum=0.0),
        ))

    sites: list[SignSiteConfig] = []
    for i, raw in enumerate(_sequence(data["sign_sites"], "field.sign_sites")):
        p = f"field.sign_sites[{i}]"
        d = _mapping(raw, p)
        _keys(d, {"id", "position", "faces"}, {"id", "position", "faces"}, p)
        faces: list[SignFaceConfig] = []
        for j, face_raw in enumerate(_sequence(d["faces"], f"{p}.faces")):
            fp = f"{p}.faces[{j}]"
            fd = _mapping(face_raw, fp)
            _keys(fd, {"id", "yaw_rad", "controlled_approach_id"}, {"id", "yaw_rad"}, fp)
            faces.append(SignFaceConfig(
                _str(fd["id"], f"{fp}.id"),
                _float(fd["yaw_rad"], f"{fp}.yaw_rad"),
                _str(fd.get("controlled_approach_id", ""), f"{fp}.controlled_approach_id", allow_empty=True),
            ))
        sites.append(SignSiteConfig(_str(d["id"], f"{p}.id"), _point(d["position"], f"{p}.position"), tuple(faces)))

    gates: list[GateConfig] = []
    for i, raw in enumerate(_sequence(data["gates"], "field.gates")):
        p = f"field.gates[{i}]"
        d = _mapping(raw, p)
        _keys(d, {"id", "pose"}, {"id", "pose"}, p)
        gates.append(GateConfig(_str(d["id"], f"{p}.id"), _pose(d["pose"], f"{p}.pose")))

    return FieldConfig(
        schema_version=_int(data["schema_version"], "field.schema_version", minimum=1),
        frame_id=_str(data["frame_id"], "field.frame_id"),
        width_m=_float(size[0], "field.field_size_m[0]", strictly_positive=True),
        height_m=_float(size[1], "field.field_size_m[1]", strictly_positive=True),
        geometry_calibrated=_bool(data["geometry_calibrated"], "field.geometry_calibrated"),
        field_to_map=transform,
        footprint=_polygon(data["footprint"], "field.footprint"),
        drivable_regions=_named_polygons(data["drivable_regions"], "field.drivable_regions"),
        platforms=_named_polygons(data["platforms"], "field.platforms"),
        anchors=tuple(anchors),
        edges=tuple(edges),
        observation_regions=tuple(observations),
        sign_sites=tuple(sites),
        gates=tuple(gates),
    )


def _parse_event(data: Mapping[str, Any]) -> EventConfig:
    path = "event"
    allowed = required = {
        "schema_version", "required_unique_bus_stops", "bus_stop_dwell_s",
        "parking_settle_time_s", "semantic_bootstrap", "allow_reverse", "allow_uturn",
        "auto_start", "official_attempt_limit_s", "start", "parking_prior",
        "event_inputs_confirmed", "geometry_calibrated",
    }
    _keys(data, allowed, required, path)
    start_d = _mapping(data["start"], "event.start")
    _keys(start_d, {"cell_id", "map_pose", "outgoing_edge_id"}, {"cell_id", "map_pose", "outgoing_edge_id"}, "event.start")
    start_pose = None if start_d["map_pose"] is None else _pose(start_d["map_pose"], "event.start.map_pose")
    prior_d = _mapping(data["parking_prior"], "event.parking_prior")
    _keys(prior_d, {"enabled", "mode", "observation_id"}, {"enabled", "mode", "observation_id"}, "event.parking_prior")
    bootstrap = _str(data["semantic_bootstrap"], "event.semantic_bootstrap")
    if bootstrap not in {"empty"}:
        raise _issue("BAD_ENUM", "event.semantic_bootstrap", f"unsupported value: {bootstrap}")
    dwell = _float(data["bus_stop_dwell_s"], "event.bus_stop_dwell_s", minimum=0.0)
    if dwell < 2.0:
        raise _issue("RULE", "event.bus_stop_dwell_s", "must be at least 2.0 seconds")
    return EventConfig(
        schema_version=_int(data["schema_version"], "event.schema_version", minimum=1),
        required_unique_bus_stops=_int(data["required_unique_bus_stops"], "event.required_unique_bus_stops", minimum=1),
        bus_stop_dwell_s=dwell,
        parking_settle_time_s=_float(data["parking_settle_time_s"], "event.parking_settle_time_s", minimum=0.0),
        semantic_bootstrap=bootstrap,
        allow_reverse=_bool(data["allow_reverse"], "event.allow_reverse"),
        allow_uturn=_bool(data["allow_uturn"], "event.allow_uturn"),
        auto_start=_bool(data["auto_start"], "event.auto_start"),
        official_attempt_limit_s=_optional_float(data["official_attempt_limit_s"], "event.official_attempt_limit_s", minimum=0.0),
        start=StartConfig(
            _optional_str(start_d["cell_id"], "event.start.cell_id"),
            start_pose,
            _optional_str(start_d["outgoing_edge_id"], "event.start.outgoing_edge_id"),
        ),
        parking_prior=ParkingPriorConfig(
            _bool(prior_d["enabled"], "event.parking_prior.enabled"),
            _str(prior_d["mode"], "event.parking_prior.mode"),
            _optional_str(prior_d["observation_id"], "event.parking_prior.observation_id"),
        ),
        event_inputs_confirmed=_bool(data["event_inputs_confirmed"], "event.event_inputs_confirmed"),
        geometry_calibrated=_bool(data["geometry_calibrated"], "event.geometry_calibrated"),
    )


def _parse_perception(data: Mapping[str, Any]) -> PerceptionConfig:
    path = "perception"
    allowed = required = {"schema_version", "camera_frame", "calibration_id", "label_map", "confidence_thresholds"}
    _keys(data, allowed, required, path)
    labels_d = _mapping(data["label_map"], "perception.label_map")
    labels: list[tuple[str, SignClass]] = []
    for label, raw_class in labels_d.items():
        label_s = _str(label, "perception.label_map.<label>")
        sign_class = _enum(SignClass, raw_class, f"perception.label_map.{label_s}")
        if sign_class == SignClass.UNKNOWN:
            raise _issue("UNSUPPORTED_LABEL", f"perception.label_map.{label_s}", "labels may not map to UNKNOWN")
        labels.append((label_s, sign_class))

    thresholds_d = _mapping(data["confidence_thresholds"], "perception.confidence_thresholds")
    thresholds: list[tuple[SignClass, float]] = []
    for class_name, raw_threshold in thresholds_d.items():
        sign_class = _enum(SignClass, class_name, f"perception.confidence_thresholds.{class_name}")
        if sign_class == SignClass.UNKNOWN:
            raise _issue("BAD_ENUM", f"perception.confidence_thresholds.{class_name}", "UNKNOWN has no acceptance threshold")
        threshold = _float(raw_threshold, f"perception.confidence_thresholds.{class_name}", minimum=0.0)
        if threshold > 1.0:
            raise _issue("RANGE", f"perception.confidence_thresholds.{class_name}", "must be <= 1.0")
        thresholds.append((sign_class, threshold))

    return PerceptionConfig(
        schema_version=_int(data["schema_version"], "perception.schema_version", minimum=1),
        camera_frame=_str(data["camera_frame"], "perception.camera_frame"),
        calibration_id=_str(data["calibration_id"], "perception.calibration_id"),
        label_map=tuple(sorted(labels, key=lambda item: item[0])),
        confidence_thresholds=tuple(sorted(thresholds, key=lambda item: int(item[0]))),
    )


def _parse_navigation(data: Mapping[str, Any]) -> NavigationConfig:
    path = "navigation"
    allowed = required = {
        "schema_version", "frames", "max_speed_mps", "max_yaw_rate_radps",
        "xy_tolerance_m", "yaw_tolerance_rad", "stationary_linear_mps",
        "stationary_angular_radps", "observation_attempts", "segment_retry_budget", "timeouts",
    }
    _keys(data, allowed, required, path)
    frames = _mapping(data["frames"], "navigation.frames")
    _keys(frames, {"map", "odom", "base"}, {"map", "odom", "base"}, "navigation.frames")
    timeouts_d = _mapping(data["timeouts"], "navigation.timeouts")
    timeout_keys = {
        "permission_max_age_s", "map_max_age_s", "pose_max_age_s", "detection_max_age_s",
        "tf_lookup_s", "observation_action_s", "action_accept_s", "action_cancel_s", "no_progress_s",
    }
    _keys(timeouts_d, timeout_keys, timeout_keys, "navigation.timeouts")
    timeouts = NavigationTimeouts(**{
        key: _float(timeouts_d[key], f"navigation.timeouts.{key}", minimum=0.0)
        for key in sorted(timeout_keys)
    })
    return NavigationConfig(
        schema_version=_int(data["schema_version"], "navigation.schema_version", minimum=1),
        map_frame=_str(frames["map"], "navigation.frames.map"),
        odom_frame=_str(frames["odom"], "navigation.frames.odom"),
        base_frame=_str(frames["base"], "navigation.frames.base"),
        max_speed_mps=_float(data["max_speed_mps"], "navigation.max_speed_mps", strictly_positive=True),
        max_yaw_rate_radps=_float(data["max_yaw_rate_radps"], "navigation.max_yaw_rate_radps", strictly_positive=True),
        xy_tolerance_m=_float(data["xy_tolerance_m"], "navigation.xy_tolerance_m", minimum=0.0),
        yaw_tolerance_rad=_float(data["yaw_tolerance_rad"], "navigation.yaw_tolerance_rad", minimum=0.0),
        stationary_linear_mps=_float(data["stationary_linear_mps"], "navigation.stationary_linear_mps", minimum=0.0),
        stationary_angular_radps=_float(data["stationary_angular_radps"], "navigation.stationary_angular_radps", minimum=0.0),
        observation_attempts=_int(data["observation_attempts"], "navigation.observation_attempts", minimum=1),
        segment_retry_budget=_int(data["segment_retry_budget"], "navigation.segment_retry_budget", minimum=0),
        timeouts=timeouts,
    )


def _duplicates(values: Iterable[str]) -> tuple[str, ...]:
    seen: set[str] = set()
    dupes: set[str] = set()
    for value in values:
        if value in seen:
            dupes.add(value)
        seen.add(value)
    return tuple(sorted(dupes))


def validate_project_config(config: ProjectConfig) -> tuple[ValidationIssue, ...]:
    """Return all cross-record validation issues for an already parsed config."""
    issues: list[ValidationIssue] = []
    field = config.field

    def add(code: str, path: str, message: str) -> None:
        issues.append(ValidationIssue(code, path, message))

    for label, values in (
        ("field.drivable_regions", (x.polygon_id for x in field.drivable_regions)),
        ("field.platforms", (x.polygon_id for x in field.platforms)),
        ("field.anchors", (x.anchor_id for x in field.anchors)),
        ("field.edges", (x.edge_id for x in field.edges)),
        ("field.observation_regions", (x.observation_id for x in field.observation_regions)),
        ("field.sign_sites", (x.site_id for x in field.sign_sites)),
        ("field.gates", (x.gate_id for x in field.gates)),
    ):
        for duplicate in _duplicates(values):
            add("DUPLICATE_ID", label, f"duplicate ID: {duplicate}")

    anchor_by_id = {a.anchor_id: a for a in field.anchors}
    region_ids = {r.polygon_id for r in field.drivable_regions}
    gate_ids = {g.gate_id for g in field.gates}
    observation_ids = {o.observation_id for o in field.observation_regions}
    edge_by_id = {e.edge_id: e for e in field.edges}

    for edge in field.edges:
        if edge.from_anchor_id not in anchor_by_id:
            add("REFERENCE", f"field.edges.{edge.edge_id}.from_anchor_id", f"unknown anchor: {edge.from_anchor_id}")
        if edge.to_anchor_id not in anchor_by_id:
            add("REFERENCE", f"field.edges.{edge.edge_id}.to_anchor_id", f"unknown anchor: {edge.to_anchor_id}")
        for region_id in edge.permitted_region_ids:
            if region_id not in region_ids:
                add("REFERENCE", f"field.edges.{edge.edge_id}.permitted_region_ids", f"unknown region: {region_id}")
        if edge.commitment_gate_id is not None and edge.commitment_gate_id not in gate_ids:
            add("REFERENCE", f"field.edges.{edge.edge_id}.commitment_gate_id", f"unknown gate: {edge.commitment_gate_id}")
        if edge.movement not in {MovementMask.NONE, MovementMask.STRAIGHT, MovementMask.LEFT, MovementMask.RIGHT}:
            add("MOVEMENT", f"field.edges.{edge.edge_id}.movement", "an edge primitive must represent one movement bit or NONE")

    for obs in field.observation_regions:
        if obs.anchor_id not in anchor_by_id:
            add("REFERENCE", f"field.observation_regions.{obs.observation_id}.anchor_id", f"unknown anchor: {obs.anchor_id}")

    for site in field.sign_sites:
        face_ids = [face.face_id for face in site.faces]
        for duplicate in _duplicates(face_ids):
            add("DUPLICATE_ID", f"field.sign_sites.{site.site_id}.faces", f"duplicate face ID: {duplicate}")

    start = config.event.start
    if start.outgoing_edge_id is not None:
        edge = edge_by_id.get(start.outgoing_edge_id)
        if edge is None:
            add("START_EDGE", "event.start.outgoing_edge_id", f"unknown edge: {start.outgoing_edge_id}")
        elif start.cell_id is not None:
            anchor = anchor_by_id.get(edge.from_anchor_id)
            if anchor is None or anchor.cell_id != start.cell_id:
                add("START_EDGE", "event.start.outgoing_edge_id", "edge does not leave the configured start cell")

    if start.cell_id is None or start.map_pose is None or start.outgoing_edge_id is None:
        if config.event.event_inputs_confirmed:
            add("START_STATE", "event.start", "confirmed event inputs require cell_id, map_pose and outgoing_edge_id")

    if config.event.geometry_calibrated and not field.geometry_calibrated:
        add("CALIBRATION", "event.geometry_calibrated", "event cannot claim calibrated geometry when field geometry is uncalibrated")
    if field.geometry_calibrated and not field.field_to_map.valid:
        add("CALIBRATION", "field.field_to_map.valid", "calibrated geometry requires a valid field-to-map transform")

    if config.event.parking_prior.enabled and config.event.parking_prior.observation_id is not None:
        if config.event.parking_prior.observation_id not in observation_ids:
            # Optional prior is intentionally non-fatal at startup in later runtime logic,
            # but configuration validation still reports the mismatch explicitly.
            add("OPTIONAL_REFERENCE", "event.parking_prior.observation_id", "unknown observation region; runtime must mark prior INVALID and use general search")

    labels = {label for label, _ in config.perception.label_map}
    if not labels:
        add("LABEL_MAP", "perception.label_map", "at least one model label mapping is required")
    threshold_classes = {cls for cls, _ in config.perception.confidence_thresholds}
    for _, cls in config.perception.label_map:
        if cls not in threshold_classes:
            add("THRESHOLD", f"perception.confidence_thresholds.{cls.name}", "missing threshold for mapped class")

    if config.navigation.map_frame != field.frame_id:
        add("FRAME", "navigation.frames.map", f"must match field frame_id '{field.frame_id}'")

    return tuple(issues)


def load_project_config(paths: ConfigPaths) -> ProjectConfig:
    """Load four strict project YAML files and return a validated immutable config.

    Parsing/shape errors and cross-file validation errors raise ConfigValidationError.
    """
    project = ProjectConfig(
        contract_version=CONTRACT_VERSION,
        field=_parse_field(_load_yaml(Path(paths.field), "field")),
        event=_parse_event(_load_yaml(Path(paths.event), "event")),
        perception=_parse_perception(_load_yaml(Path(paths.perception), "perception")),
        navigation=_parse_navigation(_load_yaml(Path(paths.navigation), "navigation")),
    )
    issues = validate_project_config(project)
    fatal = tuple(issue for issue in issues if issue.code != "OPTIONAL_REFERENCE")
    if fatal:
        raise ConfigValidationError(fatal)
    return project


def _canonical(value: Any) -> Any:
    if isinstance(value, (IntEnum, IntFlag)):
        return int(value)
    if isinstance(value, dict):
        return {str(key): _canonical(val) for key, val in sorted(value.items(), key=lambda item: str(item[0]))}
    if isinstance(value, (tuple, list)):
        return [_canonical(item) for item in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("non-finite value cannot be canonicalized")
        # 17 significant digits round-trip an IEEE-754 binary64 value.
        return format(value, ".17g")
    return value


def canonical_field_digest(config: FieldConfig) -> str:
    """Return deterministic SHA-256 digest of the validated field configuration."""
    payload = _canonical(asdict(config))
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
