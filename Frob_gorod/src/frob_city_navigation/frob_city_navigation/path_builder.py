"""Immutable metric segment construction and validation for WP08."""

from __future__ import annotations

import hashlib
import json
import math

from .field_model import FieldModel
from .geometry import (
    path_length,
    project_point_to_path,
    sample_path,
    swept_footprint_intersects_forbidden,
    validate_swept_footprint,
)
from .models import (
    ApproachResolution,
    GeometryMargins,
    GraphTransition,
    MapSnapshot,
    MetricPath,
    MovementMask,
    NavigationConfig,
    Point2D,
    Pose2D,
    PoseSample,
    RouteDependency,
    RouteSegment,
    SegmentBuildResult,
    SegmentBuildStatus,
    SegmentKind,
    SegmentValidation,
    SegmentValidationStatus,
)


def _snapshot_version(snapshot: MapSnapshot, dependency_id: str) -> int | None:
    if ":" not in dependency_id:
        return None
    prefix, entity_id = dependency_id.split(":", 1)
    if prefix == "approach":
        record = next((x for x in snapshot.approaches if x.approach_id == entity_id), None)
    elif prefix == "edge":
        record = next((x for x in snapshot.edges if x.edge_id == entity_id), None)
    elif prefix == "target":
        record = next((x for x in snapshot.targets if x.target_id == entity_id), None)
    elif prefix == "coverage":
        record = next((x for x in snapshot.coverage if x.coverage_id == entity_id), None)
    else:
        return None
    return None if record is None else record.version


def _dependencies_current(deps: tuple[RouteDependency, ...], snapshot: MapSnapshot) -> bool:
    return all(_snapshot_version(snapshot, d.dependency_id) == d.version for d in deps)


def _trim_path_from_pose(path: MetricPath, pose: Pose2D, tolerance_m: float) -> MetricPath | None:
    projection = project_point_to_path(Point2D(pose.x_m, pose.y_m), path)
    if projection.distance_m > tolerance_m:
        return None
    # Keep the current measured pose as the first point, then all primitive
    # points downstream of the projected segment. This avoids commanding the
    # robot back to the primitive's nominal start.
    downstream = list(path.poses[projection.segment_index + 1 :])
    if not downstream:
        downstream = [path.poses[-1]]
    first = Pose2D(pose.x_m, pose.y_m, projection.projected_pose.yaw_rad)
    if math.hypot(downstream[0].x_m - first.x_m, downstream[0].y_m - first.y_m) < 1e-9:
        return MetricPath((first,)) if len(downstream) == 0 else MetricPath((first, *downstream[1:]))
    return MetricPath((first, *downstream))


def _terminal_limit_respected(
    path: MetricPath,
    footprint,
    terminal_limit: Pose2D,
    extra_margin_m: float = 0.0,
) -> bool:
    nx = math.cos(terminal_limit.yaw_rad)
    ny = math.sin(terminal_limit.yaw_rad)
    c_limit = terminal_limit.x_m * nx + terminal_limit.y_m * ny
    for pose in sample_path(path):
        c = math.cos(pose.yaw_rad)
        s = math.sin(pose.yaw_rad)
        for p in footprint.points:
            x = pose.x_m + c * p.x_m - s * p.y_m
            y = pose.y_m + s * p.x_m + c * p.y_m
            if x * nx + y * ny > c_limit - extra_margin_m + 1e-8:
                return False
    return True


def _segment_id(transition: GraphTransition, path: MetricPath) -> str:
    payload = {
        "transition": transition.transition_id,
        "path": [[round(p.x_m, 9), round(p.y_m, 9), round(p.yaw_rad, 9)] for p in path.poses],
        "deps": [[d.dependency_id, d.version] for d in transition.dependencies],
        "terminal": [
            transition.terminal_limit.x_m if transition.terminal_limit else None,
            transition.terminal_limit.y_m if transition.terminal_limit else None,
            transition.terminal_limit.yaw_rad if transition.terminal_limit else None,
        ],
        "target": transition.target_id,
    }
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()[:20]
    return f"seg-{digest}"


class PathBuilder:
    def __init__(
        self,
        field: FieldModel,
        navigation: NavigationConfig,
        margins: GeometryMargins | None = None,
    ):
        self._field = field
        self._navigation = navigation
        self._margins = margins or GeometryMargins(0.0, 0.0, 0.0)

    def build_segment(
        self,
        transition: GraphTransition,
        snapshot: MapSnapshot,
        pose: PoseSample,
    ) -> SegmentBuildResult:
        if not transition.executable:
            return SegmentBuildResult(
                SegmentBuildStatus.INVALID_TRANSITION, None, "transition is not executable"
            )
        if not _dependencies_current(transition.dependencies, snapshot):
            return SegmentBuildResult(
                SegmentBuildStatus.DEPENDENCY_INVALID, None, "semantic dependency changed"
            )

        join_tolerance = max(self._navigation.xy_tolerance_m * 2.0, 0.10)
        pose_field = self._field.map_to_field_pose(pose.pose)
        path = _trim_path_from_pose(transition.path, pose_field, join_tolerance)
        if path is None or len(path.poses) < 2:
            return SegmentBuildResult(
                SegmentBuildStatus.START_JOIN_INVALID, None, "robot pose cannot join primitive"
            )

        geometry = validate_swept_footprint(
            path, transition.permitted_regions, self._field.footprint, self._margins
        )
        if not geometry.valid:
            return SegmentBuildResult(
                SegmentBuildStatus.GEOMETRY_INVALID, None, geometry.reason
            )
        if swept_footprint_intersects_forbidden(
            path, self._field.footprint, self._field.platform_polygons(), self._margins
        ):
            return SegmentBuildResult(
                SegmentBuildStatus.GEOMETRY_INVALID, None, "swept footprint intersects static platform"
            )

        terminal_limit = transition.terminal_limit or path.poses[-1]
        # HOLD terminal limits at an anchor are descriptive rather than an
        # upstream stop plane. Hard plane enforcement is needed for service
        # segments and any explicitly constrained terminal before an obstacle.
        if transition.segment_kind is SegmentKind.SERVICE_APPROACH:
            margin = self._margins.localization_margin_m + self._margins.tracking_margin_m
            if not _terminal_limit_respected(path, self._field.footprint, terminal_limit, margin):
                return SegmentBuildResult(
                    SegmentBuildStatus.TERMINAL_LIMIT_VIOLATION,
                    None,
                    "path/footprint crosses terminal limit",
                )

        duration = path_length(path) / max(self._navigation.max_speed_mps, 1e-6)
        segment = RouteSegment(
            segment_id=_segment_id(transition, path),
            kind=transition.segment_kind,
            from_anchor_id=transition.from_state.anchor_id,
            to_anchor_id=transition.to_state.anchor_id,
            approach_id=transition.approach_id,
            selected_movement=transition.movement if transition.segment_kind is SegmentKind.JUNCTION else MovementMask.NONE,
            path=path,
            allowed_regions=transition.permitted_regions,
            dependencies=transition.dependencies,
            traversed_edge_ids=(transition.edge_id,) if transition.edge_id else (),
            terminal_kind=transition.terminal_kind,
            terminal_id=transition.target_id or transition.observation_id or transition.to_state.anchor_id,
            terminal_limit=terminal_limit,
            commitment_gate=transition.commitment_gate,
            max_speed_mps=self._navigation.max_speed_mps,
            max_yaw_rate_radps=self._navigation.max_yaw_rate_radps,
            xy_tolerance_m=self._navigation.xy_tolerance_m,
            yaw_tolerance_rad=self._navigation.yaw_tolerance_rad,
            estimated_duration_s=duration,
        )
        return SegmentBuildResult(SegmentBuildStatus.BUILT, segment, "")


def validate_segment(
    segment: RouteSegment,
    snapshot: MapSnapshot,
    pose: PoseSample,
    field: FieldModel,
    margins: GeometryMargins | None = None,
) -> SegmentValidation:
    """Revalidate an immutable segment against current pose and semantics."""
    if not snapshot.ready:
        return SegmentValidation(
            SegmentValidationStatus.NOT_READY, snapshot.revision, pose.pose, reason="map not ready"
        )
    if not _dependencies_current(segment.dependencies, snapshot):
        return SegmentValidation(
            SegmentValidationStatus.INVALID,
            snapshot.revision,
            pose.pose,
            reason="semantic dependency changed",
        )

    try:
        primitive = field.movement_primitive(segment.traversed_edge_ids[0])
    except (KeyError, IndexError):
        return SegmentValidation(
            SegmentValidationStatus.INVALID, snapshot.revision, pose.pose, reason="unknown traversed edge"
        )

    if segment.kind is SegmentKind.JUNCTION:
        if segment.approach_id != primitive.approach_id or segment.selected_movement != primitive.movement:
            return SegmentValidation(
                SegmentValidationStatus.INVALID,
                snapshot.revision,
                pose.pose,
                reason="segment movement does not match surveyed primitive",
            )
        approach = next((x for x in snapshot.approaches if x.approach_id == segment.approach_id), None)
        if approach is None or approach.state not in (
            ApproachResolution.RESOLVED_CLEAR,
            ApproachResolution.RESOLVED_SIGNED,
        ):
            observation_id = ""
            regs = field.observation_regions_at_anchor(segment.from_anchor_id)
            if regs:
                observation_id = regs[0].observation_id
            return SegmentValidation(
                SegmentValidationStatus.NEED_OBSERVATION,
                snapshot.revision,
                pose.pose,
                observation_id=observation_id,
                reason="approach is no longer resolved",
            )
        if not (approach.allowed_movements & segment.selected_movement):
            return SegmentValidation(
                SegmentValidationStatus.INVALID,
                snapshot.revision,
                pose.pose,
                reason="selected movement is no longer legal",
            )

    pose_field = field.map_to_field_pose(pose.pose)
    join = project_point_to_path(Point2D(pose_field.x_m, pose_field.y_m), segment.path)
    if join.distance_m > max(segment.xy_tolerance_m * 2.0, 0.10):
        return SegmentValidation(
            SegmentValidationStatus.INVALID,
            snapshot.revision,
            pose.pose,
            reason="robot moved too far from segment start/path",
        )

    active_margins = margins or GeometryMargins(0.0, 0.0, 0.0)
    geometry = validate_swept_footprint(
        segment.path,
        segment.allowed_regions,
        field.footprint,
        active_margins,
    )
    if not geometry.valid:
        return SegmentValidation(
            SegmentValidationStatus.INVALID, snapshot.revision, pose.pose, reason=geometry.reason
        )
    if swept_footprint_intersects_forbidden(
        segment.path, field.footprint, field.platform_polygons(), active_margins
    ):
        return SegmentValidation(
            SegmentValidationStatus.INVALID,
            snapshot.revision,
            pose.pose,
            reason="swept footprint intersects static platform",
        )

    if segment.kind is SegmentKind.SERVICE_APPROACH and not _terminal_limit_respected(
        segment.path, field.footprint, segment.terminal_limit
    ):
        return SegmentValidation(
            SegmentValidationStatus.INVALID,
            snapshot.revision,
            pose.pose,
            reason="terminal limit would be crossed",
        )

    return SegmentValidation(SegmentValidationStatus.VALID, snapshot.revision, pose.pose)
