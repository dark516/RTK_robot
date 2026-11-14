"""Direction-aware road projection and service geometry for Frob City WP04."""

from __future__ import annotations

import math

from .field_model import FieldModel
from .geometry import (
    pose_at_station,
    project_point_to_path,
    signed_angle_error,
)
from .models import (
    Point2D,
    PoseProjection,
    PoseSample,
    ProjectionResult,
    ProjectionStatus,
    ServiceGeometryResult,
    ServiceGeometryStatus,
    SignSite,
)

# Initial software thresholds.  They are intentionally conservative and remain
# subject to physical calibration in WP15; G04 is a LOCAL software gate only.
_MAX_CROSS_TRACK_M = 0.35
_MAX_HEADING_ERROR_RAD = math.radians(65.0)
_HYSTERESIS_CROSS_TRACK_M = 0.45
_HYSTERESIS_HEADING_ERROR_RAD = math.radians(80.0)
_HYSTERESIS_SCORE_SLACK = 0.12
_AMBIGUITY_SCORE_EPS = 0.025
_SIGN_MAX_LATERAL_DISTANCE_M = 0.80


def _project_candidate(field: FieldModel, movement_id: str, pose_field):
    movement = field.movement_primitive(movement_id)
    raw = project_point_to_path(Point2D(pose_field.x_m, pose_field.y_m), movement.path)
    heading_error = abs(signed_angle_error(pose_field.yaw_rad, raw.projected_pose.yaw_rad))
    projection = PoseProjection(
        movement_id=movement_id,
        station_m=raw.station_m,
        path_length_m=raw.path_length_m,
        projected_pose=raw.projected_pose,
        cross_track_error_m=raw.distance_m,
        heading_error_rad=heading_error,
    )
    score = raw.distance_m + 0.15 * heading_error
    return projection, score


def project_pose(
    field: FieldModel,
    sample: PoseSample,
    previous: PoseProjection | None,
) -> ProjectionResult:
    """Project a measured map pose onto one directed motion primitive.

    Heading is part of the match; a geometrically coincident reverse edge is
    rejected.  Near intersections, a still-plausible previous primitive is
    retained with hysteresis to avoid edge flapping.
    """
    pose_field = field.map_to_field_pose(sample.pose)
    point_field = Point2D(pose_field.x_m, pose_field.y_m)

    if not field.is_inside_field_bounds(point_field, tolerance_m=0.05):
        return ProjectionResult(
            ProjectionStatus.OUT_OF_FIELD,
            None,
            (),
            "pose lies outside configured field bounds",
        )

    strict: list[tuple[float, PoseProjection]] = []
    relaxed_previous: tuple[float, PoseProjection] | None = None

    for movement_id in field.movement_ids:
        projection, score = _project_candidate(field, movement_id, pose_field)
        if (
            projection.cross_track_error_m <= _MAX_CROSS_TRACK_M
            and projection.heading_error_rad <= _MAX_HEADING_ERROR_RAD
        ):
            strict.append((score, projection))

        if previous is not None and movement_id == previous.movement_id:
            if (
                projection.cross_track_error_m <= _HYSTERESIS_CROSS_TRACK_M
                and projection.heading_error_rad <= _HYSTERESIS_HEADING_ERROR_RAD
            ):
                relaxed_previous = (score, projection)

    strict.sort(key=lambda item: (item[0], item[1].movement_id))

    if not strict:
        if relaxed_previous is not None:
            return ProjectionResult(
                ProjectionStatus.MATCHED,
                relaxed_previous[1],
                (relaxed_previous[1].movement_id,),
                "retained previous movement by intersection hysteresis",
            )
        return ProjectionResult(
            ProjectionStatus.NO_MATCH,
            None,
            (),
            "no directed road matches both position and heading",
        )

    # Prefer the previous road when it remains nearly as plausible as the best
    # current geometric candidate.  This is the intersection hysteresis rule.
    if relaxed_previous is not None:
        best_score = strict[0][0]
        if relaxed_previous[0] <= best_score + _HYSTERESIS_SCORE_SLACK:
            p = relaxed_previous[1]
            return ProjectionResult(
                ProjectionStatus.MATCHED,
                p,
                tuple(x[1].movement_id for x in strict),
                "retained previous movement by intersection hysteresis",
            )

    if len(strict) >= 2 and abs(strict[1][0] - strict[0][0]) <= _AMBIGUITY_SCORE_EPS:
        return ProjectionResult(
            ProjectionStatus.AMBIGUOUS,
            None,
            tuple(x[1].movement_id for x in strict),
            "multiple directed roads are equally plausible",
        )

    best = strict[0][1]
    return ProjectionResult(
        ProjectionStatus.MATCHED,
        best,
        tuple(x[1].movement_id for x in strict),
        "",
    )


def construct_service_geometry(
    field: FieldModel,
    sign_site: SignSite,
    directed_road_id: str,
) -> ServiceGeometryResult:
    """Construct a conservative base pose *before* a roadside sign.

    The terminal plane is perpendicular to the directed road at the sign's
    projected path station.  With no physically calibrated extra front
    clearance yet, the base service pose is placed upstream by exactly the
    measured/configured forward footprint extent.  ``calibration_required``
    therefore remains true for the nominal uncalibrated field until WP15.
    """
    try:
        movement = field.movement_primitive(directed_road_id)
    except KeyError:
        return ServiceGeometryResult(
            ServiceGeometryStatus.UNKNOWN_ROAD,
            False,
            directed_road_id,
            0.0,
            0.0,
            None,
            None,
            0.0,
            not field.geometry_calibrated,
            "unknown directed road",
        )

    projection = project_point_to_path(sign_site.position, movement.path)
    if projection.distance_m > _SIGN_MAX_LATERAL_DISTANCE_M:
        return ServiceGeometryResult(
            ServiceGeometryStatus.SIGN_NOT_PROJECTABLE,
            False,
            directed_road_id,
            projection.station_m,
            0.0,
            None,
            None,
            0.0,
            not field.geometry_calibrated,
            "sign site is too far from the directed road",
        )

    front_extent = max(p.x_m for p in field.footprint.points)
    if front_extent < 0.0:
        front_extent = 0.0
    service_station = projection.station_m - front_extent
    if service_station < -1e-9:
        return ServiceGeometryResult(
            ServiceGeometryStatus.INSUFFICIENT_UPSTREAM_ROOM,
            False,
            directed_road_id,
            projection.station_m,
            service_station,
            None,
            None,
            front_extent,
            not field.geometry_calibrated,
            "not enough upstream path for the robot front to remain before the sign plane",
        )

    service_pose = pose_at_station(movement.path, max(0.0, service_station))
    terminal_limit = pose_at_station(movement.path, projection.station_m)
    return ServiceGeometryResult(
        ServiceGeometryStatus.VALID,
        True,
        directed_road_id,
        projection.station_m,
        max(0.0, service_station),
        service_pose,
        terminal_limit,
        front_extent,
        not field.geometry_calibrated,
        "",
    )
