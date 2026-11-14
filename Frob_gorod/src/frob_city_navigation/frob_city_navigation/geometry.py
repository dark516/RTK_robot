"""Pure planar geometry helpers for Frob City (WP04).

No ROS imports are permitted here.  All dimensions are metres/radians.
Shapely is used only for robust polygon/swept-footprint operations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from shapely.affinity import rotate, translate
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.ops import unary_union

from .models import (
    FieldTransform,
    GeometryMargins,
    GeometryResult,
    GeometryStatus,
    MetricPath,
    Point2D,
    Polygon2D,
    Pose2D,
)

_EPS = 1e-9


def normalize_angle(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi)."""
    wrapped = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    # Keep the representation deterministic at the discontinuity.
    return -math.pi if abs(wrapped - math.pi) < 1e-12 else wrapped


def signed_angle_error(target_rad: float, reference_rad: float) -> float:
    return normalize_angle(target_rad - reference_rad)


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def field_to_map_point(point: Point2D, transform: FieldTransform) -> Point2D:
    rx, ry = _rotate_xy(point.x_m, point.y_m, transform.yaw_rad)
    return Point2D(rx + transform.x_m, ry + transform.y_m)


def map_to_field_point(point: Point2D, transform: FieldTransform) -> Point2D:
    dx = point.x_m - transform.x_m
    dy = point.y_m - transform.y_m
    rx, ry = _rotate_xy(dx, dy, -transform.yaw_rad)
    return Point2D(rx, ry)


def field_to_map_pose(pose: Pose2D, transform: FieldTransform) -> Pose2D:
    p = field_to_map_point(Point2D(pose.x_m, pose.y_m), transform)
    return Pose2D(p.x_m, p.y_m, normalize_angle(pose.yaw_rad + transform.yaw_rad))


def map_to_field_pose(pose: Pose2D, transform: FieldTransform) -> Pose2D:
    p = map_to_field_point(Point2D(pose.x_m, pose.y_m), transform)
    return Pose2D(p.x_m, p.y_m, normalize_angle(pose.yaw_rad - transform.yaw_rad))


def field_to_map_polygon(polygon: Polygon2D, transform: FieldTransform) -> Polygon2D:
    return Polygon2D(tuple(field_to_map_point(p, transform) for p in polygon.points))


def map_to_field_polygon(polygon: Polygon2D, transform: FieldTransform) -> Polygon2D:
    return Polygon2D(tuple(map_to_field_point(p, transform) for p in polygon.points))


def polygon_to_shapely(polygon: Polygon2D) -> ShapelyPolygon:
    poly = ShapelyPolygon([(p.x_m, p.y_m) for p in polygon.points])
    if not poly.is_valid or poly.is_empty or poly.area <= _EPS:
        raise ValueError("invalid polygon")
    return poly


def path_length(path: MetricPath) -> float:
    return sum(
        math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)
        for a, b in zip(path.poses, path.poses[1:])
    )


@dataclass(frozen=True, slots=True)
class PolylineProjection:
    station_m: float
    path_length_m: float
    projected_pose: Pose2D
    distance_m: float
    segment_index: int


def project_point_to_path(point: Point2D, path: MetricPath) -> PolylineProjection:
    if len(path.poses) < 2:
        raise ValueError("path requires at least two poses")

    best: tuple[float, float, Pose2D, int] | None = None
    cumulative = 0.0
    total = path_length(path)

    for i, (a, b) in enumerate(zip(path.poses, path.poses[1:])):
        dx = b.x_m - a.x_m
        dy = b.y_m - a.y_m
        seg_len2 = dx * dx + dy * dy
        if seg_len2 <= _EPS:
            continue
        seg_len = math.sqrt(seg_len2)
        t = ((point.x_m - a.x_m) * dx + (point.y_m - a.y_m) * dy) / seg_len2
        t = min(1.0, max(0.0, t))
        x = a.x_m + t * dx
        y = a.y_m + t * dy
        dist = math.hypot(point.x_m - x, point.y_m - y)
        station = cumulative + t * seg_len
        yaw = math.atan2(dy, dx)
        pose = Pose2D(x, y, normalize_angle(yaw))
        candidate = (dist, station, pose, i)
        if best is None or candidate[0] < best[0] - _EPS or (
            abs(candidate[0] - best[0]) <= _EPS and candidate[1] < best[1]
        ):
            best = candidate
        cumulative += seg_len

    if best is None:
        raise ValueError("path has no non-zero-length segment")
    return PolylineProjection(best[1], total, best[2], best[0], best[3])


def pose_at_station(path: MetricPath, station_m: float) -> Pose2D:
    if len(path.poses) < 2:
        raise ValueError("path requires at least two poses")
    total = path_length(path)
    if station_m < -_EPS or station_m > total + _EPS:
        raise ValueError("station outside path")
    station_m = min(total, max(0.0, station_m))

    cumulative = 0.0
    last_nonzero: Pose2D | None = None
    for a, b in zip(path.poses, path.poses[1:]):
        dx = b.x_m - a.x_m
        dy = b.y_m - a.y_m
        seg_len = math.hypot(dx, dy)
        if seg_len <= _EPS:
            continue
        yaw = normalize_angle(math.atan2(dy, dx))
        last_nonzero = Pose2D(b.x_m, b.y_m, yaw)
        if station_m <= cumulative + seg_len + _EPS:
            t = (station_m - cumulative) / seg_len
            t = min(1.0, max(0.0, t))
            return Pose2D(a.x_m + t * dx, a.y_m + t * dy, yaw)
        cumulative += seg_len

    if last_nonzero is None:
        raise ValueError("path has no non-zero-length segment")
    return last_nonzero


def _interpolate_angle(a: float, b: float, t: float) -> float:
    return normalize_angle(a + signed_angle_error(b, a) * t)


def sample_path(
    path: MetricPath,
    *,
    max_translation_step_m: float = 0.02,
    max_yaw_step_rad: float = math.radians(3.0),
) -> tuple[Pose2D, ...]:
    """Densely sample the supplied pose path for conservative footprint sweeping."""
    if len(path.poses) < 2:
        raise ValueError("path requires at least two poses")
    if max_translation_step_m <= 0.0 or max_yaw_step_rad <= 0.0:
        raise ValueError("sampling steps must be positive")

    result: list[Pose2D] = [path.poses[0]]
    for a, b in zip(path.poses, path.poses[1:]):
        translation = math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)
        yaw_delta = abs(signed_angle_error(b.yaw_rad, a.yaw_rad))
        steps = max(
            1,
            int(math.ceil(translation / max_translation_step_m)),
            int(math.ceil(yaw_delta / max_yaw_step_rad)),
        )
        for i in range(1, steps + 1):
            t = i / steps
            result.append(
                Pose2D(
                    a.x_m + (b.x_m - a.x_m) * t,
                    a.y_m + (b.y_m - a.y_m) * t,
                    _interpolate_angle(a.yaw_rad, b.yaw_rad, t),
                )
            )
    return tuple(result)


def _footprint_at_pose(base: ShapelyPolygon, pose: Pose2D) -> ShapelyPolygon:
    # Shapely rotates in degrees around origin. Robot footprint is defined in base frame.
    turned = rotate(base, math.degrees(pose.yaw_rad), origin=(0.0, 0.0), use_radians=False)
    return translate(turned, xoff=pose.x_m, yoff=pose.y_m)


def validate_swept_footprint(
    path: MetricPath,
    permitted_regions: tuple[Polygon2D, ...],
    footprint: Polygon2D,
    margins: GeometryMargins,
) -> GeometryResult:
    """Validate the *whole* swept robot footprint against allowed road geometry.

    The three configured margins are conservatively summed and applied as an
    outward buffer around the measured footprint.  The path is densely sampled
    in translation and yaw so curved/rounded-corner sweeps are represented.
    """
    try:
        if len(path.poses) < 2:
            raise ValueError("path requires at least two poses")
        if not permitted_regions:
            raise ValueError("at least one permitted region is required")
        margin = (
            margins.localization_margin_m
            + margins.tracking_margin_m
            + margins.clearance_margin_m
        )
        if margin < 0.0 or not math.isfinite(margin):
            raise ValueError("geometry margins must be finite and non-negative")

        allowed = unary_union([polygon_to_shapely(p) for p in permitted_regions])
        base = polygon_to_shapely(footprint)
        if margin > 0.0:
            base = base.buffer(margin, join_style="round")
        swept = unary_union([_footprint_at_pose(base, p) for p in sample_path(path)])

        # Tiny numerical buffer only; it is not a physical clearance margin.
        valid = allowed.buffer(1e-9).covers(swept)
        if valid:
            min_clearance = swept.distance(allowed.boundary)
            return GeometryResult(
                status=GeometryStatus.VALID,
                valid=True,
                swept_area_m2=float(swept.area),
                min_clearance_m=float(min_clearance),
                reason="",
            )
        outside_area = swept.difference(allowed).area
        return GeometryResult(
            status=GeometryStatus.OUTSIDE_PERMITTED,
            valid=False,
            swept_area_m2=float(swept.area),
            min_clearance_m=0.0,
            reason=f"swept footprint leaves permitted geometry by {outside_area:.6f} m^2",
        )
    except (ValueError, TypeError) as exc:
        return GeometryResult(
            status=GeometryStatus.INVALID_INPUT,
            valid=False,
            swept_area_m2=0.0,
            min_clearance_m=0.0,
            reason=str(exc),
        )


def swept_footprint_intersects_forbidden(
    path: MetricPath,
    footprint: Polygon2D,
    forbidden_regions: tuple[Polygon2D, ...],
    margins: GeometryMargins,
) -> bool:
    """Return True if the inflated swept footprint intersects forbidden geometry."""
    if not forbidden_regions:
        return False
    margin = (
        margins.localization_margin_m
        + margins.tracking_margin_m
        + margins.clearance_margin_m
    )
    if margin < 0.0 or not math.isfinite(margin):
        raise ValueError("geometry margins must be finite and non-negative")
    base = polygon_to_shapely(footprint)
    if margin > 0.0:
        base = base.buffer(margin, join_style="round")
    swept = unary_union([_footprint_at_pose(base, p) for p in sample_path(path)])
    forbidden = unary_union([polygon_to_shapely(p) for p in forbidden_regions])
    return bool(swept.intersects(forbidden))
