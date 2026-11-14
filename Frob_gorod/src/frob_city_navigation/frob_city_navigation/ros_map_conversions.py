"""ROS boundary conversions for the Frob City map planner.

This module is intentionally the boundary: pure domain modules remain free of
rclpy and generated ROS messages.  Runtime validation rejects malformed wire
objects instead of silently normalizing incompatible data.
"""

from __future__ import annotations

import math
from typing import Iterable

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header

from frob_city_interfaces.msg import (
    ApproachState as ApproachStateMsg,
    CoverageState as CoverageStateMsg,
    EdgeState as EdgeStateMsg,
    KnownSignState as KnownSignStateMsg,
    MapState as MapStateMsg,
    RouteSegment as RouteSegmentMsg,
    SignDetectionArray,
    TargetState as TargetStateMsg,
)

from .models import (
    BoundingBox,
    CorePlanRequest,
    DetectionObservation,
    FieldTransform,
    MapSnapshot,
    PlanGoalKind,
    Point2D,
    Polygon2D,
    Pose2D,
    PoseSample,
    RangeConvention,
    RouteSegment,
    SignClass,
)
from .geometry import field_to_map_polygon, field_to_map_pose


_EPS = 1e-9


def stamp_to_seconds(stamp: TimeMsg) -> float:
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("invalid ROS timestamp")
    return value


def seconds_to_time(value: float) -> TimeMsg:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("time must be finite and >= 0")
    sec = int(math.floor(value))
    nanosec = int(round((value - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    msg = TimeMsg()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


def _validate_finite(*values: float) -> None:
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite numeric value")


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    _validate_finite(x, y, z, w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= _EPS or abs(norm - 1.0) > 1e-2:
        raise ValueError(f"quaternion is not normalized: norm={norm}")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _fill_pose(msg, pose: Pose2D) -> None:
    _validate_finite(pose.x_m, pose.y_m, pose.yaw_rad)
    msg.position.x = float(pose.x_m)
    msg.position.y = float(pose.y_m)
    msg.position.z = 0.0
    msg.orientation.x = 0.0
    msg.orientation.y = 0.0
    msg.orientation.z = math.sin(pose.yaw_rad * 0.5)
    msg.orientation.w = math.cos(pose.yaw_rad * 0.5)


def pose2d_to_pose_stamped(pose: Pose2D, frame_id: str, stamp: TimeMsg) -> PoseStamped:
    if not frame_id:
        raise ValueError("frame_id must not be empty")
    result = PoseStamped()
    result.header = Header(stamp=stamp, frame_id=frame_id)
    _fill_pose(result.pose, pose)
    return result


def pose_stamped_to_pose2d(msg: PoseStamped, expected_frame: str) -> Pose2D:
    if msg.header.frame_id != expected_frame:
        raise ValueError(
            f"wrong pose frame: expected {expected_frame!r}, got {msg.header.frame_id!r}"
        )
    p = msg.pose.position
    q = msg.pose.orientation
    _validate_finite(p.x, p.y, p.z)
    return Pose2D(float(p.x), float(p.y), yaw_from_quaternion(q.x, q.y, q.z, q.w))


def odometry_to_pose_sample(msg: Odometry, expected_map_frame: str) -> PoseSample:
    """Convert odometry only when its pose is already expressed in map.

    Real deployments commonly publish odom-frame poses.  The node handles that
    case through capture-time TF instead of pretending odom coordinates are map
    coordinates.
    """
    if msg.header.frame_id != expected_map_frame:
        raise ValueError("odometry pose is not expressed in map frame")
    pose = PoseStamped()
    pose.header = msg.header
    pose.pose = msg.pose.pose
    core_pose = pose_stamped_to_pose2d(pose, expected_map_frame)
    lin = msg.twist.twist.linear
    ang = msg.twist.twist.angular
    linear_speed = math.hypot(float(lin.x), float(lin.y))
    angular_speed = abs(float(ang.z))
    _validate_finite(linear_speed, angular_speed)

    covariance_xy = None
    covariance_yaw = None
    cov = msg.pose.covariance
    if len(cov) >= 36:
        candidates = [float(cov[0]), float(cov[7])]
        if all(math.isfinite(v) and v >= 0.0 for v in candidates):
            covariance_xy = max(candidates)
        yaw_var = float(cov[35])
        if math.isfinite(yaw_var) and yaw_var >= 0.0:
            covariance_yaw = yaw_var

    return PoseSample(
        timestamp_s=stamp_to_seconds(msg.header.stamp),
        pose=core_pose,
        linear_speed_mps=linear_speed,
        angular_speed_radps=angular_speed,
        covariance_xy_m2=covariance_xy,
        covariance_yaw_rad2=covariance_yaw,
    )


def transform_to_pose2d(transform) -> Pose2D:
    t = transform.transform.translation
    q = transform.transform.rotation
    _validate_finite(t.x, t.y, t.z)
    return Pose2D(float(t.x), float(t.y), yaw_from_quaternion(q.x, q.y, q.z, q.w))


def polygon_to_msg(poly: Polygon2D, frame_id: str, stamp: TimeMsg) -> PolygonStamped:
    out = PolygonStamped()
    out.header = Header(stamp=stamp, frame_id=frame_id)
    out.polygon.points = [Point32(x=float(p.x_m), y=float(p.y_m), z=0.0) for p in poly.points]
    return out


def metric_path_to_msg(path, frame_id: str, stamp: TimeMsg) -> Path:
    if not path.poses:
        raise ValueError("path is empty")
    out = Path()
    out.header = Header(stamp=stamp, frame_id=frame_id)
    out.poses = [pose2d_to_pose_stamped(pose, frame_id, stamp) for pose in path.poses]
    return out


def route_segment_to_msg(
    segment: RouteSegment,
    frame_id: str,
    stamp: TimeMsg,
    *,
    field_to_map: FieldTransform | None = None,
) -> RouteSegmentMsg:
    if len({d.dependency_id for d in segment.dependencies}) != len(segment.dependencies):
        raise ValueError("duplicate route dependency id")
    out = RouteSegmentMsg()
    out.segment_id = segment.segment_id
    out.kind = int(segment.kind)
    out.from_anchor_id = segment.from_anchor_id
    out.to_anchor_id = segment.to_anchor_id
    out.approach_id = segment.approach_id
    out.selected_movement = int(segment.selected_movement)
    transform_pose = (
        (lambda pose: field_to_map_pose(pose, field_to_map))
        if field_to_map is not None
        else (lambda pose: pose)
    )
    transform_polygon = (
        (lambda polygon: field_to_map_polygon(polygon, field_to_map))
        if field_to_map is not None
        else (lambda polygon: polygon)
    )
    mapped_path = type(segment.path)(tuple(transform_pose(pose) for pose in segment.path.poses))
    out.path = metric_path_to_msg(mapped_path, frame_id, stamp)
    out.allowed_regions = [
        polygon_to_msg(transform_polygon(p), frame_id, stamp)
        for p in segment.allowed_regions
    ]
    out.dependency_ids = [d.dependency_id for d in segment.dependencies]
    out.dependency_versions = [int(d.version) for d in segment.dependencies]
    out.traversed_edge_ids = list(segment.traversed_edge_ids)
    out.terminal_kind = int(segment.terminal_kind)
    out.terminal_id = segment.terminal_id
    out.terminal_limit = pose2d_to_pose_stamped(
        transform_pose(segment.terminal_limit), frame_id, stamp
    )
    out.commitment_gate_valid = segment.commitment_gate is not None
    out.commitment_gate = pose2d_to_pose_stamped(
        transform_pose(segment.commitment_gate or segment.terminal_limit), frame_id, stamp
    )
    out.max_speed_mps = float(segment.max_speed_mps)
    out.max_yaw_rate_radps = float(segment.max_yaw_rate_radps)
    out.xy_tolerance_m = float(segment.xy_tolerance_m)
    out.yaw_tolerance_rad = float(segment.yaw_tolerance_rad)
    out.estimated_duration_s = float(segment.estimated_duration_s)
    return out


def _optional_time(value: float | None) -> TimeMsg:
    return TimeMsg() if value is None else seconds_to_time(value)


def map_snapshot_to_msg(
    snapshot: MapSnapshot,
    *,
    frame_id: str,
    stamp: TimeMsg,
    ready_override: bool | None = None,
    perception_healthy_override: bool | None = None,
    localization_healthy_override: bool | None = None,
    reason_override: str | None = None,
    field_to_map: FieldTransform | None = None,
) -> MapStateMsg:
    transform_pose = (
        (lambda pose: field_to_map_pose(pose, field_to_map))
        if field_to_map is not None
        else (lambda pose: pose)
    )
    out = MapStateMsg()
    out.header = Header(stamp=stamp, frame_id=frame_id)
    out.run_id = snapshot.run_id
    out.map_instance_id = snapshot.map_instance_id
    out.field_id = snapshot.field_id
    out.revision = int(snapshot.revision)
    out.ready = snapshot.ready if ready_override is None else bool(ready_override)
    out.perception_healthy = (
        snapshot.perception_healthy
        if perception_healthy_override is None
        else bool(perception_healthy_override)
    )
    out.localization_healthy = (
        snapshot.localization_healthy
        if localization_healthy_override is None
        else bool(localization_healthy_override)
    )
    out.precautionary_hold_ids = list(snapshot.precautionary_hold_ids)

    for item in snapshot.approaches:
        msg = ApproachStateMsg()
        msg.approach_id = item.approach_id
        msg.version = int(item.version)
        msg.state = int(item.state)
        msg.confirmed_sign_classes = [int(v) for v in sorted(item.confirmed_signs, key=int)]
        msg.allowed_movement_mask = int(item.allowed_movements)
        msg.last_qualified_observation = _optional_time(item.last_qualified_observation_s)
        msg.precautionary_hold = bool(item.precautionary_hold)
        msg.reason = item.reason
        out.approaches.append(msg)

    for item in snapshot.known_signs:
        msg = KnownSignStateMsg()
        msg.site_id = item.site_id
        msg.face_id = item.face_id
        msg.approach_id = item.approach_id
        msg.sign_class = int(item.sign_class)
        msg.version = int(item.version)
        msg.last_seen = seconds_to_time(item.last_seen_s)
        msg.confidence = float(item.confidence)
        out.known_signs.append(msg)

    for item in snapshot.targets:
        msg = TargetStateMsg()
        msg.target_id = item.target_id
        msg.version = int(item.version)
        msg.kind = int(item.kind)
        msg.state = int(item.state)
        msg.site_id = item.site_id
        msg.face_id = item.face_id
        msg.directed_road_id = item.directed_road_id
        msg.service_anchor_id = item.service_anchor_id
        msg.sign_pose = pose2d_to_pose_stamped(transform_pose(item.sign_pose), frame_id, stamp)
        msg.geometry_valid = bool(item.geometry_valid)
        if item.service_pose is not None:
            msg.service_pose = pose2d_to_pose_stamped(
                transform_pose(item.service_pose), frame_id, stamp
            )
        if item.terminal_limit is not None:
            msg.terminal_limit = pose2d_to_pose_stamped(
                transform_pose(item.terminal_limit), frame_id, stamp
            )
        msg.xy_tolerance_m = float(item.xy_tolerance_m)
        msg.yaw_tolerance_rad = float(item.yaw_tolerance_rad)
        msg.front_clearance_min_m = float(item.front_clearance_min_m)
        msg.last_seen = seconds_to_time(item.last_seen_s)
        out.targets.append(msg)

    for item in snapshot.edges:
        msg = EdgeStateMsg()
        msg.edge_id = item.edge_id
        msg.version = int(item.version)
        msg.state = int(item.state)
        msg.completed_traversals = int(item.completed_traversals)
        msg.next_recheck_after = _optional_time(item.next_recheck_after_s)
        msg.reason = item.reason
        out.edges.append(msg)

    for item in snapshot.coverage:
        msg = CoverageStateMsg()
        msg.coverage_id = item.coverage_id
        msg.version = int(item.version)
        msg.state = int(item.state)
        msg.failed_attempts = int(item.failed_attempts)
        msg.last_qualified_observation = _optional_time(item.last_qualified_observation_s)
        msg.reason = item.reason
        out.coverage.append(msg)

    out.reason = snapshot.reason if reason_override is None else reason_override
    return out


def plan_request_from_ros(request) -> CorePlanRequest:
    try:
        goal_kind = PlanGoalKind(int(request.goal_kind))
    except ValueError as exc:
        raise ValueError(f"unsupported goal_kind: {request.goal_kind}") from exc
    if len(set(request.exclude_target_ids)) != len(request.exclude_target_ids):
        raise ValueError("duplicate exclude_target_ids")
    return CorePlanRequest(
        run_id=request.run_id,
        map_instance_id=request.map_instance_id,
        request_id=int(request.request_id),
        minimum_revision=int(request.minimum_revision),
        goal_kind=goal_kind,
        target_id=request.target_id,
        exclude_target_ids=tuple(request.exclude_target_ids),
    )


def detection_array_to_core(msg: SignDetectionArray) -> tuple[DetectionObservation, ...]:
    if not msg.header.frame_id:
        raise ValueError("detection optical frame is empty")
    if not msg.detector_instance_id:
        raise ValueError("detector_instance_id is empty")
    if not msg.frame_valid:
        return ()
    timestamp_s = stamp_to_seconds(msg.header.stamp)
    observations: list[DetectionObservation] = []
    for item in msg.detections:
        try:
            sign_class = SignClass(int(item.sign_class))
            convention = RangeConvention(int(item.range_convention))
        except ValueError as exc:
            raise ValueError("unsupported detection enum") from exc
        confidence = float(item.confidence)
        if not math.isfinite(confidence) or not (0.0 <= confidence <= 1.0):
            raise ValueError("invalid detection confidence")
        if item.width_px <= 0 or item.height_px <= 0:
            raise ValueError("invalid detection bounding box")
        range_m = None
        range_std = None
        if item.range_valid:
            range_m = float(item.range_m)
            range_std = float(item.range_stddev_m)
            if convention is RangeConvention.RANGE_UNKNOWN:
                raise ValueError("valid range has unknown convention")
            if not math.isfinite(range_m) or range_m <= 0.0:
                raise ValueError("invalid range_m")
            if not math.isfinite(range_std) or range_std < 0.0:
                raise ValueError("invalid range_stddev_m")
        observations.append(
            DetectionObservation(
                timestamp_s=timestamp_s,
                detector_instance_id=msg.detector_instance_id,
                frame_sequence=int(msg.frame_sequence),
                optical_frame=msg.header.frame_id,
                sign_class=sign_class,
                confidence=confidence,
                bbox=BoundingBox(
                    int(item.x_min_px), int(item.y_min_px), int(item.width_px), int(item.height_px)
                ),
                range_valid=bool(item.range_valid),
                range_convention=convention,
                range_m=range_m,
                range_stddev_m=range_std,
                detector_track_id=item.detector_track_id,
            )
        )
    return tuple(observations)


def fill_plan_response(
    result,
    response,
    *,
    frame_id: str,
    stamp: TimeMsg,
    field_to_map: FieldTransform | None = None,
):
    response.status = int(result.status)
    response.request_id = int(result.request_id)
    response.run_id = result.run_id
    response.map_instance_id = result.map_instance_id
    response.revision = int(result.revision)
    response.route_id = result.route_id
    response.selected_target_id = result.selected_target_id
    response.start_pose = pose2d_to_pose_stamped(result.start_pose, frame_id, stamp)
    response.reaches_requested_target = bool(result.reaches_requested_target)
    response.estimated_total_cost_s = float(result.estimated_total_cost_s)
    response.cost_uses_unknown_gates = bool(result.cost_uses_unknown_gates)
    response.executable_prefix = [
        route_segment_to_msg(segment, frame_id, stamp, field_to_map=field_to_map)
        for segment in result.executable_prefix
    ]
    response.terminal_observation_id = result.terminal_observation_id
    response.reason = result.reason
    return response


def fill_validation_response(result, response, *, frame_id: str, stamp: TimeMsg):
    response.status = int(result.status)
    response.current_revision = int(result.current_revision)
    response.checked_robot_pose = pose2d_to_pose_stamped(result.checked_robot_pose, frame_id, stamp)
    response.observation_id = result.observation_id
    response.reason = result.reason
    return response
