"""ROS <-> pure-core conversions owned by the execution side.

Only boundary code imports generated ROS interfaces.  Pure mission/guard modules
remain ROS-independent.
"""
from __future__ import annotations

import math

from frob_city_interfaces.msg import MapState as MapStateMsg
from frob_city_interfaces.msg import MotionPermit as MotionPermitMsg
from frob_city_interfaces.msg import RouteSegment as RouteSegmentMsg
from nav_msgs.msg import Odometry

from .models import (
    ApproachRecord,
    ApproachResolution,
    CoverageRecord,
    CoverageResolution,
    EdgeBlockState,
    EdgeRecord,
    MapSnapshot,
    MetricPath,
    MovementMask,
    Point2D,
    Polygon2D,
    Pose2D,
    PoseSample,
    RouteDependency,
    RouteSegment,
    SegmentKind,
    SignClass,
    TargetRecord,
    TargetResolution,
    TerminalKind,
)
from .motion_guard_core import GuardPermit, OdomSample
from .ros_map_conversions import pose_stamped_to_pose2d, stamp_to_seconds


def _optional_stamp_seconds(stamp) -> float | None:
    if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
        return None
    return stamp_to_seconds(stamp)


def _require_frame(actual: str, expected: str, what: str) -> None:
    if actual != expected:
        raise ValueError(f"{what} frame must be {expected!r}, got {actual!r}")


def route_segment_from_msg(msg: RouteSegmentMsg, *, map_frame: str) -> RouteSegment:
    if not msg.segment_id:
        raise ValueError("segment_id must be non-empty")
    _require_frame(msg.path.header.frame_id, map_frame, "path")
    if not msg.path.poses:
        raise ValueError("segment path is empty")
    path = MetricPath(
        tuple(pose_stamped_to_pose2d(item, map_frame) for item in msg.path.poses)
    )

    regions: list[Polygon2D] = []
    for index, region in enumerate(msg.allowed_regions):
        _require_frame(region.header.frame_id, map_frame, f"allowed_regions[{index}]")
        points = tuple(Point2D(float(p.x), float(p.y)) for p in region.polygon.points)
        if len(points) < 3:
            raise ValueError("allowed region must contain at least three points")
        if not all(math.isfinite(v) for p in points for v in (p.x_m, p.y_m)):
            raise ValueError("allowed region contains non-finite coordinate")
        regions.append(Polygon2D(points))
    if not regions:
        raise ValueError("segment must contain at least one allowed region")

    if len(msg.dependency_ids) != len(msg.dependency_versions):
        raise ValueError("dependency id/version lengths differ")
    if len(set(msg.dependency_ids)) != len(msg.dependency_ids):
        raise ValueError("duplicate dependency id")
    dependencies = tuple(
        RouteDependency(str(dep_id), int(version))
        for dep_id, version in zip(msg.dependency_ids, msg.dependency_versions)
    )

    terminal_limit = pose_stamped_to_pose2d(msg.terminal_limit, map_frame)
    commitment_gate = None
    if msg.commitment_gate_valid:
        commitment_gate = pose_stamped_to_pose2d(msg.commitment_gate, map_frame)

    numeric = (
        msg.max_speed_mps,
        msg.max_yaw_rate_radps,
        msg.xy_tolerance_m,
        msg.yaw_tolerance_rad,
        msg.estimated_duration_s,
    )
    if not all(math.isfinite(float(v)) for v in numeric):
        raise ValueError("segment contains non-finite numeric field")
    if msg.max_speed_mps <= 0.0 or msg.max_yaw_rate_radps <= 0.0:
        raise ValueError("segment speed limits must be positive")

    return RouteSegment(
        segment_id=str(msg.segment_id),
        kind=SegmentKind(int(msg.kind)),
        from_anchor_id=str(msg.from_anchor_id),
        to_anchor_id=str(msg.to_anchor_id),
        approach_id=str(msg.approach_id),
        selected_movement=MovementMask(int(msg.selected_movement)),
        path=path,
        allowed_regions=tuple(regions),
        dependencies=dependencies,
        traversed_edge_ids=tuple(str(v) for v in msg.traversed_edge_ids),
        terminal_kind=TerminalKind(int(msg.terminal_kind)),
        terminal_id=str(msg.terminal_id),
        terminal_limit=terminal_limit,
        commitment_gate=commitment_gate,
        max_speed_mps=float(msg.max_speed_mps),
        max_yaw_rate_radps=float(msg.max_yaw_rate_radps),
        xy_tolerance_m=float(msg.xy_tolerance_m),
        yaw_tolerance_rad=float(msg.yaw_tolerance_rad),
        estimated_duration_s=float(msg.estimated_duration_s),
    )


def _disabled_segment() -> RouteSegment:
    # A disabled permission is rejected before geometry is evaluated.  The wire
    # contract explicitly permits the segment body to be ignored in that case.
    return RouteSegment(
        segment_id="DISABLED",
        kind=SegmentKind.CORRIDOR,
        from_anchor_id="",
        to_anchor_id="",
        approach_id="",
        selected_movement=MovementMask(0),
        path=MetricPath((Pose2D(0.0, 0.0, 0.0),)),
        allowed_regions=(Polygon2D((Point2D(-1.0, -1.0), Point2D(1.0, -1.0), Point2D(1.0, 1.0), Point2D(-1.0, 1.0))),),
        dependencies=(),
        traversed_edge_ids=(),
        terminal_kind=TerminalKind.HOLD,
        terminal_id="",
        terminal_limit=Pose2D(1000.0, 0.0, 0.0),
        commitment_gate=None,
        max_speed_mps=1.0,
        max_yaw_rate_radps=1.0,
        xy_tolerance_m=0.0,
        yaw_tolerance_rad=0.0,
        estimated_duration_s=0.0,
    )


def motion_permit_from_msg(
    msg: MotionPermitMsg,
    *,
    receipt_monotonic_s: float,
    map_frame: str,
) -> GuardPermit:
    if not math.isfinite(float(receipt_monotonic_s)):
        raise ValueError("non-finite permit receipt time")
    segment = (
        route_segment_from_msg(msg.segment, map_frame=map_frame)
        if msg.enabled
        else _disabled_segment()
    )
    return GuardPermit(
        stamp_s=stamp_to_seconds(msg.header.stamp),
        receipt_monotonic_s=float(receipt_monotonic_s),
        run_id=str(msg.run_id),
        map_instance_id=str(msg.map_instance_id),
        navigator_instance_id=str(msg.navigator_instance_id),
        field_id=str(msg.field_id),
        goal_generation=int(msg.goal_generation),
        sequence=int(msg.sequence),
        enabled=bool(msg.enabled),
        valid_for_s=float(msg.valid_for_s),
        segment=segment,
    )


def map_state_from_msg(msg: MapStateMsg, *, map_frame: str) -> MapSnapshot:
    _require_frame(msg.header.frame_id, map_frame, "map state")

    approaches = tuple(
        ApproachRecord(
            approach_id=str(item.approach_id),
            version=int(item.version),
            state=ApproachResolution(int(item.state)),
            confirmed_signs=frozenset(SignClass(int(v)) for v in item.confirmed_sign_classes),
            allowed_movements=MovementMask(int(item.allowed_movement_mask)),
            last_qualified_observation_s=_optional_stamp_seconds(item.last_qualified_observation),
            precautionary_hold=bool(item.precautionary_hold),
            reason=str(item.reason),
        )
        for item in msg.approaches
    )

    targets: list[TargetRecord] = []
    for item in msg.targets:
        sign_pose = pose_stamped_to_pose2d(item.sign_pose, map_frame)
        service_pose = None
        terminal_limit = None
        if item.geometry_valid:
            service_pose = pose_stamped_to_pose2d(item.service_pose, map_frame)
            terminal_limit = pose_stamped_to_pose2d(item.terminal_limit, map_frame)
        targets.append(
            TargetRecord(
                target_id=str(item.target_id),
                version=int(item.version),
                kind=SignClass(int(item.kind)),
                state=TargetResolution(int(item.state)),
                site_id=str(item.site_id),
                face_id=str(item.face_id),
                directed_road_id=str(item.directed_road_id),
                service_anchor_id=str(item.service_anchor_id),
                sign_pose=sign_pose,
                geometry_valid=bool(item.geometry_valid),
                service_pose=service_pose,
                terminal_limit=terminal_limit,
                xy_tolerance_m=float(item.xy_tolerance_m),
                yaw_tolerance_rad=float(item.yaw_tolerance_rad),
                front_clearance_min_m=float(item.front_clearance_min_m),
                last_seen_s=stamp_to_seconds(item.last_seen),
            )
        )

    edges = tuple(
        EdgeRecord(
            edge_id=str(item.edge_id),
            version=int(item.version),
            state=EdgeBlockState(int(item.state)),
            completed_traversals=int(item.completed_traversals),
            next_recheck_after_s=_optional_stamp_seconds(item.next_recheck_after),
            reason=str(item.reason),
        )
        for item in msg.edges
    )

    coverage = tuple(
        CoverageRecord(
            coverage_id=str(item.coverage_id),
            version=int(item.version),
            state=CoverageResolution(int(item.state)),
            failed_attempts=int(item.failed_attempts),
            last_qualified_observation_s=_optional_stamp_seconds(item.last_qualified_observation),
            reason=str(item.reason),
        )
        for item in msg.coverage
    )

    return MapSnapshot(
        run_id=str(msg.run_id),
        map_instance_id=str(msg.map_instance_id),
        field_id=str(msg.field_id),
        revision=int(msg.revision),
        ready=bool(msg.ready),
        perception_healthy=bool(msg.perception_healthy),
        localization_healthy=bool(msg.localization_healthy),
        precautionary_hold_ids=tuple(str(v) for v in msg.precautionary_hold_ids),
        approaches=approaches,
        targets=tuple(targets),
        edges=edges,
        coverage=coverage,
        reason=str(msg.reason),
    )


def odometry_to_guard_samples(
    msg: Odometry,
    *,
    receipt_monotonic_s: float,
    map_frame: str,
    allow_map_frame_pose: bool,
) -> tuple[OdomSample, PoseSample | None]:
    stamp_s = stamp_to_seconds(msg.header.stamp)
    lin = msg.twist.twist.linear
    ang = msg.twist.twist.angular
    values = (lin.x, lin.y, ang.z)
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError("odometry twist contains non-finite value")
    odom = OdomSample(
        timestamp_s=stamp_s,
        receipt_monotonic_s=float(receipt_monotonic_s),
        linear_x_mps=float(lin.x),
        linear_y_mps=float(lin.y),
        angular_z_radps=float(ang.z),
    )
    pose = None
    if allow_map_frame_pose and msg.header.frame_id == map_frame:
        from geometry_msgs.msg import PoseStamped

        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose.pose
        pose = PoseSample(
            timestamp_s=stamp_s,
            pose=pose_stamped_to_pose2d(p, map_frame),
            linear_speed_mps=math.hypot(float(lin.x), float(lin.y)),
            angular_speed_radps=abs(float(ang.z)),
        )
    return odom, pose

# ---- WP12 navigator-side conversions -------------------------------------------------

def route_segment_to_nav_path(msg: RouteSegmentMsg, *, map_frame: str):
    """Return a detached, validated nav_msgs/Path for Nav2 FollowPath."""
    from copy import deepcopy

    _require_frame(msg.path.header.frame_id, map_frame, 'path')
    if not msg.path.poses:
        raise ValueError('segment path is empty')
    for index, pose in enumerate(msg.path.poses):
        _require_frame(pose.header.frame_id, map_frame, f'path.poses[{index}]')
    return deepcopy(msg.path)


def build_motion_permit_msg(
    *,
    now_msg,
    run_id: str,
    map_instance_id: str,
    navigator_instance_id: str,
    field_id: str,
    goal_generation: int,
    sequence: int,
    enabled: bool,
    valid_for_s: float,
    segment_msg: RouteSegmentMsg | None,
):
    """Build the exact guard permission heartbeat used by navigator."""
    from copy import deepcopy

    if enabled and segment_msg is None:
        raise ValueError('enabled permit requires a RouteSegment')
    if sequence <= 0:
        raise ValueError('permit sequence must be positive')
    if enabled and goal_generation <= 0:
        raise ValueError('enabled permit requires positive goal_generation')
    msg = MotionPermitMsg()
    msg.header.stamp = now_msg
    msg.header.frame_id = ''
    msg.run_id = str(run_id)
    msg.map_instance_id = str(map_instance_id)
    msg.navigator_instance_id = str(navigator_instance_id)
    msg.field_id = str(field_id)
    msg.goal_generation = int(goal_generation)
    msg.sequence = int(sequence)
    msg.enabled = bool(enabled)
    msg.valid_for_s = float(valid_for_s)
    if segment_msg is not None:
        msg.segment = deepcopy(segment_msg)
    return msg


def mission_state_to_msg(snapshot, *, now_msg, current_dwell_s: float):
    """Convert the WP09 MissionSnapshot to the frozen MissionState ROS message."""
    from frob_city_interfaces.msg import MissionState

    msg = MissionState()
    msg.header.stamp = now_msg
    msg.run_id = str(snapshot.run_id)
    msg.navigator_instance_id = str(snapshot.navigator_instance_id)
    msg.phase = int(snapshot.phase)
    msg.execution_state = int(snapshot.execution_state)
    msg.objective_id = str(snapshot.objective_id)
    msg.route_id = str(snapshot.route_id)
    msg.active_segment_id = str(snapshot.active_segment_id)
    msg.goal_generation = int(snapshot.goal_generation)
    msg.required_unique_bus_stops = int(snapshot.required_unique_bus_stops)
    msg.serviced_stop_ids = list(snapshot.serviced_stop_ids)
    msg.current_dwell_s = float(max(0.0, current_dwell_s))
    msg.parking_prior_state = int(snapshot.parking_prior_state)
    msg.attempt_tainted = bool(snapshot.attempt_tainted)
    msg.reason = str(snapshot.reason)
    return msg
