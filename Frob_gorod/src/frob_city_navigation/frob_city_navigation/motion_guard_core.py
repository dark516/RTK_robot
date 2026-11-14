"""WP10A pure motion-safety guard for Frob City.

This module is deliberately ROS independent.  It evaluates the most recently
received immutable motion permit, raw controller command, map snapshot and
measured state.  It never plans, changes a route, or invents a recovery.

The guard is fail-closed: every missing, stale, identity-mismatched or
geometrically invalid condition returns a zero command.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Iterable

from frob_city_navigation.models import (
    GuardDecisionKind,
    MapSnapshot,
    Point2D,
    Polygon2D,
    Pose2D,
    PoseSample,
    RouteSegment,
)


_EPS = 1e-9
# MotionPermit.valid_for_s is float32 on the ROS wire.  Permit values at
# the configured boundary can round a few nanoseconds upward (for example
# 0.20 -> 0.20000000298).  Allow only a tiny wire-quantization margin.
_WIRE_FLOAT_EPS = 1e-6


class GuardFault(IntEnum):
    EXTERNAL = 0
    COMMAND_NONFINITE = 1
    GEOMETRY_VIOLATION = 2
    IDENTITY_CORRUPTION = 3


@dataclass(frozen=True, slots=True)
class GuardConfig:
    field_id: str
    footprint: Polygon2D
    braking_deceleration_mps2: float
    braking_latency_s: float
    hard_max_linear_speed_mps: float
    hard_max_yaw_rate_radps: float
    path_lateral_tolerance_m: float
    permit_max_age_s: float = 0.20
    raw_command_max_age_s: float = 0.20
    map_max_age_s: float = 1.00
    pose_max_age_s: float = 0.20
    odom_max_age_s: float = 0.20
    max_permit_lifetime_s: float = 0.20
    stationary_linear_max_mps: float = 0.01
    stationary_angular_max_radps: float = 0.03
    footprint_edge_sample_m: float = 0.02
    rollout_step_m: float = 0.02
    geometry_epsilon_m: float = 1e-6

    def __post_init__(self) -> None:
        if not self.field_id:
            raise ValueError("field_id must be non-empty")
        if len(self.footprint.points) < 3:
            raise ValueError("footprint must contain at least three points")
        positive = (
            "braking_deceleration_mps2",
            "hard_max_linear_speed_mps",
            "hard_max_yaw_rate_radps",
            "permit_max_age_s",
            "raw_command_max_age_s",
            "map_max_age_s",
            "pose_max_age_s",
            "odom_max_age_s",
            "max_permit_lifetime_s",
            "footprint_edge_sample_m",
            "rollout_step_m",
        )
        nonnegative = (
            "braking_latency_s",
            "path_lateral_tolerance_m",
            "stationary_linear_max_mps",
            "stationary_angular_max_radps",
            "geometry_epsilon_m",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        for name in nonnegative:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")


@dataclass(frozen=True, slots=True)
class GuardPermit:
    stamp_s: float
    receipt_monotonic_s: float
    run_id: str
    map_instance_id: str
    navigator_instance_id: str
    field_id: str
    goal_generation: int
    sequence: int
    enabled: bool
    valid_for_s: float
    segment: RouteSegment


@dataclass(frozen=True, slots=True)
class RawCommand:
    stamp_s: float
    receipt_monotonic_s: float
    linear_x_mps: float
    linear_y_mps: float
    angular_z_radps: float


@dataclass(frozen=True, slots=True)
class OdomSample:
    timestamp_s: float
    receipt_monotonic_s: float
    linear_x_mps: float
    linear_y_mps: float
    angular_z_radps: float


@dataclass(frozen=True, slots=True)
class GuardInputs:
    now_ros_s: float
    run_id: str
    map_instance_id: str
    navigator_instance_id: str
    field_id: str
    permit: GuardPermit | None
    raw_command: RawCommand | None
    map_snapshot: MapSnapshot | None
    map_stamp_s: float
    map_receipt_monotonic_s: float
    pose: PoseSample | None
    pose_receipt_monotonic_s: float
    odom: OdomSample | None
    emergency_stop_active: bool = False


@dataclass(frozen=True, slots=True)
class GuardDecision:
    kind: GuardDecisionKind
    motion_allowed: bool
    linear_x_mps: float
    linear_y_mps: float
    angular_z_radps: float
    stopped: bool
    committed: bool
    active_segment_id: str
    last_permission_sequence: int
    fault_latched: bool
    reason: str


@dataclass(frozen=True, slots=True)
class ResetContext:
    run_id: str
    map_instance_id: str
    navigator_instance_id: str
    measured_stopped: bool
    emergency_stop_active: bool = False


@dataclass(frozen=True, slots=True)
class ResetResult:
    accepted: bool
    fault_latched: bool
    reason: str


class MotionGuardCore:
    """Stateful fail-closed evaluator for a single navigator session."""

    def __init__(self, config: GuardConfig) -> None:
        self._config = config
        self._fault_latched = False
        self._fault_reason = ""
        self._context: tuple[str, str, str, str] | None = None
        self._last_sequence = -1
        self._last_sequence_receipt_s = -math.inf
        self._active_generation = -1
        self._generation_started_monotonic_s = -math.inf

    def evaluate(self, inputs: GuardInputs, now_monotonic: float) -> GuardDecision:
        now_monotonic = float(now_monotonic)
        if not math.isfinite(now_monotonic) or not math.isfinite(float(inputs.now_ros_s)):
            return self._hold(inputs, "clock_nonfinite", fault=True)

        stopped = self._measured_stopped(inputs.odom)

        if self._fault_latched:
            return self._zero(
                GuardDecisionKind.FAULT,
                inputs,
                stopped=stopped,
                reason=f"fault_latched:{self._fault_reason}",
            )
        if inputs.emergency_stop_active:
            return self._zero(
                GuardDecisionKind.HOLD,
                inputs,
                stopped=stopped,
                reason="emergency_stop_active",
            )

        identity_reason = self._validate_context_identity(inputs)
        if identity_reason:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, identity_reason)

        permit = inputs.permit
        if permit is None:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, "permit_missing")
        if not permit.enabled:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, "permit_disabled")

        permit_reason = self._validate_permit(permit, inputs, now_monotonic)
        if permit_reason:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, permit_reason)

        sequence_reason = self._accept_or_validate_cached_permit(permit)
        if sequence_reason:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, sequence_reason)

        map_reason = self._validate_map(inputs, now_monotonic, permit.segment)
        if map_reason:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, map_reason)

        pose_reason = self._validate_pose_and_odom(inputs, now_monotonic)
        if pose_reason:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, pose_reason)
        assert inputs.pose is not None
        assert inputs.odom is not None

        geometry_reason = self._validate_current_geometry(inputs.pose.pose, permit.segment)
        if geometry_reason:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, geometry_reason)

        cmd = inputs.raw_command
        if cmd is None:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, "raw_command_missing")
        command_reason = self._validate_command(cmd, inputs, permit.segment, now_monotonic)
        if command_reason:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, command_reason)

        rollout_reason = self._validate_braking_rollout(
            inputs.pose.pose,
            inputs.odom,
            cmd,
            permit.segment,
        )
        if rollout_reason:
            return self._zero(GuardDecisionKind.HOLD, inputs, stopped, rollout_reason)

        committed = _footprint_crosses_plane(
            inputs.pose.pose,
            self._config.footprint,
            permit.segment.commitment_gate,
            self._config.geometry_epsilon_m,
        ) if permit.segment.commitment_gate is not None else False

        return GuardDecision(
            kind=GuardDecisionKind.ALLOW,
            motion_allowed=True,
            linear_x_mps=float(cmd.linear_x_mps),
            linear_y_mps=float(cmd.linear_y_mps),
            angular_z_radps=float(cmd.angular_z_radps),
            stopped=stopped,
            committed=committed,
            active_segment_id=permit.segment.segment_id,
            last_permission_sequence=self._last_sequence,
            fault_latched=False,
            reason="motion_permitted",
        )

    def latch_fault(self, reason: GuardFault | str) -> GuardDecision:
        if isinstance(reason, GuardFault):
            text = reason.name.lower()
        else:
            text = str(reason).strip() or "external"
        self._fault_latched = True
        self._fault_reason = text
        return GuardDecision(
            kind=GuardDecisionKind.FAULT,
            motion_allowed=False,
            linear_x_mps=0.0,
            linear_y_mps=0.0,
            angular_z_radps=0.0,
            stopped=False,
            committed=False,
            active_segment_id="",
            last_permission_sequence=self._last_sequence,
            fault_latched=True,
            reason=f"fault_latched:{text}",
        )

    def reset_while_stopped(self, context: ResetContext) -> ResetResult:
        if not context.run_id or not context.map_instance_id or not context.navigator_instance_id:
            return ResetResult(False, self._fault_latched, "reset_identity_missing")
        if context.emergency_stop_active:
            return ResetResult(False, self._fault_latched, "emergency_stop_active")
        if not context.measured_stopped:
            return ResetResult(False, self._fault_latched, "robot_not_measured_stopped")
        self._fault_latched = False
        self._fault_reason = ""
        self._context = None
        self._last_sequence = -1
        self._last_sequence_receipt_s = -math.inf
        self._active_generation = -1
        self._generation_started_monotonic_s = -math.inf
        return ResetResult(True, False, "reset_accepted")

    def _validate_context_identity(self, inputs: GuardInputs) -> str | None:
        identity = (
            inputs.run_id,
            inputs.map_instance_id,
            inputs.navigator_instance_id,
            inputs.field_id,
        )
        if not all(identity):
            return "current_identity_missing"
        if inputs.field_id != self._config.field_id:
            return "field_identity_mismatch"
        if self._context is None:
            self._context = identity
        elif self._context != identity:
            # A session change is not silently accepted while the core is live.
            return "current_session_changed_without_reset"
        return None

    def _validate_permit(
        self,
        permit: GuardPermit,
        inputs: GuardInputs,
        now_monotonic: float,
    ) -> str | None:
        values = (
            permit.stamp_s,
            permit.receipt_monotonic_s,
            permit.valid_for_s,
        )
        if not all(math.isfinite(float(v)) for v in values):
            return "permit_nonfinite"
        if permit.goal_generation < 0 or permit.sequence < 0:
            return "permit_counter_invalid"
        if permit.valid_for_s <= 0.0:
            return "permit_lifetime_invalid"
        if permit.valid_for_s > self._config.max_permit_lifetime_s + _WIRE_FLOAT_EPS:
            return "permit_lifetime_excessive"
        if (
            permit.run_id != inputs.run_id
            or permit.map_instance_id != inputs.map_instance_id
            or permit.navigator_instance_id != inputs.navigator_instance_id
            or permit.field_id != inputs.field_id
        ):
            return "permit_identity_mismatch"
        ros_age = float(inputs.now_ros_s) - float(permit.stamp_s)
        mono_age = now_monotonic - float(permit.receipt_monotonic_s)
        if ros_age < -_EPS or mono_age < -_EPS:
            return "permit_from_future"
        if ros_age > min(self._config.permit_max_age_s, permit.valid_for_s) + _EPS:
            return "permit_ros_stale"
        if mono_age > min(self._config.permit_max_age_s, permit.valid_for_s) + _EPS:
            return "permit_receipt_stale"
        return None

    def _accept_or_validate_cached_permit(self, permit: GuardPermit) -> str | None:
        if self._last_sequence < 0:
            self._last_sequence = permit.sequence
            self._last_sequence_receipt_s = permit.receipt_monotonic_s
            self._active_generation = permit.goal_generation
            self._generation_started_monotonic_s = permit.receipt_monotonic_s
            return None

        if permit.sequence < self._last_sequence:
            return "permit_sequence_reordered"
        if permit.sequence == self._last_sequence:
            if abs(permit.receipt_monotonic_s - self._last_sequence_receipt_s) > _EPS:
                return "permit_duplicate_replayed"
            if permit.goal_generation != self._active_generation:
                return "permit_generation_changed_without_new_sequence"
            return None

        # New permission heartbeat. Generation may stay the same or increase,
        # never decrease.
        if permit.goal_generation < self._active_generation:
            return "permit_old_generation"
        if permit.goal_generation > self._active_generation:
            self._active_generation = permit.goal_generation
            self._generation_started_monotonic_s = permit.receipt_monotonic_s
        self._last_sequence = permit.sequence
        self._last_sequence_receipt_s = permit.receipt_monotonic_s
        return None

    def _validate_map(
        self,
        inputs: GuardInputs,
        now_monotonic: float,
        segment: RouteSegment,
    ) -> str | None:
        snap = inputs.map_snapshot
        if snap is None:
            return "map_snapshot_missing"
        if not math.isfinite(float(inputs.map_stamp_s)) or not math.isfinite(float(inputs.map_receipt_monotonic_s)):
            return "map_timestamp_nonfinite"
        ros_age = float(inputs.now_ros_s) - float(inputs.map_stamp_s)
        mono_age = now_monotonic - float(inputs.map_receipt_monotonic_s)
        if ros_age < -_EPS or mono_age < -_EPS:
            return "map_snapshot_from_future"
        if ros_age > self._config.map_max_age_s + _EPS or mono_age > self._config.map_max_age_s + _EPS:
            return "map_snapshot_stale"
        if (
            snap.run_id != inputs.run_id
            or snap.map_instance_id != inputs.map_instance_id
            or snap.field_id != inputs.field_id
        ):
            return "map_identity_mismatch"
        if not snap.ready:
            return "map_not_ready"
        if not snap.perception_healthy:
            return "perception_unhealthy"
        if not snap.localization_healthy:
            return "localization_unhealthy"

        versions = _dependency_versions(snap)
        for dep in segment.dependencies:
            current = versions.get(dep.dependency_id)
            if current is None:
                return f"dependency_missing:{dep.dependency_id}"
            if current != dep.version:
                return f"dependency_changed:{dep.dependency_id}"

        relevant = _segment_relevance_ids(segment)
        for hold_id in snap.precautionary_hold_ids:
            if hold_id in relevant or any(item.endswith(f":{hold_id}") for item in relevant):
                return f"precautionary_hold:{hold_id}"
        return None

    def _validate_pose_and_odom(self, inputs: GuardInputs, now_monotonic: float) -> str | None:
        pose = inputs.pose
        odom = inputs.odom
        if pose is None:
            return "pose_missing"
        if odom is None:
            return "odometry_missing"
        for name, value in (
            ("pose_timestamp", pose.timestamp_s),
            ("pose_receipt", inputs.pose_receipt_monotonic_s),
            ("odom_timestamp", odom.timestamp_s),
            ("odom_receipt", odom.receipt_monotonic_s),
        ):
            if not math.isfinite(float(value)):
                return f"{name}_nonfinite"
        pose_ros_age = float(inputs.now_ros_s) - float(pose.timestamp_s)
        pose_mono_age = now_monotonic - float(inputs.pose_receipt_monotonic_s)
        odom_ros_age = float(inputs.now_ros_s) - float(odom.timestamp_s)
        odom_mono_age = now_monotonic - float(odom.receipt_monotonic_s)
        if min(pose_ros_age, pose_mono_age, odom_ros_age, odom_mono_age) < -_EPS:
            return "pose_or_odometry_from_future"
        if pose_ros_age > self._config.pose_max_age_s + _EPS or pose_mono_age > self._config.pose_max_age_s + _EPS:
            return "pose_stale"
        if odom_ros_age > self._config.odom_max_age_s + _EPS or odom_mono_age > self._config.odom_max_age_s + _EPS:
            return "odometry_stale"
        vals = (
            pose.pose.x_m,
            pose.pose.y_m,
            pose.pose.yaw_rad,
            odom.linear_x_mps,
            odom.linear_y_mps,
            odom.angular_z_radps,
        )
        if not all(math.isfinite(float(v)) for v in vals):
            return "pose_or_odometry_nonfinite"
        return None

    def _validate_current_geometry(self, pose: Pose2D, segment: RouteSegment) -> str | None:
        if not segment.path.poses:
            return "segment_path_empty"
        if not segment.allowed_regions:
            return "segment_corridor_empty"
        distance = _distance_to_path(pose, segment.path.poses)
        if distance > self._config.path_lateral_tolerance_m + self._config.geometry_epsilon_m:
            return "pose_too_far_from_permitted_path"
        if not _footprint_inside_regions(
            pose,
            self._config.footprint,
            segment.allowed_regions,
            self._config.footprint_edge_sample_m,
            self._config.geometry_epsilon_m,
        ):
            return "footprint_outside_corridor"
        if _footprint_crosses_plane(
            pose,
            self._config.footprint,
            segment.terminal_limit,
            self._config.geometry_epsilon_m,
        ):
            return "terminal_limit_crossed"
        return None

    def _validate_command(
        self,
        cmd: RawCommand,
        inputs: GuardInputs,
        segment: RouteSegment,
        now_monotonic: float,
    ) -> str | None:
        vals = (
            cmd.stamp_s,
            cmd.receipt_monotonic_s,
            cmd.linear_x_mps,
            cmd.linear_y_mps,
            cmd.angular_z_radps,
        )
        if not all(math.isfinite(float(v)) for v in vals):
            return "raw_command_nonfinite"
        ros_age = float(inputs.now_ros_s) - float(cmd.stamp_s)
        mono_age = now_monotonic - float(cmd.receipt_monotonic_s)
        if ros_age < -_EPS or mono_age < -_EPS:
            return "raw_command_from_future"
        if ros_age > self._config.raw_command_max_age_s + _EPS or mono_age > self._config.raw_command_max_age_s + _EPS:
            return "raw_command_stale"
        if cmd.receipt_monotonic_s + _EPS < self._generation_started_monotonic_s:
            return "raw_command_from_previous_generation"
        linear = math.hypot(float(cmd.linear_x_mps), float(cmd.linear_y_mps))
        allowed_linear = min(self._config.hard_max_linear_speed_mps, float(segment.max_speed_mps))
        allowed_yaw = min(self._config.hard_max_yaw_rate_radps, float(segment.max_yaw_rate_radps))
        if linear > allowed_linear + _EPS:
            return "linear_speed_limit_exceeded"
        if abs(float(cmd.angular_z_radps)) > allowed_yaw + _EPS:
            return "yaw_rate_limit_exceeded"
        return None

    def _validate_braking_rollout(
        self,
        pose: Pose2D,
        odom: OdomSample,
        cmd: RawCommand,
        segment: RouteSegment,
    ) -> str | None:
        measured_speed = math.hypot(float(odom.linear_x_mps), float(odom.linear_y_mps))
        command_speed = math.hypot(float(cmd.linear_x_mps), float(cmd.linear_y_mps))
        speed = max(measured_speed, command_speed)
        omega = max(abs(float(odom.angular_z_radps)), abs(float(cmd.angular_z_radps)))

        if speed <= _EPS and omega <= _EPS:
            return None

        stop_time = self._config.braking_latency_s + speed / self._config.braking_deceleration_mps2
        stop_distance = (
            speed * self._config.braking_latency_s
            + (speed * speed) / (2.0 * self._config.braking_deceleration_mps2)
        )

        bx = float(cmd.linear_x_mps)
        by = float(cmd.linear_y_mps)
        if math.hypot(bx, by) <= _EPS:
            bx = float(odom.linear_x_mps)
            by = float(odom.linear_y_mps)
        body_norm = math.hypot(bx, by)
        if body_norm <= _EPS:
            ux_world = 0.0
            uy_world = 0.0
        else:
            bx /= body_norm
            by /= body_norm
            c = math.cos(pose.yaw_rad)
            s = math.sin(pose.yaw_rad)
            ux_world = c * bx - s * by
            uy_world = s * bx + c * by

        samples = max(1, int(math.ceil(stop_distance / self._config.rollout_step_m)))
        for i in range(1, samples + 1):
            fraction = i / samples
            predicted = Pose2D(
                x_m=pose.x_m + ux_world * stop_distance * fraction,
                y_m=pose.y_m + uy_world * stop_distance * fraction,
                yaw_rad=_wrap_angle(pose.yaw_rad + float(cmd.angular_z_radps) * stop_time * fraction),
            )
            if not _footprint_inside_regions(
                predicted,
                self._config.footprint,
                segment.allowed_regions,
                self._config.footprint_edge_sample_m,
                self._config.geometry_epsilon_m,
            ):
                return "braking_rollout_leaves_corridor"
            if _footprint_crosses_plane(
                predicted,
                self._config.footprint,
                segment.terminal_limit,
                self._config.geometry_epsilon_m,
            ):
                return "braking_rollout_crosses_terminal_limit"
        return None

    def _measured_stopped(self, odom: OdomSample | None) -> bool:
        if odom is None:
            return False
        values = (odom.linear_x_mps, odom.linear_y_mps, odom.angular_z_radps)
        if not all(math.isfinite(float(v)) for v in values):
            return False
        return (
            math.hypot(float(odom.linear_x_mps), float(odom.linear_y_mps))
            <= self._config.stationary_linear_max_mps + _EPS
            and abs(float(odom.angular_z_radps))
            <= self._config.stationary_angular_max_radps + _EPS
        )

    def _hold(self, inputs: GuardInputs, reason: str, fault: bool = False) -> GuardDecision:
        if fault:
            self._fault_latched = True
            self._fault_reason = reason
        return self._zero(
            GuardDecisionKind.FAULT if fault else GuardDecisionKind.HOLD,
            inputs,
            stopped=self._measured_stopped(inputs.odom),
            reason=reason,
        )

    def _zero(
        self,
        kind: GuardDecisionKind,
        inputs: GuardInputs,
        stopped: bool,
        reason: str,
    ) -> GuardDecision:
        segment_id = inputs.permit.segment.segment_id if inputs.permit is not None else ""
        committed = False
        if (
            inputs.permit is not None
            and inputs.pose is not None
            and inputs.permit.segment.commitment_gate is not None
        ):
            committed = _footprint_crosses_plane(
                inputs.pose.pose,
                self._config.footprint,
                inputs.permit.segment.commitment_gate,
                self._config.geometry_epsilon_m,
            )
        return GuardDecision(
            kind=kind,
            motion_allowed=False,
            linear_x_mps=0.0,
            linear_y_mps=0.0,
            angular_z_radps=0.0,
            stopped=stopped,
            committed=committed,
            active_segment_id=segment_id,
            last_permission_sequence=self._last_sequence,
            fault_latched=self._fault_latched,
            reason=reason,
        )


def _dependency_versions(snapshot: MapSnapshot) -> dict[str, int]:
    out: dict[str, int] = {}
    for item in snapshot.approaches:
        out[f"approach:{item.approach_id}"] = item.version
    for item in snapshot.targets:
        out[f"target:{item.target_id}"] = item.version
    for item in snapshot.edges:
        out[f"edge:{item.edge_id}"] = item.version
    for item in snapshot.coverage:
        out[f"coverage:{item.coverage_id}"] = item.version
    return out


def _segment_relevance_ids(segment: RouteSegment) -> set[str]:
    ids = {segment.approach_id, segment.terminal_id, *segment.traversed_edge_ids}
    for dep in segment.dependencies:
        ids.add(dep.dependency_id)
        if ":" in dep.dependency_id:
            ids.add(dep.dependency_id.split(":", 1)[1])
    return {x for x in ids if x}


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _transform_point(pose: Pose2D, point: Point2D) -> Point2D:
    c = math.cos(pose.yaw_rad)
    s = math.sin(pose.yaw_rad)
    return Point2D(
        pose.x_m + c * point.x_m - s * point.y_m,
        pose.y_m + s * point.x_m + c * point.y_m,
    )


def _footprint_world(pose: Pose2D, footprint: Polygon2D) -> tuple[Point2D, ...]:
    return tuple(_transform_point(pose, p) for p in footprint.points)


def _point_on_segment(p: Point2D, a: Point2D, b: Point2D, eps: float) -> bool:
    dx = b.x_m - a.x_m
    dy = b.y_m - a.y_m
    cross = (p.x_m - a.x_m) * dy - (p.y_m - a.y_m) * dx
    if abs(cross) > eps * max(1.0, math.hypot(dx, dy)):
        return False
    dot = (p.x_m - a.x_m) * dx + (p.y_m - a.y_m) * dy
    if dot < -eps:
        return False
    if dot > dx * dx + dy * dy + eps:
        return False
    return True


def _point_in_polygon(point: Point2D, polygon: Polygon2D, eps: float) -> bool:
    pts = polygon.points
    if len(pts) < 3:
        return False
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        a = pts[j]
        b = pts[i]
        if _point_on_segment(point, a, b, eps):
            return True
        intersects = (
            (b.y_m > point.y_m) != (a.y_m > point.y_m)
            and point.x_m
            < (a.x_m - b.x_m) * (point.y_m - b.y_m) / ((a.y_m - b.y_m) or 1e-300) + b.x_m
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def _point_in_any_region(point: Point2D, regions: Iterable[Polygon2D], eps: float) -> bool:
    return any(_point_in_polygon(point, polygon, eps) for polygon in regions)


def _footprint_inside_regions(
    pose: Pose2D,
    footprint: Polygon2D,
    regions: tuple[Polygon2D, ...],
    edge_sample_m: float,
    eps: float,
) -> bool:
    world = _footprint_world(pose, footprint)
    if not world:
        return False
    # Sample vertices and the complete boundary.  This catches a footprint edge
    # leaving a concave/simple permitted union even when both endpoints are in.
    for i, a in enumerate(world):
        b = world[(i + 1) % len(world)]
        length = math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)
        n = max(1, int(math.ceil(length / edge_sample_m)))
        for k in range(n + 1):
            t = k / n
            p = Point2D(
                a.x_m + (b.x_m - a.x_m) * t,
                a.y_m + (b.y_m - a.y_m) * t,
            )
            if not _point_in_any_region(p, regions, eps):
                return False
    return True


def _distance_point_to_segment(px: float, py: float, a: Pose2D, b: Pose2D) -> float:
    vx = b.x_m - a.x_m
    vy = b.y_m - a.y_m
    wx = px - a.x_m
    wy = py - a.y_m
    vv = vx * vx + vy * vy
    if vv <= _EPS:
        return math.hypot(wx, wy)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
    qx = a.x_m + t * vx
    qy = a.y_m + t * vy
    return math.hypot(px - qx, py - qy)


def _distance_to_path(pose: Pose2D, path: tuple[Pose2D, ...]) -> float:
    if len(path) == 1:
        return math.hypot(pose.x_m - path[0].x_m, pose.y_m - path[0].y_m)
    return min(
        _distance_point_to_segment(pose.x_m, pose.y_m, path[i], path[i + 1])
        for i in range(len(path) - 1)
    )


def _plane_local_x(point: Point2D, plane: Pose2D) -> float:
    dx = point.x_m - plane.x_m
    dy = point.y_m - plane.y_m
    return math.cos(plane.yaw_rad) * dx + math.sin(plane.yaw_rad) * dy


def _footprint_crosses_plane(
    pose: Pose2D,
    footprint: Polygon2D,
    plane: Pose2D | None,
    eps: float,
) -> bool:
    if plane is None:
        return False
    return any(_plane_local_x(p, plane) > eps for p in _footprint_world(pose, footprint))
