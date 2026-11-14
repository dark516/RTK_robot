from dataclasses import replace

import pytest

from frob_city_navigation.models import (
    ApproachRecord,
    ApproachResolution,
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
    TerminalKind,
)
from frob_city_navigation.motion_guard_core import (
    GuardConfig,
    GuardFault,
    GuardInputs,
    GuardPermit,
    MotionGuardCore,
    OdomSample,
    RawCommand,
    ResetContext,
)


NOW_ROS = 100.0
NOW_MONO = 50.0
RUN = "RUN"
MAP = "MAP"
NAV = "NAV"
FIELD = "FIELD"
APP = "APP_0"


def footprint():
    return Polygon2D((
        Point2D(-0.10, -0.10),
        Point2D(0.10, -0.10),
        Point2D(0.10, 0.10),
        Point2D(-0.10, 0.10),
    ))


def corridor(y_half=0.50, x_max=3.0):
    return Polygon2D((
        Point2D(-1.0, -y_half),
        Point2D(x_max, -y_half),
        Point2D(x_max, y_half),
        Point2D(-1.0, y_half),
    ))


def config(**kw):
    values = dict(
        field_id=FIELD,
        footprint=footprint(),
        braking_deceleration_mps2=1.0,
        braking_latency_s=0.05,
        hard_max_linear_speed_mps=0.40,
        hard_max_yaw_rate_radps=1.0,
        path_lateral_tolerance_m=0.20,
    )
    values.update(kw)
    return GuardConfig(**values)


def segment(**kw):
    values = dict(
        segment_id="SEG_1",
        kind=SegmentKind.CORRIDOR,
        from_anchor_id="A",
        to_anchor_id="B",
        approach_id=APP,
        selected_movement=MovementMask.NONE,
        path=MetricPath((Pose2D(0.0, 0.0, 0.0), Pose2D(2.2, 0.0, 0.0))),
        allowed_regions=(corridor(),),
        dependencies=(RouteDependency(f"approach:{APP}", 1),),
        traversed_edge_ids=("E0",),
        terminal_kind=TerminalKind.HOLD,
        terminal_id="TERM",
        terminal_limit=Pose2D(2.50, 0.0, 0.0),
        commitment_gate=Pose2D(1.00, 0.0, 0.0),
        max_speed_mps=0.30,
        max_yaw_rate_radps=0.80,
        xy_tolerance_m=0.05,
        yaw_tolerance_rad=0.10,
        estimated_duration_s=5.0,
    )
    values.update(kw)
    return RouteSegment(**values)


def snapshot(**kw):
    values = dict(
        run_id=RUN,
        map_instance_id=MAP,
        field_id=FIELD,
        revision=1,
        ready=True,
        perception_healthy=True,
        localization_healthy=True,
        precautionary_hold_ids=(),
        approaches=(ApproachRecord(
            approach_id=APP,
            version=1,
            state=ApproachResolution.RESOLVED_CLEAR,
            confirmed_signs=frozenset(),
            allowed_movements=MovementMask.STRAIGHT,
            last_qualified_observation_s=99.0,
            precautionary_hold=False,
            reason="",
        ),),
        targets=(),
        edges=(),
        coverage=(),
        reason="",
    )
    values.update(kw)
    return MapSnapshot(**values)


def permit(seg=None, **kw):
    values = dict(
        stamp_s=NOW_ROS - 0.02,
        receipt_monotonic_s=NOW_MONO - 0.02,
        run_id=RUN,
        map_instance_id=MAP,
        navigator_instance_id=NAV,
        field_id=FIELD,
        goal_generation=1,
        sequence=1,
        enabled=True,
        valid_for_s=0.20,
        segment=seg or segment(),
    )
    values.update(kw)
    return GuardPermit(**values)


def command(**kw):
    values = dict(
        stamp_s=NOW_ROS - 0.01,
        receipt_monotonic_s=NOW_MONO - 0.01,
        linear_x_mps=0.10,
        linear_y_mps=0.0,
        angular_z_radps=0.0,
    )
    values.update(kw)
    return RawCommand(**values)


def odom(**kw):
    values = dict(
        timestamp_s=NOW_ROS - 0.01,
        receipt_monotonic_s=NOW_MONO - 0.01,
        linear_x_mps=0.10,
        linear_y_mps=0.0,
        angular_z_radps=0.0,
    )
    values.update(kw)
    return OdomSample(**values)


def inputs(**kw):
    values = dict(
        now_ros_s=NOW_ROS,
        run_id=RUN,
        map_instance_id=MAP,
        navigator_instance_id=NAV,
        field_id=FIELD,
        permit=permit(),
        raw_command=command(),
        map_snapshot=snapshot(),
        map_stamp_s=NOW_ROS - 0.05,
        map_receipt_monotonic_s=NOW_MONO - 0.05,
        pose=PoseSample(NOW_ROS - 0.01, Pose2D(0.50, 0.0, 0.0), 0.10, 0.0),
        pose_receipt_monotonic_s=NOW_MONO - 0.01,
        odom=odom(),
        emergency_stop_active=False,
    )
    values.update(kw)
    return GuardInputs(**values)


def test_valid_fresh_contract_forwards_controller_command():
    core = MotionGuardCore(config())
    d = core.evaluate(inputs(), NOW_MONO)
    assert d.motion_allowed
    assert d.linear_x_mps == pytest.approx(0.10)
    assert d.reason == "motion_permitted"


@pytest.mark.parametrize("change,reason", [
    ({"permit": None}, "permit_missing"),
    ({"permit": permit(enabled=False)}, "permit_disabled"),
    ({"emergency_stop_active": True}, "emergency_stop_active"),
])
def test_missing_disabled_or_emergency_stays_zero(change, reason):
    d = MotionGuardCore(config()).evaluate(inputs(**change), NOW_MONO)
    assert not d.motion_allowed
    assert (d.linear_x_mps, d.linear_y_mps, d.angular_z_radps) == (0.0, 0.0, 0.0)
    assert d.reason == reason


def test_excessive_permit_lifetime_is_rejected():
    p = permit(valid_for_s=0.30)
    d = MotionGuardCore(config()).evaluate(inputs(permit=p), NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == "permit_lifetime_excessive"


def test_wrong_permit_identity_is_rejected():
    p = permit(run_id="OLD")
    d = MotionGuardCore(config()).evaluate(inputs(permit=p), NOW_MONO)
    assert d.reason == "permit_identity_mismatch"


@pytest.mark.parametrize("p,reason", [
    (permit(stamp_s=NOW_ROS - 0.25), "permit_ros_stale"),
    (permit(receipt_monotonic_s=NOW_MONO - 0.25), "permit_receipt_stale"),
])
def test_stale_permit_is_zero(p, reason):
    d = MotionGuardCore(config()).evaluate(inputs(permit=p), NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == reason


def test_cached_same_permission_can_remain_active_until_expiry():
    core = MotionGuardCore(config())
    i = inputs()
    assert core.evaluate(i, NOW_MONO).motion_allowed
    assert core.evaluate(i, NOW_MONO + 0.01).motion_allowed


def test_duplicate_reception_does_not_refresh_permission():
    core = MotionGuardCore(config())
    assert core.evaluate(inputs(), NOW_MONO).motion_allowed
    duplicated = permit(receipt_monotonic_s=NOW_MONO - 0.005)
    d = core.evaluate(inputs(permit=duplicated), NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == "permit_duplicate_replayed"


def test_reordered_permission_sequence_is_rejected():
    core = MotionGuardCore(config())
    p5 = permit(sequence=5)
    assert core.evaluate(inputs(permit=p5), NOW_MONO).motion_allowed
    p4 = permit(sequence=4)
    d = core.evaluate(inputs(permit=p4), NOW_MONO)
    assert d.reason == "permit_sequence_reordered"


def test_new_goal_generation_flushes_previous_raw_command():
    core = MotionGuardCore(config())
    assert core.evaluate(inputs(), NOW_MONO).motion_allowed
    p2 = permit(sequence=2, goal_generation=2, receipt_monotonic_s=NOW_MONO)
    old_cmd = command(receipt_monotonic_s=NOW_MONO - 0.01)
    d = core.evaluate(inputs(permit=p2, raw_command=old_cmd), NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == "raw_command_from_previous_generation"
    fresh_cmd = command(receipt_monotonic_s=NOW_MONO, stamp_s=NOW_ROS)
    assert core.evaluate(inputs(permit=p2, raw_command=fresh_cmd), NOW_MONO).motion_allowed


@pytest.mark.parametrize("change,reason", [
    ({"map_stamp_s": NOW_ROS - 2.0}, "map_snapshot_stale"),
    ({"map_snapshot": snapshot(ready=False)}, "map_not_ready"),
    ({"map_snapshot": snapshot(perception_healthy=False)}, "perception_unhealthy"),
    ({"map_snapshot": snapshot(localization_healthy=False)}, "localization_unhealthy"),
])
def test_bad_map_state_stays_zero(change, reason):
    d = MotionGuardCore(config()).evaluate(inputs(**change), NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == reason


def test_changed_dependency_stops_motion():
    app = replace(snapshot().approaches[0], version=2)
    d = MotionGuardCore(config()).evaluate(
        inputs(map_snapshot=snapshot(approaches=(app,))), NOW_MONO
    )
    assert not d.motion_allowed
    assert d.reason == f"dependency_changed:approach:{APP}"


def test_missing_dependency_stops_motion():
    d = MotionGuardCore(config()).evaluate(
        inputs(map_snapshot=snapshot(approaches=())), NOW_MONO
    )
    assert d.reason == f"dependency_missing:approach:{APP}"


def test_applicable_precautionary_hold_stops_but_unrelated_hold_does_not():
    core = MotionGuardCore(config())
    d = core.evaluate(inputs(map_snapshot=snapshot(precautionary_hold_ids=(APP,))), NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == f"precautionary_hold:{APP}"

    core = MotionGuardCore(config())
    d = core.evaluate(inputs(map_snapshot=snapshot(precautionary_hold_ids=("OTHER",))), NOW_MONO)
    assert d.motion_allowed


@pytest.mark.parametrize("change,reason", [
    ({"pose": PoseSample(NOW_ROS - 1.0, Pose2D(0.5, 0.0, 0.0))}, "pose_stale"),
    ({"odom": odom(timestamp_s=NOW_ROS - 1.0)}, "odometry_stale"),
    ({"raw_command": command(stamp_s=NOW_ROS - 1.0)}, "raw_command_stale"),
])
def test_stale_mandatory_inputs_are_zero(change, reason):
    d = MotionGuardCore(config()).evaluate(inputs(**change), NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == reason


@pytest.mark.parametrize("cmd,reason", [
    (command(linear_x_mps=0.31), "linear_speed_limit_exceeded"),
    (command(angular_z_radps=0.81), "yaw_rate_limit_exceeded"),
    (command(linear_x_mps=float("nan")), "raw_command_nonfinite"),
])
def test_command_limits_and_nonfinite_values_fail_closed(cmd, reason):
    d = MotionGuardCore(config()).evaluate(inputs(raw_command=cmd), NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == reason


def test_pose_too_far_from_path_is_rejected():
    p = PoseSample(NOW_ROS - 0.01, Pose2D(0.5, 0.30, 0.0), 0.0, 0.0)
    d = MotionGuardCore(config()).evaluate(inputs(pose=p), NOW_MONO)
    assert d.reason == "pose_too_far_from_permitted_path"


def test_full_footprint_outside_corridor_is_rejected():
    p = PoseSample(NOW_ROS - 0.01, Pose2D(0.5, 0.16, 0.0), 0.0, 0.0)
    seg = segment(allowed_regions=(corridor(y_half=0.20),))
    d = MotionGuardCore(config(path_lateral_tolerance_m=0.30)).evaluate(
        inputs(pose=p, permit=permit(seg=seg)), NOW_MONO
    )
    assert d.reason == "footprint_outside_corridor"


def test_current_terminal_plane_crossing_is_rejected():
    p = PoseSample(NOW_ROS - 0.01, Pose2D(2.45, 0.0, 0.0), 0.0, 0.0)
    d = MotionGuardCore(config(path_lateral_tolerance_m=0.5)).evaluate(inputs(pose=p), NOW_MONO)
    assert d.reason == "terminal_limit_crossed"


def test_braking_rollout_must_fit_corridor():
    seg = segment(allowed_regions=(corridor(y_half=0.13),))
    p = PoseSample(NOW_ROS - 0.01, Pose2D(0.5, 0.0, 0.0), 0.25, 0.0)
    cmd = command(linear_x_mps=0.0, linear_y_mps=0.25)
    o = odom(linear_x_mps=0.0, linear_y_mps=0.25)
    d = MotionGuardCore(config(path_lateral_tolerance_m=0.30)).evaluate(
        inputs(pose=p, raw_command=cmd, odom=o, permit=permit(seg=seg)), NOW_MONO
    )
    assert not d.motion_allowed
    assert d.reason == "braking_rollout_leaves_corridor"


def test_braking_rollout_must_not_cross_terminal_limit():
    p = PoseSample(NOW_ROS - 0.01, Pose2D(2.35, 0.0, 0.0), 0.30, 0.0)
    cmd = command(linear_x_mps=0.30)
    o = odom(linear_x_mps=0.30)
    d = MotionGuardCore(config(path_lateral_tolerance_m=0.5)).evaluate(
        inputs(pose=p, raw_command=cmd, odom=o), NOW_MONO
    )
    assert not d.motion_allowed
    assert d.reason == "braking_rollout_crosses_terminal_limit"


def test_commitment_gate_state_is_reported_without_inventing_route_change():
    core = MotionGuardCore(config(path_lateral_tolerance_m=0.5))
    before = inputs(pose=PoseSample(NOW_ROS - 0.01, Pose2D(0.80, 0.0, 0.0), 0.0, 0.0),
                    raw_command=command(linear_x_mps=0.0), odom=odom(linear_x_mps=0.0))
    assert not core.evaluate(before, NOW_MONO).committed

    # Fresh heartbeat with a new sequence, same generation.
    p2 = permit(sequence=2, receipt_monotonic_s=NOW_MONO)
    after = inputs(
        permit=p2,
        pose=PoseSample(NOW_ROS, Pose2D(1.05, 0.0, 0.0), 0.0, 0.0),
        pose_receipt_monotonic_s=NOW_MONO,
        raw_command=command(stamp_s=NOW_ROS, receipt_monotonic_s=NOW_MONO, linear_x_mps=0.0),
        odom=odom(timestamp_s=NOW_ROS, receipt_monotonic_s=NOW_MONO, linear_x_mps=0.0),
    )
    assert core.evaluate(after, NOW_MONO).committed


def test_fault_latch_requires_measured_stopped_reset():
    core = MotionGuardCore(config())
    d = core.latch_fault(GuardFault.GEOMETRY_VIOLATION)
    assert d.fault_latched and not d.motion_allowed
    d = core.evaluate(inputs(), NOW_MONO)
    assert d.fault_latched and not d.motion_allowed

    r = core.reset_while_stopped(ResetContext(RUN, MAP, NAV, measured_stopped=False))
    assert not r.accepted and r.fault_latched
    r = core.reset_while_stopped(ResetContext(RUN, MAP, NAV, measured_stopped=True))
    assert r.accepted and not r.fault_latched


def test_missing_odometry_never_claims_stopped():
    d = MotionGuardCore(config()).evaluate(inputs(odom=None), NOW_MONO)
    assert not d.motion_allowed
    assert not d.stopped
    assert d.reason == "odometry_missing"


def test_session_change_requires_explicit_stopped_reset():
    core = MotionGuardCore(config())
    assert core.evaluate(inputs(), NOW_MONO).motion_allowed
    changed = inputs(run_id="RUN2", permit=permit(run_id="RUN2"), map_snapshot=snapshot(run_id="RUN2"))
    d = core.evaluate(changed, NOW_MONO)
    assert not d.motion_allowed
    assert d.reason == "current_session_changed_without_reset"


def test_float32_quantized_max_permit_lifetime_is_accepted():
    # MotionPermit.valid_for_s is float32 in the ROS interface.  Exactly 0.20
    # seconds becomes slightly larger when represented on the wire.
    import struct

    wire_value = struct.unpack("f", struct.pack("f", 0.20))[0]
    assert wire_value > 0.20

    p = permit(valid_for_s=wire_value)
    d = MotionGuardCore(config(max_permit_lifetime_s=0.20)).evaluate(
        inputs(permit=p), NOW_MONO
    )
    assert d.motion_allowed
    assert d.reason == "motion_permitted"


def test_meaningfully_excessive_permit_lifetime_still_rejected_after_wire_tolerance():
    p = permit(valid_for_s=0.20001)
    d = MotionGuardCore(config(max_permit_lifetime_s=0.20)).evaluate(
        inputs(permit=p), NOW_MONO
    )
    assert not d.motion_allowed
    assert d.reason == "permit_lifetime_excessive"
