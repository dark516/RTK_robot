from frob_city_navigation.models import (
    EdgeBlockState, EdgeRecord, MapSnapshot, MetricPath, MovementMask,
    Point2D, Polygon2D, Pose2D, RouteDependency, RouteSegment, SegmentKind,
    TerminalKind,
)
from frob_city_navigation.ros_execution_conversions import (
    map_state_from_msg, motion_permit_from_msg, route_segment_from_msg,
)
from frob_city_navigation.ros_map_conversions import map_snapshot_to_msg, route_segment_to_msg, seconds_to_time
from frob_city_interfaces.msg import MotionPermit


def _segment():
    return RouteSegment(
        segment_id="SEG",
        kind=SegmentKind.CORRIDOR,
        from_anchor_id="A",
        to_anchor_id="B",
        approach_id="APP",
        selected_movement=MovementMask.STRAIGHT,
        path=MetricPath((Pose2D(0, 0, 0), Pose2D(1, 0, 0))),
        allowed_regions=(Polygon2D((Point2D(-1,-1), Point2D(2,-1), Point2D(2,1), Point2D(-1,1))),),
        dependencies=(RouteDependency("edge:E0", 1),),
        traversed_edge_ids=("E0",),
        terminal_kind=TerminalKind.HOLD,
        terminal_id="",
        terminal_limit=Pose2D(1.5, 0, 0),
        commitment_gate=None,
        max_speed_mps=0.3,
        max_yaw_rate_radps=0.5,
        xy_tolerance_m=0.05,
        yaw_tolerance_rad=0.1,
        estimated_duration_s=2.0,
    )


def test_route_segment_roundtrip():
    msg = route_segment_to_msg(_segment(), "map", seconds_to_time(1.0))
    out = route_segment_from_msg(msg, map_frame="map")
    assert out == _segment()


def test_map_state_roundtrip_keeps_dependency_version():
    snap = MapSnapshot(
        run_id="R", map_instance_id="M", field_id="F", revision=3,
        ready=True, perception_healthy=True, localization_healthy=True,
        precautionary_hold_ids=(), approaches=(), targets=(),
        edges=(EdgeRecord("E0", 1, EdgeBlockState.OPEN, 0, None, ""),), coverage=(),
    )
    msg = map_snapshot_to_msg(snap, frame_id="map", stamp=seconds_to_time(1.0))
    out = map_state_from_msg(msg, map_frame="map")
    assert out == snap


def test_disabled_permission_does_not_require_segment_body():
    msg = MotionPermit()
    msg.header.stamp = seconds_to_time(2.0)
    msg.run_id = "R"
    msg.map_instance_id = "M"
    msg.navigator_instance_id = "N"
    msg.field_id = "F"
    msg.goal_generation = 1
    msg.sequence = 2
    msg.enabled = False
    msg.valid_for_s = 0.2
    permit = motion_permit_from_msg(msg, receipt_monotonic_s=5.0, map_frame="map")
    assert not permit.enabled
    assert permit.segment.segment_id == "DISABLED"
