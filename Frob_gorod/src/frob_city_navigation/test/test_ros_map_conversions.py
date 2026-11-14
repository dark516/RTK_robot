import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from frob_city_interfaces.msg import SignDetection, SignDetectionArray
from frob_city_interfaces.srv import PlanRoute

from frob_city_navigation.models import (
    ApproachRecord,
    ApproachResolution,
    CorePlanResult,
    CoverageRecord,
    CoverageResolution,
    EdgeBlockState,
    EdgeRecord,
    FieldTransform,
    KnownSignRecord,
    MapSnapshot,
    MetricPath,
    MovementMask,
    PlanStatus,
    Point2D,
    Polygon2D,
    Pose2D,
    RouteDependency,
    RouteSegment,
    SegmentKind,
    SignClass,
    TargetRecord,
    TargetResolution,
    TerminalKind,
)
from frob_city_navigation.ros_map_conversions import (
    detection_array_to_core,
    fill_plan_response,
    map_snapshot_to_msg,
    odometry_to_pose_sample,
    pose2d_to_pose_stamped,
    pose_stamped_to_pose2d,
    route_segment_to_msg,
)


def stamp(sec=10, nanosec=0):
    return Time(sec=sec, nanosec=nanosec)


def test_pose_round_trip_and_wrong_frame_rejected():
    pose = Pose2D(1.2, -0.4, 0.7)
    msg = pose2d_to_pose_stamped(pose, "map", stamp())
    out = pose_stamped_to_pose2d(msg, "map")
    assert out.x_m == pytest.approx(pose.x_m)
    assert out.y_m == pytest.approx(pose.y_m)
    assert out.yaw_rad == pytest.approx(pose.yaw_rad)
    with pytest.raises(ValueError):
        pose_stamped_to_pose2d(msg, "odom")


def test_odometry_rejects_non_map_pose_and_bad_quaternion():
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.header.stamp = stamp()
    msg.pose.pose.orientation.w = 1.0
    with pytest.raises(ValueError):
        odometry_to_pose_sample(msg, "map")

    msg.header.frame_id = "map"
    msg.pose.pose.orientation.w = 0.2
    with pytest.raises(ValueError):
        odometry_to_pose_sample(msg, "map")


def test_detection_array_preserves_capture_time_range_convention_and_uncertainty():
    msg = SignDetectionArray()
    msg.header.frame_id = "camera_optical_frame"
    msg.header.stamp = stamp(5, 250_000_000)
    msg.detector_instance_id = "det-A"
    msg.frame_sequence = 42
    msg.image_width_px = 640
    msg.image_height_px = 480
    msg.calibration_id = "cal-1"
    msg.frame_valid = True
    det = SignDetection()
    det.sign_class = SignDetection.PARKING
    det.confidence = 0.9
    det.x_min_px = 100
    det.y_min_px = 80
    det.width_px = 30
    det.height_px = 40
    det.range_valid = True
    det.range_convention = SignDetection.RAY_RANGE
    det.range_m = 1.25
    det.range_stddev_m = 0.08
    det.detector_track_id = "diag-track"
    msg.detections = [det]
    out = detection_array_to_core(msg)
    assert len(out) == 1
    obs = out[0]
    assert obs.timestamp_s == pytest.approx(5.25)
    assert obs.sign_class is SignClass.PARKING
    assert obs.range_m == pytest.approx(1.25)
    assert obs.range_stddev_m == pytest.approx(0.08)
    assert obs.detector_track_id == "diag-track"


def test_map_snapshot_conversion_is_complete_and_order_preserving():
    snapshot = MapSnapshot(
        run_id="run-1",
        map_instance_id="map-1",
        field_id="field-1",
        revision=7,
        ready=True,
        perception_healthy=True,
        localization_healthy=True,
        precautionary_hold_ids=("approach:A",),
        approaches=(ApproachRecord(
            "A", 2, ApproachResolution.RESOLVED_SIGNED,
            frozenset({SignClass.NO_LEFT}), MovementMask.STRAIGHT | MovementMask.RIGHT,
            4.0, True, "ok"),),
        known_signs=(KnownSignRecord(
            "S", "F", "A", SignClass.NO_LEFT, 1, 3.5, 0.91),),
        targets=(TargetRecord(
            "BUS_STOP:S:F", 1, SignClass.BUS_STOP, TargetResolution.CONFIRMED,
            "S", "F", "E", "anchor", Pose2D(1,2,0), True,
            Pose2D(0.8,2,0), Pose2D(0.9,2,0), 0.05, 0.1, 0.15, 3.0),),
        edges=(EdgeRecord("E", 0, EdgeBlockState.OPEN, 3, None, ""),),
        coverage=(CoverageRecord("OBS", 1, CoverageResolution.COMPLETE, 0, 4.0, ""),),
        reason="",
    )
    msg = map_snapshot_to_msg(
        snapshot, frame_id="map", stamp=stamp(), ready_override=False,
        perception_healthy_override=False, reason_override="degraded",
        field_to_map=FieldTransform(10.0, -2.0, 0.0, True))
    assert msg.run_id == "run-1"
    assert msg.revision == 7
    assert not msg.ready
    assert not msg.perception_healthy
    assert msg.reason == "degraded"
    assert msg.approaches[0].allowed_movement_mask == 5
    assert msg.known_signs[0].site_id == "S"
    assert msg.known_signs[0].face_id == "F"
    assert msg.known_signs[0].sign_class == int(SignClass.NO_LEFT)
    assert msg.targets[0].target_id == "BUS_STOP:S:F"
    assert msg.targets[0].sign_pose.pose.position.x == pytest.approx(11.0)
    assert msg.targets[0].sign_pose.pose.position.y == pytest.approx(0.0)
    assert msg.targets[0].service_pose.pose.position.x == pytest.approx(10.8)
    assert msg.targets[0].terminal_limit.pose.position.x == pytest.approx(10.9)
    assert msg.edges[0].completed_traversals == 3
    assert msg.coverage[0].coverage_id == "OBS"


def _segment():
    return RouteSegment(
        segment_id="seg-1",
        kind=SegmentKind.CORRIDOR,
        from_anchor_id="A",
        to_anchor_id="B",
        approach_id="",
        selected_movement=MovementMask.NONE,
        path=MetricPath((Pose2D(0,0,0), Pose2D(1,0,0))),
        allowed_regions=(Polygon2D((Point2D(-1,-1), Point2D(2,-1), Point2D(2,1), Point2D(-1,1))),),
        dependencies=(RouteDependency("edge:E", 0),),
        traversed_edge_ids=("E",),
        terminal_kind=TerminalKind.HOLD,
        terminal_id="B",
        terminal_limit=Pose2D(1,0,0),
        commitment_gate=None,
        max_speed_mps=0.15,
        max_yaw_rate_radps=0.6,
        xy_tolerance_m=0.05,
        yaw_tolerance_rad=0.1,
        estimated_duration_s=6.0,
    )


def test_route_segment_and_plan_response_use_checked_path_contract():
    segment = _segment()
    msg = route_segment_to_msg(segment, "map", stamp())
    assert msg.segment_id == "seg-1"
    assert len(msg.path.poses) == 2
    assert msg.dependency_ids == ["edge:E"]
    assert list(msg.dependency_versions) == [0]
    assert not msg.commitment_gate_valid

    result = CorePlanResult(
        status=PlanStatus.ROUTE_READY,
        request_id=5,
        run_id="run",
        map_instance_id="map-i",
        revision=2,
        route_id="route",
        selected_target_id="",
        start_pose=Pose2D(0,0,0),
        reaches_requested_target=False,
        estimated_total_cost_s=6.0,
        cost_uses_unknown_gates=False,
        executable_prefix=(segment,),
    )
    response = PlanRoute.Response()
    fill_plan_response(result, response, frame_id="map", stamp=stamp())
    assert response.status == PlanRoute.Response.ROUTE_READY
    assert response.route_id == "route"
    assert response.executable_prefix[0].segment_id == "seg-1"


def test_route_segment_is_transformed_from_field_coordinates_to_map():
    segment = _segment()
    transform = FieldTransform(x_m=10.0, y_m=-2.0, yaw_rad=math.pi / 2.0, valid=True)

    msg = route_segment_to_msg(
        segment,
        "map",
        stamp(),
        field_to_map=transform,
    )

    start = msg.path.poses[0].pose.position
    end = msg.path.poses[-1].pose.position
    assert (start.x, start.y) == pytest.approx((10.0, -2.0))
    assert (end.x, end.y) == pytest.approx((10.0, -1.0))
    assert msg.terminal_limit.pose.position.x == pytest.approx(10.0)
    assert msg.terminal_limit.pose.position.y == pytest.approx(-1.0)
    polygon = msg.allowed_regions[0].polygon.points
    assert (polygon[0].x, polygon[0].y) == pytest.approx((11.0, -3.0))
