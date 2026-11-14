from __future__ import annotations

from dataclasses import replace
from math import pi

from frob_city_navigation.field_model import FieldModel
from frob_city_navigation.graph import build_transitions
from frob_city_navigation.models import (
    ApproachRecord,
    ApproachResolution,
    CorePlanRequest,
    CoverageRecord,
    CoverageResolution,
    DirectedAnchorConfig,
    DirectedEdgeConfig,
    EdgeBlockState,
    EdgeRecord,
    FieldConfig,
    FieldTransform,
    GateConfig,
    GeometryMargins,
    MapSnapshot,
    MetricPath,
    MovementMask,
    NamedPolygon,
    NavigationConfig,
    NavigationTimeouts,
    ObservationRegionConfig,
    PlanGoalKind,
    PlanStatus,
    Point2D,
    Polygon2D,
    Pose2D,
    PoseSample,
    SignClass,
    SignFaceConfig,
    SignSiteConfig,
    TargetRecord,
    TargetResolution,
)
from frob_city_navigation.planner import RoutePlanner


def poly(*pts):
    return Polygon2D(tuple(Point2D(x, y) for x, y in pts))


def path(*pts):
    poses = []
    for i, (x, y) in enumerate(pts):
        if i + 1 < len(pts):
            nx, ny = pts[i + 1]
            import math
            yaw = math.atan2(ny - y, nx - x)
        elif poses:
            yaw = poses[-1].yaw_rad
        else:
            yaw = 0.0
        poses.append(Pose2D(x, y, yaw))
    return MetricPath(tuple(poses))


def navigation():
    return NavigationConfig(
        schema_version=2,
        map_frame="map",
        odom_frame="odom",
        base_frame="base_footprint",
        max_speed_mps=0.2,
        max_yaw_rate_radps=0.8,
        xy_tolerance_m=0.05,
        yaw_tolerance_rad=0.10,
        stationary_linear_mps=0.01,
        stationary_angular_radps=0.03,
        observation_attempts=2,
        segment_retry_budget=1,
        timeouts=NavigationTimeouts(0.2, 1.0, 0.2, 0.5, 0.1, 3.0, 1.0, 1.0, 30.0),
    )


def make_field(*, alternative=True, long_edge=False, platform_cut=False):
    anchors = [
        DirectedAnchorConfig("A", "C0_0", Pose2D(0.4, 0.4, 0.0)),
        DirectedAnchorConfig("B", "C1_0", Pose2D(1.2, 0.4, 0.0)),
        DirectedAnchorConfig("C", "C2_0", Pose2D(2.0, 0.4, 0.0)),
        DirectedAnchorConfig("T", "C3_0", Pose2D(2.8, 0.4, 0.0)),
    ]
    edges = [
        DirectedEdgeConfig("AB", "A", "B", MovementMask.STRAIGHT, path((0.4, 0.4), (1.2, 0.4)), ("ROAD",)),
        DirectedEdgeConfig("BC", "B", "C", MovementMask.STRAIGHT, path((1.2, 0.4), (2.0, 0.4)), ("ROAD",), "APP_B", "G_B"),
        DirectedEdgeConfig("CT", "C", "T", MovementMask.STRAIGHT, path((2.0, 0.4), (2.8, 0.4)), ("ROAD",)),
    ]
    if alternative:
        anchors.extend([
            DirectedAnchorConfig("D", "C0_1", Pose2D(0.4, 1.2, pi / 2)),
            DirectedAnchorConfig("E", "C1_1", Pose2D(1.2, 1.2, 0.0)),
        ])
        edges.extend([
            DirectedEdgeConfig("AD", "A", "D", MovementMask.LEFT, path((0.4, 0.4), (0.4, 1.2)), ("ROAD",)),
            DirectedEdgeConfig("DE", "D", "E", MovementMask.RIGHT, path((0.4, 1.2), (1.2, 1.2)), ("ROAD",)),
            DirectedEdgeConfig("EC", "E", "C", MovementMask.RIGHT, path((1.2, 1.2), (2.0, 0.4)), ("ROAD",)),
        ])
    if long_edge:
        edges.append(DirectedEdgeConfig("AC_LONG", "A", "C", MovementMask.STRAIGHT, path((0.4, 0.4), (1.2, 0.4), (2.0, 0.4)), ("ROAD",)))
    platforms = ()
    if platform_cut:
        platforms = (NamedPolygon("PLATFORM", poly((1.45, 0.25), (1.75, 0.25), (1.75, 0.55), (1.45, 0.55))),)
    return FieldConfig(
        schema_version=2,
        frame_id="map",
        width_m=3.2,
        height_m=2.0,
        geometry_calibrated=False,
        field_to_map=FieldTransform(0.0, 0.0, 0.0, False),
        footprint=poly((-0.15, -0.15), (0.15, -0.15), (0.15, 0.15), (-0.15, 0.15)),
        drivable_regions=(NamedPolygon("ROAD", poly((0.0, 0.0), (3.2, 0.0), (3.2, 1.6), (0.0, 1.6))),),
        platforms=platforms,
        anchors=tuple(anchors),
        edges=tuple(edges),
        observation_regions=(ObservationRegionConfig("OBS_B", poly((1.05, 0.2), (1.35, 0.2), (1.35, 0.6), (1.05, 0.6)), "B", 0.0, 0.3),),
        sign_sites=(SignSiteConfig("SITE_T", Point2D(2.6, 0.05), (SignFaceConfig("FACE_W", pi, ""),)),),
        gates=(GateConfig("G_B", Pose2D(1.35, 0.4, 0.0)),),
    )


def snapshot(field, *, approach_state=ApproachResolution.RESOLVED_CLEAR, allowed=MovementMask.STRAIGHT | MovementMask.LEFT | MovementMask.RIGHT, coverage=CoverageResolution.COMPLETE, block_bc=False, target=True):
    edges = tuple(
        EdgeRecord(e.edge_id, 1 if (block_bc and e.edge_id == "BC") else 0,
                   EdgeBlockState.TEMP_BLOCKED if (block_bc and e.edge_id == "BC") else EdgeBlockState.OPEN,
                   0, None, "")
        for e in field.edges
    )
    approaches = (
        ApproachRecord("APP_B", 1, approach_state, frozenset(), allowed, 1.0 if approach_state in (ApproachResolution.RESOLVED_CLEAR, ApproachResolution.RESOLVED_SIGNED) else None, False, ""),
    )
    coverage_rows = (CoverageRecord("OBS_B", 1 if coverage is CoverageResolution.COMPLETE else 0, coverage, 0, 1.0 if coverage is CoverageResolution.COMPLETE else None, ""),)
    targets = ()
    if target:
        targets = (TargetRecord(
            target_id="BUS_STOP:SITE_T:FACE_W",
            version=1,
            kind=SignClass.BUS_STOP,
            state=TargetResolution.CONFIRMED,
            site_id="SITE_T",
            face_id="FACE_W",
            directed_road_id="CT",
            service_anchor_id="service:BUS_STOP:SITE_T:FACE_W",
            sign_pose=Pose2D(2.6, 0.4, 0.0),
            geometry_valid=True,
            service_pose=Pose2D(2.45, 0.4, 0.0),
            terminal_limit=Pose2D(2.6, 0.4, 0.0),
            xy_tolerance_m=0.05,
            yaw_tolerance_rad=0.10,
            front_clearance_min_m=0.15,
            last_seen_s=1.0,
        ),)
    return MapSnapshot("run-1", "map-1", "field-1", 3, True, True, True, (), approaches, targets, edges, coverage_rows, "")


def request(kind=PlanGoalKind.TARGET, target="BUS_STOP:SITE_T:FACE_W", request_id=10):
    return CorePlanRequest("run-1", "map-1", request_id, 0, kind, target, ())


def start_pose():
    return PoseSample(1.0, Pose2D(0.4, 0.4, 0.0))


def test_legal_longer_route_beats_shorter_illegal_route():
    cfg = make_field(alternative=True)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg, approach_state=ApproachResolution.RESOLVED_SIGNED, allowed=MovementMask.RIGHT)
    result = RoutePlanner(field, navigation()).plan(request(), snap, start_pose())
    assert result.status is PlanStatus.ROUTE_READY
    traversed = [edge for seg in result.executable_prefix for edge in seg.traversed_edge_ids]
    assert "BC" not in traversed
    assert traversed[:3] == ["AD", "DE", "EC"]
    assert result.reaches_requested_target


def test_target_behind_unknown_gate_returns_only_prefix_before_gate():
    cfg = make_field(alternative=False)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg, approach_state=ApproachResolution.UNKNOWN, coverage=CoverageResolution.UNKNOWN)
    result = RoutePlanner(field, navigation()).plan(request(), snap, start_pose())
    assert result.status is PlanStatus.NEED_OBSERVATION
    assert [e for s in result.executable_prefix for e in s.traversed_edge_ids] == ["AB"]
    assert result.terminal_observation_id == "OBS_B"
    assert result.cost_uses_unknown_gates
    assert not result.reaches_requested_target


def test_poi_behind_unknown_gate_returns_observation_prefix_not_no_route():
    cfg = make_field(alternative=False)
    cfg = replace(cfg, observation_regions=cfg.observation_regions + (
        ObservationRegionConfig(
            "OBS_C",
            poly((1.85, 0.2), (2.15, 0.2), (2.15, 0.6), (1.85, 0.6)),
            "C",
            0.0,
            0.3,
        ),
    ))
    field = FieldModel.from_config(cfg)
    snap = snapshot(
        cfg,
        approach_state=ApproachResolution.UNKNOWN,
        coverage=CoverageResolution.UNKNOWN,
        target=False,
    )

    result = RoutePlanner(field, navigation()).plan(
        request(PlanGoalKind.CHECK_POI, target="OBS_C"), snap, start_pose()
    )

    assert result.status is PlanStatus.NEED_OBSERVATION
    assert [e for s in result.executable_prefix for e in s.traversed_edge_ids] == ["AB"]
    assert result.terminal_observation_id == "OBS_B"
    assert result.cost_uses_unknown_gates


def test_multicell_corridor_is_not_executable_until_interior_obligation_complete():
    cfg = make_field(alternative=False, long_edge=True)
    field = FieldModel.from_config(cfg)
    unknown = snapshot(cfg, coverage=CoverageResolution.UNKNOWN)
    transitions = {t.edge_id: t for t in build_transitions(field, unknown, navigation())}
    assert not transitions["AC_LONG"].executable
    assert transitions["AC_LONG"].provisional
    assert transitions["AC_LONG"].observation_id == "OBS_B"

    complete = snapshot(cfg, coverage=CoverageResolution.COMPLETE)
    transitions = {t.edge_id: t for t in build_transitions(field, complete, navigation())}
    assert transitions["AC_LONG"].executable


def test_blocked_short_edge_uses_legal_alternative():
    cfg = make_field(alternative=True)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg, block_bc=True)
    result = RoutePlanner(field, navigation()).plan(request(), snap, start_pose())
    assert result.status is PlanStatus.ROUTE_READY
    traversed = [e for s in result.executable_prefix for e in s.traversed_edge_ids]
    assert "BC" not in traversed
    assert "AD" in traversed


def test_blocked_edge_without_alternative_reports_no_route():
    cfg = make_field(alternative=False)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg, block_bc=True)
    result = RoutePlanner(field, navigation()).plan(request(), snap, start_pose())
    assert result.status is PlanStatus.NO_ROUTE


def test_same_snapshot_and_request_is_deterministic_and_does_not_mutate_snapshot():
    cfg = make_field(alternative=True)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg, approach_state=ApproachResolution.RESOLVED_SIGNED, allowed=MovementMask.RIGHT)
    before = snap
    planner = RoutePlanner(field, navigation())
    a = planner.plan(request(request_id=77), snap, start_pose())
    b = planner.plan(request(request_id=77), snap, start_pose())
    assert a == b
    assert snap == before
    assert a.route_id == b.route_id


def test_frontier_failure_count_breaks_equal_cost_tie():
    # Make an alternate observation at D and ensure fewer failed attempts wins.
    cfg = make_field(alternative=True)
    cfg = replace(cfg, observation_regions=cfg.observation_regions + (
        ObservationRegionConfig("OBS_D", poly((0.25, 1.05), (0.55, 1.05), (0.55, 1.35), (0.25, 1.35)), "D", pi/2, 0.3),
    ))
    field = FieldModel.from_config(cfg)
    base = snapshot(cfg, target=False, coverage=CoverageResolution.UNKNOWN)
    cov = (
        CoverageRecord("OBS_B", 1, CoverageResolution.INCONCLUSIVE, 3, 1.0, "failed"),
        CoverageRecord("OBS_D", 0, CoverageResolution.UNKNOWN, 0, None, ""),
    )
    snap = replace(base, coverage=cov)
    result = RoutePlanner(field, navigation()).plan(
        request(PlanGoalKind.EXPLORE_STOPS, target="", request_id=88), snap, start_pose()
    )
    assert result.status in (PlanStatus.ROUTE_READY, PlanStatus.ALREADY_AT_TERMINAL)
    assert result.terminal_observation_id == "OBS_D"
