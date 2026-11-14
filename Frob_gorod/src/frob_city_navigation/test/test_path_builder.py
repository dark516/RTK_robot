from dataclasses import replace

from frob_city_navigation.field_model import FieldModel
from frob_city_navigation.graph import build_transitions
from frob_city_navigation.models import (
    GeometryMargins,
    GraphState,
    GraphTransition,
    MovementMask,
    Pose2D,
    PoseSample,
    RouteDependency,
    RouteSegment,
    SegmentKind,
    SegmentValidationStatus,
    TerminalKind,
)
from frob_city_navigation.path_builder import PathBuilder, validate_segment

from test_planner import make_field, navigation, snapshot


def test_platform_cut_is_rejected_by_path_builder():
    cfg = make_field(alternative=False, platform_cut=True)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg)
    transition = next(t for t in build_transitions(field, snap, navigation()) if t.edge_id == "BC")
    built = PathBuilder(field, navigation()).build_segment(
        transition, snap, PoseSample(1.0, Pose2D(1.2, 0.4, 0.0))
    )
    assert built.segment is None
    assert "platform" in built.reason


def test_wrong_junction_exit_is_rejected_on_revalidation():
    cfg = make_field(alternative=False)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg)
    transition = next(t for t in build_transitions(field, snap, navigation()) if t.edge_id == "BC")
    built = PathBuilder(field, navigation()).build_segment(
        transition, snap, PoseSample(1.0, Pose2D(1.2, 0.4, 0.0))
    )
    assert built.segment is not None
    bad = replace(built.segment, selected_movement=MovementMask.LEFT)
    checked = validate_segment(bad, snap, PoseSample(1.0, Pose2D(1.2, 0.4, 0.0)), field)
    assert checked.status is SegmentValidationStatus.INVALID
    assert "movement" in checked.reason


def test_terminal_plane_overshoot_is_rejected():
    cfg = make_field(alternative=False)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg)
    # Build a synthetic service transition whose path endpoint puts the robot
    # front beyond x=2.30, so the terminal plane must reject it.
    primitive = field.movement_primitive("CT")
    transition = GraphTransition(
        transition_id="bad-service",
        from_state=GraphState("C", ""),
        to_state=GraphState("target:X", "CT"),
        edge_id="CT",
        movement=MovementMask.STRAIGHT,
        path=primitive.path,
        permitted_regions=primitive.permitted_regions,
        approach_id="",
        executable=True,
        provisional=True,
        dependencies=(RouteDependency("edge:CT", 0),),
        target_id="X",
        segment_kind=SegmentKind.SERVICE_APPROACH,
        terminal_kind=TerminalKind.BUS_SERVICE,
        terminal_limit=Pose2D(2.30, 0.4, 0.0),
        estimated_cost_s=1.0,
    )
    built = PathBuilder(field, navigation()).build_segment(
        transition, snap, PoseSample(1.0, Pose2D(2.0, 0.4, 0.0))
    )
    assert built.segment is None
    assert "terminal" in built.reason


def test_stale_dependency_invalidates_segment():
    cfg = make_field(alternative=False)
    field = FieldModel.from_config(cfg)
    snap = snapshot(cfg)
    transition = next(t for t in build_transitions(field, snap, navigation()) if t.edge_id == "AB")
    built = PathBuilder(field, navigation()).build_segment(
        transition, snap, PoseSample(1.0, Pose2D(0.4, 0.4, 0.0))
    )
    assert built.segment is not None
    changed_edges = tuple(replace(e, version=e.version + 1) if e.edge_id == "AB" else e for e in snap.edges)
    changed = replace(snap, revision=snap.revision + 1, edges=changed_edges)
    checked = validate_segment(built.segment, changed, PoseSample(2.0, Pose2D(0.4, 0.4, 0.0)), field)
    assert checked.status is SegmentValidationStatus.INVALID
    assert "dependency" in checked.reason
