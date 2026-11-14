from pathlib import Path
import math

import pytest

from frob_city_navigation.config_loader import load_project_config
from frob_city_navigation.models import (
    ApproachResolution,
    ConfigPaths,
    CoverageResolution,
    EvidenceEvent,
    EvidenceKind,
    ExecutionEvent,
    ExecutionEventKind,
    MovementMask,
    Pose2D,
    PoseSample,
    RangeConvention,
    SignClass,
    TargetResolution,
)
from frob_city_navigation.semantic_map import SemanticMap
from frob_city_navigation.field_model import FieldModel
from frob_city_navigation.map_planner_node import _FieldQueryAdapter
from frob_city_navigation.sign_fusion import CanonicalDetection


FIXTURE = Path(__file__).parent / "fixtures" / "valid"
APPROACH = "C1_0/FROM_WEST"
OBS = "OBS_EAST"
SITE = "SITE_0"
EDGE = "C0_0->C1_0"


def _config():
    return load_project_config(
        ConfigPaths(
            field=FIXTURE / "field.yaml",
            event=FIXTURE / "event.yaml",
            perception=FIXTURE / "perception.yaml",
            navigation=FIXTURE / "navigation.yaml",
        )
    )


def _map(run_id="run-1"):
    m = SemanticMap(_config().field)
    m.begin_run(run_id, "field-digest")
    return m


def ev(event_id, kind, *, sign=SignClass.UNKNOWN, approach="", site="", obs="", t=1.0, run="run-1"):
    return EvidenceEvent(
        run_id=run,
        event_id=event_id,
        timestamp_s=t,
        kind=kind,
        site_id=site,
        approach_id=approach,
        sign_class=sign,
        confidence=0.9,
        observation_id=obs,
        reason="test",
    )


def test_begin_run_builds_separate_initial_entities_and_sorted_snapshot():
    m = _map()
    s = m.snapshot()
    assert s.revision == 0
    assert s.ready
    assert [a.approach_id for a in s.approaches] == sorted(a.approach_id for a in s.approaches)
    assert [e.edge_id for e in s.edges] == [EDGE]
    assert [c.coverage_id for c in s.coverage] == [OBS]
    assert s.targets == ()
    assert s.approaches[0].state == ApproachResolution.UNKNOWN
    assert s.coverage[0].state == CoverageResolution.UNKNOWN


def test_map_instance_persists_across_run_reset_but_runtime_semantics_clear():
    m = _map()
    instance = m.map_instance_id
    m.apply(ev("p1", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site=SITE, approach=APPROACH))
    assert len(m.snapshot().targets) == 1
    m.begin_run("run-2", "field-digest")
    s = m.snapshot()
    assert s.map_instance_id == instance
    assert s.run_id == "run-2"
    assert s.revision == 0
    assert s.targets == ()
    assert s.approaches[0].state == ApproachResolution.UNKNOWN


def test_old_run_event_is_rejected_after_reset():
    m = _map()
    m.begin_run("run-2", "field-digest")
    update = m.apply(ev("old", EvidenceKind.QUALIFIED_CLEAR, approach=APPROACH, run="run-1"))
    assert not update.accepted
    assert m.snapshot().revision == 0


def test_directional_positive_resolves_signed_and_derives_mask_through_rule_engine():
    m = _map()
    u = m.apply(ev("s1", EvidenceKind.POSITIVE, sign=SignClass.STRAIGHT_ONLY, approach=APPROACH, obs=OBS))
    a = m.snapshot().approaches[0]
    assert u.accepted and u.changed and u.revision == 1
    assert a.state == ApproachResolution.RESOLVED_SIGNED
    assert a.confirmed_signs == frozenset({SignClass.STRAIGHT_ONLY})
    assert a.allowed_movements == MovementMask.STRAIGHT
    assert a.version == 1


def test_repeat_same_event_id_is_idempotent():
    m = _map()
    event = ev("s1", EvidenceKind.POSITIVE, sign=SignClass.STRAIGHT_ONLY, approach=APPROACH, obs=OBS)
    m.apply(event)
    u = m.apply(event)
    assert u.accepted and u.duplicate and not u.changed
    assert m.snapshot().revision == 1
    assert m.snapshot().approaches[0].version == 1


def test_new_supporting_event_renews_freshness_without_revision_storm():
    m = _map()
    m.apply(ev("s1", EvidenceKind.POSITIVE, sign=SignClass.STRAIGHT_ONLY, approach=APPROACH, obs=OBS, t=1.0))
    u = m.apply(ev("s2", EvidenceKind.POSITIVE, sign=SignClass.STRAIGHT_ONLY, approach=APPROACH, obs=OBS, t=2.0))
    a = m.snapshot().approaches[0]
    assert not u.changed
    assert m.snapshot().revision == 1
    assert a.version == 1
    assert a.last_qualified_observation_s == 2.0


def test_qualified_clear_resolves_unknown_approach_using_physical_topology():
    m = _map()
    m.apply(ev("c1", EvidenceKind.QUALIFIED_CLEAR, approach=APPROACH, obs=OBS))
    s = m.snapshot()
    a = s.approaches[0]
    assert a.state == ApproachResolution.RESOLVED_CLEAR
    assert a.allowed_movements == MovementMask.STRAIGHT  # fixture has only straight edge
    assert s.coverage[0].state == CoverageResolution.COMPLETE


def test_clear_observation_does_not_erase_previous_confirmed_sign():
    m = _map()
    m.apply(ev("s1", EvidenceKind.POSITIVE, sign=SignClass.STRAIGHT_ONLY, approach=APPROACH, obs=OBS))
    rev = m.snapshot().revision
    m.apply(ev("c1", EvidenceKind.QUALIFIED_CLEAR, approach=APPROACH, t=2.0))
    a = m.snapshot().approaches[0]
    assert a.state == ApproachResolution.RESOLVED_SIGNED
    assert a.confirmed_signs == frozenset({SignClass.STRAIGHT_ONLY})
    assert a.version == 1
    assert m.snapshot().revision == rev


def test_incompatible_directional_signs_create_one_approach_conflict():
    # On this fixture only STRAIGHT physically exists; adding LEFT_ONLY after
    # STRAIGHT_ONLY creates an empty intersection and therefore a conflict.
    m = _map()
    m.apply(ev("s1", EvidenceKind.POSITIVE, sign=SignClass.STRAIGHT_ONLY, approach=APPROACH, obs=OBS))
    m.apply(ev("s2", EvidenceKind.POSITIVE, sign=SignClass.LEFT_ONLY, approach=APPROACH, obs=OBS))
    a = m.snapshot().approaches[0]
    assert a.state == ApproachResolution.CONFLICT
    assert a.allowed_movements == MovementMask.NONE
    assert a.version == 2


def test_temporally_confirmed_moving_directional_detection_resolves_approach():
    m = _map()
    update = m.apply(
        ev(
            "moving-1",
            EvidenceKind.POSITIVE,
            sign=SignClass.STRAIGHT_ONLY,
            approach=APPROACH,
        )
    )
    a = m.snapshot().approaches[0]
    assert update.accepted and update.changed
    assert a.state == ApproachResolution.RESOLVED_SIGNED
    assert a.confirmed_signs == frozenset({SignClass.STRAIGHT_ONLY})
    assert a.allowed_movements == MovementMask.STRAIGHT
    assert a.last_qualified_observation_s is None

    m.apply(
        ev(
            "qualified-1",
            EvidenceKind.POSITIVE,
            sign=SignClass.STRAIGHT_ONLY,
            approach=APPROACH,
            obs=OBS,
            t=2.0,
        )
    )
    a = m.snapshot().approaches[0]
    assert a.state == ApproachResolution.RESOLVED_SIGNED
    assert a.allowed_movements == MovementMask.STRAIGHT


def test_directional_signs_keep_exact_physical_site_and_face_identity():
    m = _map()
    m.apply(ev("near", EvidenceKind.POSITIVE, sign=SignClass.LEFT_ONLY,
               approach=APPROACH, site=f"{SITE}/FACE_W"))

    signs = m.snapshot().known_signs
    assert [(s.site_id, s.face_id, s.sign_class) for s in signs] == [
        (SITE, "FACE_W", SignClass.LEFT_ONLY),
    ]


@pytest.mark.parametrize("sign_class", [item for item in SignClass if item is not SignClass.UNKNOWN])
def test_every_detectable_sign_class_is_kept_without_route_approach(sign_class):
    m = _map()
    update = m.apply(
        ev(
            f"sign-{int(sign_class)}",
            EvidenceKind.POSITIVE,
            sign=sign_class,
            site=f"{SITE}/FACE_W",
            approach="",
        )
    )

    assert update.accepted and update.changed
    assert [(item.site_id, item.face_id, item.sign_class) for item in m.snapshot().known_signs] == [
        (SITE, "FACE_W", sign_class)
    ]


def test_bus_target_has_stable_physical_identity_across_repeated_views():
    m = _map()
    m.apply(ev("b1", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site=SITE, approach=APPROACH, t=1.0))
    target1 = m.snapshot().targets[0]
    rev = m.snapshot().revision
    m.apply(ev("b2", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site=SITE, approach=APPROACH, t=4.0))
    target2 = m.snapshot().targets[0]
    assert target1.target_id == "BUS_STOP:SITE_0:FACE_W"
    assert target2.target_id == target1.target_id
    assert target2.version == 1
    assert target2.last_seen_s == 4.0
    assert m.snapshot().revision == rev
    assert target2.geometry_valid
    assert target2.service_pose is not None
    assert target2.terminal_limit is not None


def test_target_accepts_fusion_physical_face_key_without_losing_site_identity():
    m = _map()
    update = m.apply(
        ev(
            "b-face-1",
            EvidenceKind.POSITIVE,
            sign=SignClass.BUS_STOP,
            site=f"{SITE}/FACE_W",
            approach=APPROACH,
        )
    )
    assert update.accepted and update.changed
    target = m.snapshot().targets[0]
    assert target.site_id == SITE
    assert target.face_id == "FACE_W"
    assert target.target_id == "BUS_STOP:SITE_0:FACE_W"


def test_lab_geometry_distinguishes_b2_end_and_rejects_far_detections():
    config_dir = Path(__file__).resolve().parents[1] / "config"
    config = load_project_config(
        ConfigPaths(
            field=config_dir / "lab_sign_test_field.yaml",
            event=config_dir / "lab_sign_test_event.yaml",
            perception=config_dir / "lab_perception.yaml",
            navigation=config_dir / "wp11_test_navigation.yaml",
        )
    )
    field = FieldModel.from_config(config.field)
    adapter = _FieldQueryAdapter(
        field,
        focal_length_x_px=1500.0,
        principal_point_x_px=640.0,
        max_detection_range_m=1.60,
        max_bearing_error_rad=0.40,
        max_depth_error_m=0.40,
        face_view_tolerance_rad=1.40,
        min_right_lateral_m=0.05,
        score_margin=0.35,
    )
    pose = PoseSample(
        timestamp_s=1.0,
        pose=field.field_to_map_pose(Pose2D(0.4, 0.4, math.pi / 2.0)),
    )

    visible = CanonicalDetection(
        sign_class=SignClass.LEFT_ONLY,
        confidence=0.90,
        bbox_xyxy_px=(731.0, 96.0, 882.0, 252.0),
        range_m=0.97,
        range_convention=RangeConvention.OPTICAL_Z,
        range_stddev_m=0.10,
        track_id="current-live-shape",
    )
    candidates = adapter.candidates(pose=pose, detection=visible)
    assert [(item.site_id, item.face_id) for item in candidates] == [
        ("SIGN_B2_NW", "FACE_W")
    ]

    too_far = CanonicalDetection(
        sign_class=visible.sign_class,
        confidence=visible.confidence,
        bbox_xyxy_px=visible.bbox_xyxy_px,
        range_m=2.40,
        range_convention=visible.range_convention,
        range_stddev_m=visible.range_stddev_m,
        track_id="three-cells-away",
    )
    far_candidates = adapter.candidates(pose=pose, detection=too_far)
    assert far_candidates
    assert not any(item.current_junction for item in far_candidates)


def test_manual_field_has_all_sites_and_selects_road_facing_side():
    config_dir = Path(__file__).resolve().parents[1] / "config"
    config = load_project_config(
        ConfigPaths(
            field=config_dir / "lab_manual_field.yaml",
            event=config_dir / "lab_manual_event.yaml",
            perception=config_dir / "lab_perception.yaml",
            navigation=config_dir / "wp11_test_navigation.yaml",
        )
    )
    assert len(config.field.sign_sites) == 16
    assert {len(site.faces) for site in config.field.sign_sites} == {4}
    assert {
        face.face_id
        for site in config.field.sign_sites
        for face in site.faces
    } == {"FACE_E", "FACE_N", "FACE_W", "FACE_S"}

    field = FieldModel.from_config(config.field)
    adapter = _FieldQueryAdapter(
        field,
        focal_length_x_px=1500.0,
        principal_point_x_px=640.0,
        max_detection_range_m=1.60,
        max_bearing_error_rad=0.40,
        max_depth_error_m=0.40,
        face_view_tolerance_rad=0.60,
        min_right_lateral_m=0.05,
        score_margin=0.35,
    )
    pose = PoseSample(
        timestamp_s=1.0,
        pose=field.field_to_map_pose(Pose2D(0.4, 0.4, math.pi / 2.0)),
    )
    detection = CanonicalDetection(
        sign_class=SignClass.LEFT_ONLY,
        confidence=0.90,
        bbox_xyxy_px=(731.0, 96.0, 882.0, 252.0),
        range_m=0.97,
        range_convention=RangeConvention.OPTICAL_Z,
        range_stddev_m=0.10,
        track_id="west-road-northbound",
    )

    candidates = adapter.candidates(pose=pose, detection=detection)
    assert [(item.site_id, item.face_id) for item in candidates] == [
        ("SIGN_B2_NW", "FACE_S")
    ]

    def associated_face(site_id, robot_field_pose):
        robot_pose = field.field_to_map_pose(robot_field_pose)
        sign_point = field.field_to_map_point(field.sign_site(site_id).position)
        dx = sign_point.x_m - robot_pose.x_m
        dy = sign_point.y_m - robot_pose.y_m
        forward = math.cos(robot_pose.yaw_rad) * dx + math.sin(robot_pose.yaw_rad) * dy
        left = -math.sin(robot_pose.yaw_rad) * dx + math.cos(robot_pose.yaw_rad) * dy
        bearing = math.atan2(left, forward)
        image_u = 640.0 - math.tan(bearing) * 1500.0
        exact = CanonicalDetection(
            sign_class=SignClass.NO_RIGHT,
            confidence=0.92,
            bbox_xyxy_px=(image_u - 50.0, 100.0, image_u + 50.0, 220.0),
            range_m=forward,
            range_convention=RangeConvention.OPTICAL_Z,
            range_stddev_m=0.05,
            track_id=f"scan-{site_id}",
        )
        matches = adapter.candidates(
            pose=PoseSample(timestamp_s=2.0, pose=robot_pose),
            detection=exact,
        )
        assert len(matches) == 1
        return matches[0].site_id, matches[0].face_id

    # Every physical hole is distinguishable from a parallel road pass.  West
    # corners are approached northbound; east corners are approached southbound.
    for site_id in field.sign_site_ids:
        site = field.sign_site(site_id)
        if site_id.endswith(("_SW", "_NW")):
            robot = Pose2D(site.position.x_m - 0.15, site.position.y_m - 0.80, math.pi / 2.0)
            expected_face = "FACE_S"
        else:
            robot = Pose2D(site.position.x_m + 0.15, site.position.y_m + 0.80, -math.pi / 2.0)
            expected_face = "FACE_N"
        assert associated_face(site_id, robot) == (site_id, expected_face)

    # Complete the four camera travel directions with two perpendicular passes.
    sw = field.sign_site("SIGN_B2_SW").position
    assert associated_face(
        "SIGN_B2_SW", Pose2D(sw.x_m + 0.80, sw.y_m - 0.15, math.pi)
    ) == ("SIGN_B2_SW", "FACE_E")
    ne = field.sign_site("SIGN_B2_NE").position
    assert associated_face(
        "SIGN_B2_NE", Pose2D(ne.x_m - 0.80, ne.y_m + 0.15, 0.0)
    ) == ("SIGN_B2_NE", "FACE_W")


def test_runtime_contract_keeps_moving_scan_and_positive_action_result():
    node_path = (
        Path(__file__).resolve().parents[1]
        / "frob_city_navigation"
        / "map_planner_node.py"
    )
    source = node_path.read_text(encoding="utf-8")
    detection_handler = source.split("def _on_detections", 1)[1].split(
        "# ------------------------------------------------------------------\n    # Snapshot / diagnostics",
        1,
    )[0]
    assert "if active is None:\n            return" not in detection_handler
    assert 'else "moving_scan"' in detection_handler
    assert 'elif kind_name == "POSITIVE":' in source
    assert 'elif kind_name == "PRESENT":' not in source


def test_bus_parking_contradiction_at_same_face_becomes_one_conflicted_target():
    m = _map()
    m.apply(ev("b1", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site=SITE, approach=APPROACH))
    m.apply(ev("p1", EvidenceKind.POSITIVE, sign=SignClass.PARKING, site=SITE, approach=APPROACH, t=2.0))
    targets = m.snapshot().targets
    assert len(targets) == 1
    assert targets[0].target_id == "BUS_STOP:SITE_0:FACE_W"
    assert targets[0].state == TargetResolution.CONFLICT
    assert targets[0].version == 2


def test_nondirectional_target_does_not_authorize_unknown_approach():
    m = _map()
    m.apply(ev("b1", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site=SITE, approach=APPROACH))
    a = m.snapshot().approaches[0]
    assert a.state == ApproachResolution.UNKNOWN
    assert a.allowed_movements == MovementMask.NONE


def test_inconclusive_updates_coverage_failure_accounting():
    m = _map()
    m.apply(ev("i1", EvidenceKind.INCONCLUSIVE, obs=OBS))
    m.apply(ev("i2", EvidenceKind.INCONCLUSIVE, obs=OBS, t=2.0))
    c = m.snapshot().coverage[0]
    assert c.state == CoverageResolution.INCONCLUSIVE
    assert c.failed_attempts == 2
    assert c.version == 2
    assert m.snapshot().revision == 2


def test_precautionary_hold_is_separate_from_approach_semantic_version():
    m = _map()
    m.apply(ev("h1", EvidenceKind.PRECAUTIONARY_HOLD, approach=APPROACH))
    s = m.snapshot()
    assert s.precautionary_hold_ids == (f"approach:{APPROACH}",)
    assert s.approaches[0].precautionary_hold
    assert s.approaches[0].version == 0
    assert s.approaches[0].state == ApproachResolution.UNKNOWN


def test_resolving_held_approach_removes_hold_and_changes_semantics_once():
    m = _map()
    m.apply(ev("h1", EvidenceKind.PRECAUTIONARY_HOLD, approach=APPROACH))
    rev1 = m.snapshot().revision
    m.apply(ev("c1", EvidenceKind.QUALIFIED_CLEAR, approach=APPROACH))
    s = m.snapshot()
    assert rev1 == 1
    assert s.revision == 2
    assert s.precautionary_hold_ids == ()
    assert not s.approaches[0].precautionary_hold
    assert s.approaches[0].state == ApproachResolution.RESOLVED_CLEAR


def test_explicit_conflict_updates_approach_and_existing_target_without_duplication():
    m = _map()
    m.apply(ev("b1", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site=SITE, approach=APPROACH))
    m.apply(ev("x1", EvidenceKind.CONFLICT, site=SITE, approach=APPROACH, t=3.0))
    s = m.snapshot()
    assert s.approaches[0].state == ApproachResolution.CONFLICT
    assert len(s.targets) == 1
    assert s.targets[0].state == TargetResolution.CONFLICT


def test_version_of_uses_namespaced_dependency_ids():
    m = _map()
    assert m.version_of(f"approach:{APPROACH}") == 0
    assert m.version_of(f"edge:{EDGE}") == 0
    assert m.version_of(f"coverage:{OBS}") == 0
    assert m.version_of("garbage") is None
    m.apply(ev("b1", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site=SITE, approach=APPROACH))
    assert m.version_of("target:BUS_STOP:SITE_0:FACE_W") == 1


def test_dependency_match_detects_stale_unknown_duplicate_and_length_mismatch():
    m = _map()
    ids = (f"approach:{APPROACH}", f"edge:{EDGE}")
    assert m.dependencies_match(ids, (0, 0)).matches
    m.apply(ev("s1", EvidenceKind.POSITIVE, sign=SignClass.STRAIGHT_ONLY, approach=APPROACH))
    stale = m.dependencies_match(ids, (0, 0))
    assert not stale.matches and stale.mismatched_ids == (f"approach:{APPROACH}",)
    unknown = m.dependencies_match(("target:missing",), (1,))
    assert unknown.unknown_ids == ("target:missing",)
    assert not m.dependencies_match(("edge:x",), ()).matches
    assert not m.dependencies_match((f"edge:{EDGE}", f"edge:{EDGE}"), (0, 0)).matches


def test_snapshots_are_frozen_and_callers_cannot_mutate_internal_state():
    m = _map()
    s = m.snapshot()
    with pytest.raises(Exception):
        s.revision = 99
    assert m.snapshot().revision == 0


def test_execution_reports_are_idempotent_without_guessing_segment_edges():
    m = _map()
    event = ExecutionEvent(
        run_id="run-1",
        navigator_instance_id="nav-A",
        report_sequence=7,
        segment_id="opaque-segment-id",
        kind=ExecutionEventKind.SEGMENT_COMPLETED,
        terminal_pose=Pose2D(1.0, 1.0, 0.0),
        timestamp_s=2.0,
    )
    first = m.apply(event)
    second = m.apply(event)
    assert first.accepted and not first.changed
    assert second.accepted and second.duplicate
    assert m.snapshot().edges[0].completed_traversals == 0
    assert m.snapshot().revision == 0


def test_invalid_target_site_is_rejected_atomically():
    m = _map()
    before = m.snapshot()
    update = m.apply(ev("bad", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site="NO_SITE", approach=APPROACH))
    after = m.snapshot()
    assert not update.accepted
    assert before == after


def test_map_does_not_contain_serviced_mission_state():
    # Architectural ownership check: a map target is a world fact, not a mission receipt.
    m = _map()
    m.apply(ev("b1", EvidenceKind.POSITIVE, sign=SignClass.BUS_STOP, site=SITE, approach=APPROACH))
    target = m.snapshot().targets[0]
    assert not hasattr(target, "serviced")
    assert not hasattr(m.snapshot(), "serviced_stop_ids")


def test_wp11_execution_wrapper_resolves_cached_edges_without_changing_dependency_version():
    m = _map()
    before = m.snapshot()
    event = ExecutionEvent(
        run_id="run-1",
        navigator_instance_id="nav-A",
        report_sequence=8,
        segment_id="seg-1",
        kind=ExecutionEventKind.SEGMENT_COMPLETED,
        terminal_pose=Pose2D(1.0, 1.0, 0.0),
        timestamp_s=2.0,
    )
    update = m.apply_execution_with_edges(event, (EDGE,))
    after = m.snapshot()
    assert update.accepted and update.changed
    assert after.edges[0].completed_traversals == before.edges[0].completed_traversals + 1
    assert after.edges[0].version == before.edges[0].version
    assert after.revision == before.revision + 1


def test_wp11_execution_wrapper_deduplicates_same_navigator_sequence():
    m = _map()
    event = ExecutionEvent(
        run_id="run-1",
        navigator_instance_id="nav-A",
        report_sequence=9,
        segment_id="seg-1",
        kind=ExecutionEventKind.SEGMENT_COMPLETED,
        terminal_pose=Pose2D(1.0, 1.0, 0.0),
        timestamp_s=2.0,
    )
    first = m.apply_execution_with_edges(event, (EDGE,))
    second = m.apply_execution_with_edges(event, (EDGE,))
    assert first.accepted and first.changed
    assert second.accepted and second.duplicate and not second.changed
    assert m.snapshot().edges[0].completed_traversals == 1
