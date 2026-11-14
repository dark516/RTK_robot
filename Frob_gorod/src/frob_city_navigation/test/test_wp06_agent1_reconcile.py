from __future__ import annotations

from dataclasses import fields
import unittest

from frob_city_navigation import models
from frob_city_navigation.sign_fusion import (
    CanonicalDetection,
    CanonicalDetectionFrame,
    DetectorBoundaryConfig,
    LegacyDetectorAdapter,
    LegacyRawDetection,
    LegacyRawDetectionFrame,
    ObservationContext,
    Pose2D,
    PoseLookupResult,
    PoseSample,
    SignFusion,
    SignFusionConfig,
    SiteCandidate,
)
from frob_city_navigation.observation import (
    ObservationConfig,
    ObservationFrame,
    ObservationSession,
    PoseSample as ObservationPoseSample,
)


def first_two_sign_classes():
    values = list(models.SignClass)
    if len(values) < 2:
        raise unittest.SkipTest('SignClass needs two classes for conflict test')
    return values[0], values[1]


def convention_named(token):
    token = token.upper()
    for item in models.RangeConvention:
        if token in item.name.upper():
            return item
    raise unittest.SkipTest(f'No RangeConvention containing {token}')


class FakePose:
    def __init__(self):
        self.samples = {}
        self.requests = []

    def add(self, sample):
        self.samples[sample.timestamp_s] = sample

    def pose_at(self, timestamp_s):
        self.requests.append(timestamp_s)
        item = self.samples.get(timestamp_s)
        if item is None:
            return PoseLookupResult(False, 'missing_capture_time_pose')
        return PoseLookupResult(True, 'ok', item)


class FakeField:
    def __init__(self):
        self.items = ()

    def candidates(self, *, pose, detection):
        del pose, detection
        return tuple(self.items)


def frame(seq, timestamp, sign_class, confidence=0.90):
    return CanonicalDetectionFrame(
        sequence=seq,
        capture_timestamp_s=timestamp,
        frame_id='camera',
        detections=(
            CanonicalDetection(
                sign_class=sign_class,
                confidence=confidence,
                bbox_xyxy_px=(0.0, 0.0, 10.0, 10.0),
                range_m=1.0,
                range_convention=convention_named('RAY_RANGE'),
                range_stddev_m=0.05,
                track_id=f'track-{seq}',
            ),
        ),
    )



def semantic_kind(event):
    if (
        event.kind is models.EvidenceKind.POSITIVE
        and event.reason == 'required_distinct_positive_frames_met'
    ):
        return 'confirmed'
    if event.kind is models.EvidenceKind.CONFLICT:
        return 'conflict'
    if event.kind is models.EvidenceKind.INCONCLUSIVE:
        return 'inconclusive'
    if event.kind is models.EvidenceKind.PRECAUTIONARY_HOLD:
        return 'hold'
    if event.kind is models.EvidenceKind.QUALIFIED_CLEAR:
        return 'clear'
    return event.kind.name.lower()


class TestAgent1Contract(unittest.TestCase):
    def test_shared_semantic_types_are_identity_imports(self):
        import frob_city_navigation.sign_fusion as sf
        self.assertIs(sf.EvidenceEvent, models.EvidenceEvent)
        self.assertIs(sf.EvidenceKind, models.EvidenceKind)
        self.assertIs(sf.SignClass, models.SignClass)

    def test_required_evidence_fields_exist(self):
        names = {item.name for item in fields(models.EvidenceEvent)}
        required = {
            'run_id',
            'event_id',
            'timestamp_s',
            'kind',
            'site_id',
            'approach_id',
            'sign_class',
            'confidence',
            'observation_id',
            'reason',
        }
        self.assertTrue(required.issubset(names))

    def test_candidate_has_face_approach_and_no_class_truth(self):
        names = {item.name for item in fields(SiteCandidate)}
        self.assertIn('face_id', names)
        self.assertIn('controlled_approach_id', names)
        self.assertNotIn('expected_label', names)

    def test_detection_preserves_convention_and_uncertainty(self):
        names = {item.name for item in fields(CanonicalDetection)}
        self.assertIn('range_convention', names)
        self.assertIn('range_stddev_m', names)


class TestLegacyBoundary(unittest.TestCase):
    def test_cm_to_m_once_and_preserves_range_semantics(self):
        sign_a, _ = first_two_sign_classes()
        convention = convention_named('RAY_RANGE')
        adapter = LegacyDetectorAdapter(
            DetectorBoundaryConfig(
                label_to_sign_class={'A': sign_a},
            )
        )
        result = adapter.convert(
            LegacyRawDetectionFrame(
                sequence=1,
                capture_timestamp_s=1.0,
                frame_id='camera',
                healthy=True,
                detections=(
                    LegacyRawDetection(
                        label='A',
                        confidence=0.9,
                        bbox_xyxy_px=(0, 0, 10, 10),
                        range_value=120.0,
                        range_unit='cm',
                        range_convention=convention,
                        range_stddev_value=10.0,
                    ),
                ),
            )
        )
        self.assertTrue(result.accepted)
        det = result.frame.detections[0]
        self.assertAlmostEqual(det.range_m, 1.2)
        self.assertAlmostEqual(det.range_stddev_m, 0.1)
        self.assertIs(det.range_convention, convention)


class TestFusionTimingAndIdentity(unittest.TestCase):
    def setUp(self):
        self.sign_a, self.sign_b = first_two_sign_classes()
        self.pose = FakePose()
        self.field = FakeField()
        self.field.items = (
            SiteCandidate(
                site_id='SLOT_0',
                face_id='NORTH',
                controlled_approach_id='APP_0',
                bearing_rad=0.0,
                range_m=1.0,
            ),
        )
        self.context = ObservationContext(
            run_id='RUN_1',
            observation_id='OBS_1',
            approach_id='APP_0',
            map_instance_id='MAP_1',
            route_revision='R1',
        )

    def fusion(self):
        return SignFusion(
            self.pose,
            self.field,
            SignFusionConfig(
                required_positive_frames=3,
                positive_window_s=0.8,
                positive_min_span_s=0.15,
            ),
        )

    def add_pose(self, t):
        self.pose.add(PoseSample(t, Pose2D(0.0, 0.0, 0.0)))

    def add_moving_pose(self, t, *, yaw_rad=0.0, angular_speed_radps=0.0):
        self.pose.add(
            PoseSample(
                t,
                Pose2D(0.0, 0.0, yaw_rad),
                linear_speed_mps=0.10,
                angular_speed_radps=angular_speed_radps,
            )
        )

    def test_far_apart_frames_do_not_confirm(self):
        fusion = self.fusion()
        last = None
        for seq, t in ((1, 0.0), (2, 100.0), (3, 200.0)):
            self.add_pose(t)
            last = fusion.ingest(frame(seq, t, self.sign_a), self.context)[0]
        self.assertEqual(semantic_kind(last), 'inconclusive')

    def test_too_close_frames_do_not_confirm(self):
        fusion = self.fusion()
        last = None
        for seq, t in ((1, 1.00), (2, 1.02), (3, 1.04)):
            self.add_pose(t)
            last = fusion.ingest(frame(seq, t, self.sign_a), self.context)[0]
        self.assertEqual(semantic_kind(last), 'inconclusive')

    def test_three_frames_inside_window_with_min_span_confirm(self):
        fusion = self.fusion()
        events = []
        for seq, t in ((1, 2.00), (2, 2.10), (3, 2.20)):
            self.add_pose(t)
            events.append(
                fusion.ingest(frame(seq, t, self.sign_a), self.context)[0]
            )
        self.assertEqual(semantic_kind(events[-1]), 'confirmed')
        self.assertEqual(events[-1].run_id, 'RUN_1')
        self.assertEqual(events[-1].observation_id, 'OBS_1')
        self.assertEqual(events[-1].approach_id, 'APP_0')
        self.assertEqual(events[-1].site_id, 'SLOT_0/NORTH')
        self.assertIs(events[-1].sign_class, self.sign_a)

    def test_default_window_confirms_two_distinct_moving_frames(self):
        fusion = SignFusion(self.pose, self.field)
        events = []
        for seq, t in ((1, 10.0), (2, 10.72)):
            self.add_pose(t)
            events.append(
                fusion.ingest(frame(seq, t, self.sign_a), self.context)[0]
            )
        self.assertEqual(semantic_kind(events[-1]), 'confirmed')

    def test_mapping_does_not_confirm_while_robot_is_turning(self):
        fusion = SignFusion(self.pose, self.field)
        passive_context = ObservationContext(
            run_id='RUN_1',
            observation_id='',
            approach_id='',
            map_instance_id='MAP_1',
            route_revision='R1',
        )
        events = []
        for seq, t, yaw in (
            (1, 20.0, 0.00),
            (2, 21.0, 0.30),
            (3, 22.0, 0.60),
        ):
            self.add_moving_pose(
                t,
                yaw_rad=yaw,
                angular_speed_radps=0.30,
            )
            events.append(
                fusion.ingest(frame(seq, t, self.sign_a), passive_context)[0]
            )

        self.assertTrue(
            all(event.reason == 'turn_in_progress' for event in events)
        )
        self.assertNotIn('confirmed', [semantic_kind(event) for event in events])
        self.assertEqual(
            fusion.positive_count(
                site_id='SLOT_0',
                face_id='NORTH',
                sign_class=self.sign_a,
            ),
            0,
        )

    def test_mapping_rejects_negative_angular_velocity_too(self):
        fusion = SignFusion(self.pose, self.field)
        self.add_moving_pose(23.0, angular_speed_radps=-0.30)
        event = fusion.ingest(frame(1, 23.0, self.sign_a), self.context)[0]
        self.assertEqual(event.reason, 'turn_in_progress')

    def test_conflict_comes_from_competing_observed_class_same_face(self):
        fusion = self.fusion()
        self.add_pose(3.0)
        self.add_pose(3.1)
        first = fusion.ingest(
            frame(1, 3.0, self.sign_a, 0.90),
            self.context,
        )[0]
        second = fusion.ingest(
            frame(2, 3.1, self.sign_b, 0.88),
            self.context,
        )[0]
        self.assertEqual(semantic_kind(first), 'inconclusive')
        self.assertEqual(semantic_kind(second), 'conflict')

    def test_new_run_reset_clears_confirmed_state(self):
        fusion = self.fusion()
        for seq, t in ((1, 4.0), (2, 4.1), (3, 4.2)):
            self.add_pose(t)
            fusion.ingest(frame(seq, t, self.sign_a), self.context)
        self.assertEqual(
            fusion.positive_count(
                site_id='SLOT_0',
                face_id='NORTH',
                sign_class=self.sign_a,
            ),
            3,
        )
        fusion.begin_run('RUN_2')
        self.assertEqual(
            fusion.positive_count(
                site_id='SLOT_0',
                face_id='NORTH',
                sign_class=self.sign_a,
            ),
            0,
        )

    def test_context_run_mismatch_fails_closed(self):
        fusion = self.fusion()
        fusion.begin_run('RUN_A')
        self.add_pose(5.0)
        event = fusion.ingest(
            frame(1, 5.0, self.sign_a),
            self.context,
        )[0]
        self.assertIn(
            semantic_kind(event),
            ('hold', 'inconclusive'),
        )


class TestObservationSpan(unittest.TestCase):
    def test_eight_frames_over_point_35_seconds_are_not_clear(self):
        session = ObservationSession(
            ObservationConfig(
                required_distinct_frames=8,
                required_sectors=frozenset(
                    {'left', 'center', 'right'}
                ),
                min_hits_per_sector=2,
                min_temporal_span_s=1.0,
                max_capture_age_s=0.75,
            )
        )
        session.begin('RUN', 'OBS')
        sectors = (
            {'left'},
            {'center'},
            {'right'},
            {'left'},
            {'center'},
            {'right'},
            {'left', 'center'},
            {'center', 'right'},
        )
        for i, covered in enumerate(sectors):
            t = 30.0 + i * 0.05
            session.ingest(
                ObservationFrame(
                    run_id='RUN',
                    observation_id='OBS',
                    sequence=i + 1,
                    capture_timestamp_s=t,
                    healthy=True,
                    covered_sectors=frozenset(covered),
                ),
                ObservationPoseSample(t, models.Pose2D(0.0, 0.0, 0.0)),
                t + 0.01,
            )
        out = session.finish(30.4)
        self.assertIs(
            out.kind,
            models.EvidenceKind.INCONCLUSIVE,
        )
        self.assertEqual(out.reason, 'insufficient_temporal_span')

    def test_qualified_clear_requires_at_least_one_second(self):
        session = ObservationSession(
            ObservationConfig(
                required_distinct_frames=8,
                required_sectors=frozenset(
                    {'left', 'center', 'right'}
                ),
                min_hits_per_sector=2,
                min_temporal_span_s=1.0,
                max_capture_age_s=0.75,
            )
        )
        session.begin('RUN', 'OBS')
        times = (40.0, 40.15, 40.30, 40.45, 40.60, 40.75, 40.90, 41.05)
        for i, t in enumerate(times):
            session.ingest(
                ObservationFrame(
                    run_id='RUN',
                    observation_id='OBS',
                    sequence=i + 1,
                    capture_timestamp_s=t,
                    healthy=True,
                    covered_sectors=frozenset(
                        {'left', 'center', 'right'}
                    ),
                ),
                ObservationPoseSample(t, models.Pose2D(0.0, 0.0, 0.0)),
                t + 0.01,
            )
        out = session.finish(41.10)
        self.assertIs(
            out.kind,
            models.EvidenceKind.QUALIFIED_CLEAR,
        )
        self.assertGreaterEqual(out.temporal_span_s, 1.0)


class TestCanonicalObservationEvidence(unittest.TestCase):
    def test_observation_consumes_frozen_evidence_event(self):
        names = {item.name for item in fields(ObservationFrame)}
        self.assertIn('evidence', names)

        session = ObservationSession(
            ObservationConfig(
                required_distinct_frames=1,
                required_sectors=frozenset({'center'}),
                min_hits_per_sector=1,
                min_temporal_span_s=0.0,
            )
        )
        session.begin('RUN', 'OBS')
        sign_a, _ = first_two_sign_classes()
        event = models.EvidenceEvent(
            run_id='RUN',
            event_id='E1',
            timestamp_s=1.0,
            kind=models.EvidenceKind.POSITIVE,
            site_id='SLOT_0/NORTH',
            approach_id='APP_0',
            sign_class=sign_a,
            confidence=0.95,
            observation_id='OBS',
            reason='required_distinct_positive_frames_met',
        )
        progress = session.ingest(
            ObservationFrame(
                run_id='RUN',
                observation_id='OBS',
                sequence=1,
                capture_timestamp_s=1.0,
                healthy=True,
                covered_sectors=frozenset({'center'}),
                evidence=(event,),
            ),
            ObservationPoseSample(1.0, models.Pose2D(0.0, 0.0, 0.0)),
            now=1.01,
        )
        self.assertTrue(progress.accepted)
        outcome = session.finish(1.02)
        self.assertIs(outcome.kind, models.EvidenceKind.POSITIVE)
        self.assertEqual(outcome.site_ids, ('SLOT_0/NORTH',))

    def test_unconfirmed_fusion_is_canonical_inconclusive(self):
        sign_a, _ = first_two_sign_classes()
        pose_port = FakePose()
        field = FakeField()
        field.items = (
            SiteCandidate(
                site_id='SLOT_0',
                face_id='NORTH',
                controlled_approach_id='APP_0',
                bearing_rad=0.0,
                range_m=1.0,
            ),
        )
        pose_port.add(PoseSample(1.0, Pose2D(0.0, 0.0, 0.0)))
        fusion = SignFusion(pose_port, field)
        context = ObservationContext(
            run_id='RUN',
            observation_id='OBS',
            approach_id='APP_0',
            map_instance_id='MAP',
            route_revision='R1',
        )
        event = fusion.ingest(
            frame(1, 1.0, sign_a),
            context,
        )[0]
        self.assertIs(event.kind, models.EvidenceKind.INCONCLUSIVE)
        self.assertEqual(event.reason, 'positive_unconfirmed')

    def test_confirmed_fusion_is_canonical_positive(self):
        sign_a, _ = first_two_sign_classes()
        pose_port = FakePose()
        field = FakeField()
        field.items = (
            SiteCandidate(
                site_id='SLOT_0',
                face_id='NORTH',
                controlled_approach_id='APP_0',
                bearing_rad=0.0,
                range_m=1.0,
            ),
        )
        fusion = SignFusion(pose_port, field)
        context = ObservationContext(
            run_id='RUN',
            observation_id='OBS',
            approach_id='APP_0',
            map_instance_id='MAP',
            route_revision='R1',
        )
        event = None
        for seq, t in ((1, 2.0), (2, 2.1), (3, 2.2)):
            pose_port.add(PoseSample(t, Pose2D(0.0, 0.0, 0.0)))
            event = fusion.ingest(frame(seq, t, sign_a), context)[0]
        self.assertIs(event.kind, models.EvidenceKind.POSITIVE)
        self.assertEqual(
            event.reason,
            'required_distinct_positive_frames_met',
        )

if __name__ == '__main__':
    unittest.main(verbosity=2)


def test_stale_timeline_old_run_wp06_evidence_cannot_mutate_current_map():
    """A stale WP06 event from an old run must fail closed at the map boundary."""
    from pathlib import Path

    import frob_city_navigation.sign_fusion as sign_fusion

    from frob_city_navigation.config_loader import load_project_config
    from frob_city_navigation.models import (
        ConfigPaths,
        EvidenceEvent,
        EvidenceKind,
    )
    from frob_city_navigation.semantic_map import SemanticMap

    # WP06 must expose/use the exact canonical Agent-1 evidence type.
    assert sign_fusion.EvidenceEvent is EvidenceEvent

    fixture = Path(__file__).parent / "fixtures" / "valid"
    config = load_project_config(
        ConfigPaths(
            field=fixture / "field.yaml",
            event=fixture / "event.yaml",
            perception=fixture / "perception.yaml",
            navigation=fixture / "navigation.yaml",
        )
    )

    semantic_map = SemanticMap(config.field)
    semantic_map.begin_run(
        run_id="run-current",
        field_id="field-test",
    )

    stale_event = sign_fusion.EvidenceEvent(
        run_id="run-old",
        event_id="wp06-stale-old-run",
        timestamp_s=1.0,
        kind=EvidenceKind.QUALIFIED_CLEAR,
        approach_id="C1_0/FROM_WEST",
        reason="stale timeline from previous run",
    )

    update = semantic_map.apply(stale_event)

    assert not update.accepted
    assert not update.changed
    assert semantic_map.snapshot().run_id == "run-current"
    assert semantic_map.snapshot().revision == 0
