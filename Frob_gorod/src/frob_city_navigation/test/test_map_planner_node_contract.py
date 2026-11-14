from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

# This test is shipped in two layouts:
# 1. inside the handoff archive: ROOT/candidate/frob_city_navigation/...
# 2. after installation:       ROOT/frob_city_navigation/...
# Always prefer the installed production source when it exists.
_INSTALLED_NODE = ROOT / 'frob_city_navigation' / 'map_planner_node.py'
_ARCHIVE_NODE = ROOT / 'candidate' / 'frob_city_navigation' / 'map_planner_node.py'
NODE_PATH = _INSTALLED_NODE if _INSTALLED_NODE.is_file() else _ARCHIVE_NODE

if not NODE_PATH.is_file():
    raise FileNotFoundError(
        'map_planner_node.py not found in installed or archive layout: '
        f'{_INSTALLED_NODE} ; {_ARCHIVE_NODE}'
    )

FIELD_PATH = ROOT / 'config' / 'wp11b_test_field.yaml'
if not FIELD_PATH.is_file():
    raise FileNotFoundError(f'WP11B test field not found: {FIELD_PATH}')

NODE = NODE_PATH.read_text(encoding='utf-8')
FIELD = FIELD_PATH.read_text(encoding='utf-8')


def test_wp11b_frozen_runtime_endpoint_names_present():
    for endpoint in (
        '"/traffic_sign/detections"',
        '"/odom"',
        '"/city/map_state"',
        '"/city/plan_route"',
        '"/city/validate_segment"',
        '"/city/report_execution"',
        '"/city/reset_map"',
        '"/city/observe"',
        '"/diagnostics"',
    ):
        assert endpoint in NODE


def test_wp11b_integrates_real_wp06_without_production_fake():
    assert 'self._wp06_integrated = True' in NODE
    assert 'SignFusion(' in NODE
    assert 'ObservationSession()' in NODE
    assert 'CanonicalDetectionFrame(' in NODE
    assert 'ObservationContext(' in NODE
    assert 'ObservationFrame(' in NODE
    assert 'WP06 fusion/observation integration pending' not in NODE
    assert 'fake_wp06' not in NODE.lower()
    assert 'test.fakes' not in NODE


def test_wp11b_detection_evidence_reaches_semantic_map():
    assert 'evidence = self._fusion.ingest(frame, context)' in NODE
    assert 'self._map.apply(event)' in NODE
    assert '{"POSITIVE", "CONFLICT", "PRECAUTIONARY_HOLD"}' in NODE
    assert 'EvidenceKind.QUALIFIED_CLEAR' in NODE
    assert 'EvidenceKind.INCONCLUSIVE' in NODE


def test_wp11b_capture_time_pose_does_not_fall_back_to_latest():
    assert 'class _PoseHistoryPort' in NODE
    assert 'capture_pose_not_bracketed' in NODE
    assert 'capture_pose_gap_too_large' in NODE
    assert 'self._pose_history.pose_at(capture_stamp_s)' in NODE
    detection = NODE.split('def _on_detections', 1)[1].split(
        '# ------------------------------------------------------------------\n    # Snapshot / diagnostics', 1
    )[0]
    assert 'self._latest_pose' not in detection


def test_wp11b_observe_is_executor_cooperative_and_session_mutation_is_locked():
    assert 'callback_group=self._observe_group' in NODE
    assert 'async def _execute_observe' in NODE
    assert 'await self._executor_sleep(0.02)' in NODE
    assert 'asyncio.sleep' not in NODE
    assert 'executor = self.executor' in NODE
    assert 'future = executor.create_future()' in NODE
    assert 'await future' in NODE
    assert 'outcome = active.session.finish' in NODE
    assert 'if self._active_observation is not active' in NODE


def test_wp11b_reset_resets_fusion_and_runtime_session():
    assert 'self._fusion.begin_run(request.new_run_id)' in NODE
    assert 'self._active_observation = None' in NODE


def test_wp11b_exact_time_tf_deferral_remains_bounded():
    assert 'self._pending_odom' in NODE
    assert 'self._retry_pending_odom' in NODE
    assert 'timeouts.tf_lookup_s' in NODE
    assert 'timeout=Duration(seconds=0.0)' in NODE
    assert 'lookup_transform(' in NODE


def test_wp11b_pending_odometry_is_fifo_and_new_frames_do_not_starve_tf():
    assert 'deque[tuple[Odometry, float]]' in NODE
    assert 'self._pending_odom.append(' in NODE
    assert 'self._pending_odom.popleft()' in NODE
    assert 'self._pending_odom = msg' not in NODE


def test_wp11b_synthetic_calibrated_fixture_is_explicitly_test_only():
    assert 'WP11 COMPONENT TEST FIXTURE ONLY' in FIELD
    assert 'NOT COMPETITION-CALIBRATED' in FIELD
    assert 'geometry_calibrated: true' in FIELD
    assert 'valid: true' in FIELD


def test_wp11b_shutdown_remains_idempotent():
    assert 'if rclpy.ok():' in NODE
    assert 'rclpy.shutdown()' in NODE
