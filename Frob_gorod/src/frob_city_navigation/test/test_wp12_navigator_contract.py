from pathlib import Path


def _find_node():
    here = Path(__file__).resolve()
    candidates = [
        here.parents[1] / 'candidate' / 'frob_city_navigation' / 'navigator_node.py',
        here.parents[1] / 'frob_city_navigation' / 'navigator_node.py',
    ]
    for p in candidates:
        if p.exists():
            return p.read_text(encoding='utf-8')
    raise FileNotFoundError(candidates)


NODE = _find_node()


def test_navigator_uses_wp09_machine_and_follow_path_adapter():
    assert 'self._mission = MissionMachine()' in NODE
    assert 'self._nav2 = Nav2Adapter(' in NODE
    assert 'route_segment_to_nav_path(' in NODE
    assert 'PoseStamped' in NODE
    assert 'NavigateToPose' not in NODE


def test_permit_only_enabled_after_nav2_acceptance():
    accepted = NODE.index('def _on_nav2_accepted')
    result = NODE.index('def _on_nav2_result')
    block = NODE[accepted:result]
    assert 'if not accepted:' in block
    assert 'self._permit_enabled = True' in block
    assert 'self._publish_permit(enabled=True)' in block


def test_guard_generation_barrier_is_a_handshake_not_an_immediate_cancel():
    from frob_city_navigation.navigator_node import _guard_denial_is_fatal

    common = dict(
        permit_enabled=True,
        message_generation=2,
        active_generation=2,
        motion_allowed=False,
    )
    assert not _guard_denial_is_fatal(
        **common,
        previously_allowed_generation=0,
    )
    assert _guard_denial_is_fatal(
        **common,
        previously_allowed_generation=2,
    )


def test_mission_state_uses_reliable_transient_local_qos():
    start = NODE.index('self._mission_pub = self.create_publisher(')
    end = NODE.index('self.create_subscription(', start)
    assert 'map_qos' in NODE[start:end]


def test_every_nav2_result_revokes_permit_before_success_processing():
    start = NODE.index('def _on_nav2_result')
    end = NODE.index('def _request_observe', start)
    block = NODE[start:end]
    off = block.index("self._force_motion_disabled('nav2_result'")
    success = block.index('Nav2Completion.SUCCEEDED')
    assert off < success


def test_nav2_success_requires_terminal_verification_before_execution_complete():
    result_start = NODE.index('def _on_nav2_result')
    verify = NODE.index('def _terminal_verified')
    complete = NODE.index('def _complete_verified_segment')
    assert result_start < verify < complete
    result_block = NODE[result_start:NODE.index('def _request_observe', result_start)]
    assert 'MissionEventKind.EXECUTION_COMPLETED' not in result_block
    complete_block = NODE[complete:NODE.index('def _begin_service_verification', complete)]
    assert 'MissionEventKind.EXECUTION_COMPLETED' in complete_block


def test_safe_pause_order_is_permit_off_then_cancel():
    start = NODE.index('def _safe_pause')
    end = NODE.index('def _reset_after_mission_reset', start)
    block = NODE[start:end]
    assert block.index('self._force_motion_disabled(reason)') < block.index('self._nav2.cancel_active()')


def test_map_revision_change_revalidates_not_blindly_cancels():
    assert 'self._request_live_revalidation()' in NODE
    assert "live_revalidation_failed" in NODE


def test_one_active_execution_token_and_stale_generation_checks_exist():
    assert 'if self._active_token is not None or self._nav2.has_active_goal:' in NODE
    assert 'token.goal_generation == state.goal_generation' in NODE
    assert 'token.route_id == state.route_id' in NODE
    assert 'token.segment_id == state.active_segment_id' in NODE


def test_no_asyncio_sleep_or_raw_cmd_vel_bypass():
    assert 'asyncio.sleep' not in NODE
    assert '/cmd_vel' not in NODE
    assert '/city/cmd_vel_raw' not in NODE
