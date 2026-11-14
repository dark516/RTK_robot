from pathlib import Path


def root():
    here = Path(__file__).resolve()
    for r in (here.parents[1], here.parents[2]):
        if (r / 'launch/city_autonomy.launch.py').is_file():
            return r
    raise AssertionError('WP13 launch root not found')


def test_autonomy_has_required_include_arguments_and_no_manual_goal_sources():
    src = (root() / 'launch/city_autonomy.launch.py').read_text()
    for arg in ('include_robot_bringup', 'include_localization', 'include_nav2', 'include_detector', 'include_detection_adapter'):
        assert f"DeclareLaunchArgument('{arg}'" in src
    low = src.lower()
    for forbidden in ('keyboard_teleop', 'joy_control', 'rviz2', 'teleop_twist_keyboard'):
        assert forbidden not in low


def test_autonomy_launch_wires_guard_between_raw_and_driver_topics():
    src = (root() / 'launch/city_autonomy.launch.py').read_text()
    assert "'raw_cmd_topic': raw_cmd" in src
    assert "'driver_cmd_topic': driver_cmd" in src
    assert "'require_system_ready': True" in src


def test_legacy_detector_is_adapted_on_a_distinct_topic():
    src = (root() / 'launch/city_autonomy.launch.py').read_text()
    assert "DeclareLaunchArgument('legacy_detections_topic'" in src
    assert "executable='detection_adapter'" in src
    assert "'input_topic': LaunchConfiguration('legacy_detections_topic')" in src
    assert "'output_topic': detections_topic" in src


def test_replay_is_hardware_isolated():
    src = (root() / 'launch/city_replay.launch.py').read_text()
    assert "'include_robot_bringup': 'false'" in src
    assert "'driver_cmd_topic': '/city/replay/cmd_vel_sink'" in src
    assert "'real_driver_topic': '/cmd_vel'" in src
    assert "'replay_mode': 'true'" in src


def test_wp13_empty_model_files_remain_typed_string_array():
    """Empty optional model list must never become an untyped launch tuple."""
    autonomy = (root() / 'launch/city_autonomy.launch.py').read_text()
    assert "'model_files': model_files if model_files else ['']" in autonomy
    assert "'model_files': model_files," not in autonomy


def _wp13_live_harness_source() -> str:
    """Read the live harness from either the WP bundle or installed test snapshot."""
    here = Path(__file__).resolve()
    candidates = (
        here.parents[1] / 'tools' / 'wp13_live_headless.py',
        here.parent / 'wp13_live_headless_harness.py',
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.read_text(encoding='utf-8')
    raise AssertionError(
        'WP13 live harness contract source missing; checked: '
        + ', '.join(str(p) for p in candidates)
    )


def test_wp13_live_harness_kills_complete_launch_process_group():
    live = _wp13_live_harness_source()
    assert 'start_new_session=True' in live
    assert 'os.killpg(proc.pid, signal.SIGTERM)' in live
    assert 'os.killpg(proc.pid, signal.SIGKILL)' in live


def _wp13_readiness_node_source() -> str:
    here = Path(__file__).resolve()
    candidates = (
        here.parents[1] / 'candidate' / 'frob_city_navigation' / 'readiness_node.py',
        here.parents[1] / 'frob_city_navigation' / 'readiness_node.py',
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.read_text(encoding='utf-8')
    raise AssertionError(
        'WP13 readiness node contract source missing; checked: '
        + ', '.join(str(p) for p in candidates)
    )


def test_wp13_replay_readiness_checks_real_driver_publishers():
    node = _wp13_readiness_node_source()
    assert "get_publishers_info_by_topic(real_topic)" in node
    assert "driver_topic != real_topic and not self._replay_real_driver_publishers" in node
    assert "key='real_driver_publishers'" in node


def test_wp13_live_harness_uses_dedicated_ros_domain():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    candidates = [
        root / 'tools' / 'wp13_live_headless.py',
        root / 'test' / 'wp13_live_headless_harness.py',
    ]
    live_path = next((p for p in candidates if p.is_file()), None)
    assert live_path is not None
    live = live_path.read_text(encoding='utf-8')
    assert "WP13_ROS_DOMAIN_ID" in live
    assert "os.environ['ROS_DOMAIN_ID'] = test_domain" in live
    assert "start_new_session=True" in live
