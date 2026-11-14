from pathlib import Path


def test_guard_node_has_required_endpoints_and_fail_closed_defaults():
    text = (Path(__file__).parents[1] / "frob_city_navigation" / "motion_guard_node.py").read_text()
    assert '"/city/cmd_vel_raw"' in text
    assert '"/cmd_vel"' in text
    assert '"/city/motion_permit"' in text
    assert '"/city/map_state"' in text
    assert '"/city/guard_state"' in text
    assert 'self.declare_parameter("commissioned", False)' in text
    assert 'self._publish_zero()' in text
    assert 'time.monotonic()' in text


def test_guard_node_uses_pure_core_and_no_planner_logic():
    text = (Path(__file__).parents[1] / "frob_city_navigation" / "motion_guard_node.py").read_text()
    assert "MotionGuardCore" in text
    assert "planner" not in text.lower()
    assert "dijkstra" not in text.lower()


def test_footprint_parameter_is_declared_as_numeric_array():
    text = (Path(__file__).parents[1] / "frob_city_navigation" / "motion_guard_node.py").read_text()
    assert 'declare_parameter("footprint_xy", [0.0])' in text
    assert 'declare_parameter("footprint_xy", [])' not in text
