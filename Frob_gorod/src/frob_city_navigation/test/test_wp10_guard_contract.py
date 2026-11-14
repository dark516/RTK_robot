import ast
from pathlib import Path

import pytest

from frob_city_navigation.models import Point2D, Polygon2D
from frob_city_navigation.motion_guard_core import GuardConfig


def test_motion_guard_core_has_no_ros_imports():
    path = Path(__file__).parents[1] / 'candidate' / 'frob_city_navigation' / 'motion_guard_core.py'
    # Installed test path fallback.
    if not path.exists():
        import frob_city_navigation.motion_guard_core as module
        path = Path(module.__file__)
    tree = ast.parse(path.read_text())
    banned = ('rclpy', 'geometry_msgs', 'nav_msgs', 'tf2_ros', 'frob_city_interfaces')
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.append(node.module)
    assert not [name for name in imported if name.startswith(banned)]


def test_guard_core_does_not_import_planner_or_semantic_map():
    import frob_city_navigation.motion_guard_core as module
    text = Path(module.__file__).read_text()
    assert 'from frob_city_navigation.planner' not in text
    assert 'from frob_city_navigation.semantic_map' not in text
    assert 'allowed_movements(' not in text


def test_physical_braking_and_footprint_are_explicit_configuration():
    fp = Polygon2D((Point2D(-0.1, -0.1), Point2D(0.1, -0.1), Point2D(0.0, 0.1)))
    with pytest.raises(TypeError):
        GuardConfig(field_id='FIELD')
    cfg = GuardConfig(
        field_id='FIELD',
        footprint=fp,
        braking_deceleration_mps2=1.0,
        braking_latency_s=0.05,
        hard_max_linear_speed_mps=0.3,
        hard_max_yaw_rate_radps=1.0,
        path_lateral_tolerance_m=0.1,
    )
    assert cfg.footprint is fp
