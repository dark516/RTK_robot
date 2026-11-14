from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_web_node_has_no_velocity_or_navigation_publishers():
    source = (ROOT / "frob_city_web" / "web_node.py").read_text(encoding="utf-8")
    assert "create_publisher" not in source
    assert "ActionClient" not in source
    assert "GUI_path" not in source
    assert "cmd_vel" in source  # observation is intentional
    assert "create_subscription" in source
    assert "create_client" in source


def test_web_node_does_not_shadow_rclpy_internal_endpoint_lists():
    source = (ROOT / "frob_city_web" / "web_node.py").read_text(encoding="utf-8")
    assert "self._subscriptions =" not in source


def test_control_allowlist_is_exact():
    source = (ROOT / "frob_city_web" / "http_server.py").read_text(encoding="utf-8")
    expected = {"START", "PAUSE", "RESUME", "ABORT", "RESET"}
    line = next(line for line in source.splitlines() if line.startswith("_CONTROL_COMMANDS"))
    assert {name for name in expected if f'"{name}"' in line} == expected
    assert "FORWARD" not in line and "BACKUP" not in line


def test_package_does_not_depend_on_web_frameworks():
    package = (ROOT / "package.xml").read_text(encoding="utf-8").lower()
    setup = (ROOT / "setup.py").read_text(encoding="utf-8").lower()
    for forbidden in ("flask", "fastapi", "aiohttp", "rosbridge"):
        assert forbidden not in package
        assert forbidden not in setup


def test_web_node_wires_calibration_as_read_only_ros_state():
    source = (ROOT / "frob_city_web" / "web_node.py").read_text(encoding="utf-8")
    config = (ROOT / "config" / "web.yaml").read_text(encoding="utf-8")
    assert "CalibrationService" in source
    assert "live_map_snapshot=self._calibration_live_map" in source
    for name in ("calibration_root", "calibration_map_roots", "calibration_live_map_stale_s"):
        assert name in source
        assert name in config
    assert "create_publisher" not in source
