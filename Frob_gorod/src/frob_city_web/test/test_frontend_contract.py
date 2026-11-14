from pathlib import Path


STATIC = Path(__file__).parents[1] / "frob_city_web" / "static"


def test_dashboard_contains_all_operator_surfaces():
    html = (STATIC / "index.html").read_text(encoding="utf-8")
    for required in (
        'lang="ru"', 'name="viewport"', 'id="map-canvas"', 'id="camera-stream"',
        'id="mission-panel"', 'id="guard-panel"', 'id="perception-panel"',
        'id="readiness-panel"', 'id="event-panel"', 'id="control-deck"',
        'id="disconnect-banner"',
    ):
        assert required in html
    for command in ("START", "PAUSE", "RESUME", "ABORT", "RESET"):
        assert f'data-command="{command}"' in html


def test_frontend_is_local_accessible_and_has_no_velocity_control():
    html = (STATIC / "index.html").read_text(encoding="utf-8")
    css = (STATIC / "app.css").read_text(encoding="utf-8")
    js = (STATIC / "app.js").read_text(encoding="utf-8")
    joined = html + css + js
    assert "https://" not in joined and "http://" not in joined
    assert "@media" in css and "prefers-reduced-motion" in css
    assert "aria-label" in html
    assert "/api/events" in js and "/api/snapshot" in js
    assert "/api/camera.mjpg" in js and "/api/control" in js
    assert "cmd_vel" not in js
    assert "EventSource" in js
    assert "requestAnimationFrame" in js


def test_camera_can_be_disabled_and_abort_is_not_confirmed():
    js = (STATIC / "app.js").read_text(encoding="utf-8")
    assert "camera-enabled" in js
    assert 'command === "START" || command === "RESET"' in js
    assert 'command === "ABORT"' in js
