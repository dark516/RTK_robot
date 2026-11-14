from pathlib import Path


def node_text():
    here = Path(__file__).resolve()
    candidates = [
        here.parents[1] / 'frob_city_navigation' / 'navigator_node.py',
        here.parents[1] / 'candidate' / 'frob_city_navigation' / 'navigator_node.py',
    ]
    for p in candidates:
        if p.is_file():
            return p.read_text()
    raise AssertionError(f'navigator source not found: {candidates}')


def test_system_readiness_is_optional_for_wp12_compatibility_but_available():
    src = node_text()
    assert "'require_system_ready': False" in src
    assert "'system_ready_topic': '/city/system_ready'" in src
    assert 'def _system_ready_gate_ok' in src


def test_start_is_rejected_before_reset_when_system_not_ready():
    src = node_text()
    gate = src.index("response.reason = 'system_not_ready'")
    reset = src.index("self._reset_client.service_is_ready()")
    assert gate < reset


def test_readiness_loss_during_motion_uses_safe_pause():
    src = node_text()
    assert "self._safe_pause('system_readiness_lost')" in src
    assert 'if not self._system_ready_gate_ok()' in src
