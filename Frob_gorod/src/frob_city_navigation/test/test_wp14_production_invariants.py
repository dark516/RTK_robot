from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'frob_city_navigation'
if not PKG.exists():
    # Bundle mode: production source is intentionally not bundled/patched.
    PKG=None

def test_wp14_does_not_install_private_planner_or_second_mission_fsm():
    # This WP is integration/replay only. There must be no candidate production tree.
    bundle=Path(__file__).resolve().parents[1]
    assert not (bundle/'candidate').exists() or not any((bundle/'candidate').rglob('*.py'))

def test_installed_navigator_preserves_permission_off_before_cancel_when_available():
    if PKG is None:return
    src=(PKG/'navigator_node.py').read_text()
    block=src[src.index('def _safe_pause'):src.index('def _reset_after_mission_reset')]
    assert block.index("self._force_motion_disabled(reason)") < block.index('self._nav2.cancel_active()')

def test_installed_replay_launch_never_targets_real_driver_when_available():
    launch=ROOT/'launch'/'city_replay.launch.py'
    if not launch.exists():return
    text=launch.read_text()
    assert "'driver_cmd_topic': '/city/replay/cmd_vel_sink'" in text
    assert "'real_driver_topic': '/cmd_vel'" in text


def _wp14_bundle_root() -> Path:
    here = Path(__file__).resolve()
    # bundle: <root>/tests/test_... ; installed: <pkg>/test/test_...
    candidate = here.parents[1]
    if (candidate / 'config' / 'wp14_navigator_base.yaml').exists():
        return candidate
    # Installed mode keeps the fixture under test/fixtures/wp14 and the live
    # harness itself only in the work-package bundle, so validate the copied
    # YAML directly here and source-level override through installed snapshot
    # only when available.
    return candidate


def test_wp14_boarding_dwell_never_weakens_wp09_two_second_invariant():
    import re
    import yaml
    here = Path(__file__).resolve()
    bundle_root = here.parents[1]
    yaml_path = bundle_root / 'config' / 'wp14_navigator_base.yaml'
    if not yaml_path.exists():
        yaml_path = bundle_root / 'test' / 'fixtures' / 'wp14' / 'wp14_navigator_base.yaml'
    data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    params = data['/**']['ros__parameters']
    assert float(params['boarding_dwell_s']) >= 2.0

    # Bundle mode additionally checks the executable live scenario does not
    # override the validated fixture with a sub-2-second value.
    live = bundle_root / 'tools' / 'wp14_full_mission.py'
    if live.exists():
        text = live.read_text(encoding='utf-8')
        values = [float(v) for v in re.findall(r'boarding_dwell_s:=([0-9.]+)', text)]
        assert values
        assert min(values) >= 2.0
