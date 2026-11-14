from dataclasses import replace
import math
import pytest

from frob_city_navigation.service_monitor import (
    ServiceMode,
    ServiceMonitor,
    ServiceMonitorConfig,
    ServiceState,
    StopPredicateSample,
)


def valid(**changes):
    base = StopPredicateSample(
        run_id='run-1', map_instance_id='map-1', target_id='BUS_STOP:S:F',
        target_confirmed=True, geometry_valid=True, correct_directed_approach=True,
        within_xy_tolerance=True, within_yaw_tolerance=True,
        front_before_terminal_limit=True,
        measured_linear_speed_mps=0.0, measured_angular_speed_radps=0.0,
        odometry_fresh=True, localization_fresh=True,
        guard_zero_output=True, guard_measured_stopped=True,
        movement_action_active=False, target_conflict=False, identity_current=True,
    )
    return replace(base, **changes)


def test_boarding_2p2_continuous_seconds_completes_once():
    m = ServiceMonitor()
    assert m.update(valid(), 10.0).state is ServiceState.DWELLING
    assert not m.update(valid(), 12.19).completed_now
    done = m.update(valid(), 12.20)
    assert done.state is ServiceState.COMPLETE
    assert done.completed_now
    assert done.dwell_s == pytest.approx(2.2)
    assert not m.update(valid(), 12.30).completed_now


def test_parking_half_second_settle_is_allowed_and_completes():
    m = ServiceMonitor(ServiceMonitorConfig(mode=ServiceMode.PARKING, required_dwell_s=0.5))
    sample = valid(target_id='PARKING:P:F')
    m.update(sample, 1.0)
    assert not m.update(sample, 1.49).completed_now
    assert m.update(sample, 1.50).completed_now


def test_boarding_config_cannot_go_below_official_two_seconds():
    with pytest.raises(ValueError):
        ServiceMonitorConfig(mode=ServiceMode.BOARDING, required_dwell_s=1.99)


def test_drift_at_1p9_resets_and_requires_full_new_dwell():
    m = ServiceMonitor(); m.update(valid(), 0.0)
    p = m.update(valid(within_xy_tolerance=False), 1.9)
    assert p.state is ServiceState.WAITING
    assert p.reason == 'outside_xy_tolerance'
    assert m.update(valid(), 2.0).dwell_s == 0.0
    assert not m.update(valid(), 4.19).completed_now
    assert m.update(valid(), 4.20).completed_now


def test_stale_odometry_at_1p9_resets():
    m = ServiceMonitor(); m.update(valid(), 0.0)
    p = m.update(valid(odometry_fresh=False), 1.9)
    assert p.state is ServiceState.WAITING
    assert p.reason == 'odometry_stale'


def test_zero_guard_output_does_not_prove_stopped_if_measured_motion_continues():
    m = ServiceMonitor()
    p = m.update(
        valid(
            guard_zero_output=True,
            guard_measured_stopped=False,
            measured_linear_speed_mps=0.08,
        ),
        0.0,
    )
    assert p.state is ServiceState.WAITING
    assert p.reason == 'guard_not_measured_stopped'


def test_linear_or_angular_motion_resets():
    m = ServiceMonitor(); m.update(valid(), 0.0)
    assert m.update(valid(measured_linear_speed_mps=0.011), 1.0).state is ServiceState.WAITING
    m.update(valid(), 2.0)
    assert m.update(valid(measured_angular_speed_radps=0.031), 3.0).state is ServiceState.WAITING


def test_wrong_target_or_run_never_carries_old_dwell():
    m = ServiceMonitor(); m.update(valid(), 0.0); m.update(valid(), 1.5)
    p = m.update(valid(target_id='BUS_STOP:S2:F'), 1.6)
    assert p.dwell_s == 0.0
    p = m.update(valid(run_id='run-2', target_id='BUS_STOP:S2:F'), 1.7)
    assert p.dwell_s == 0.0


def test_conflict_terminal_crossing_active_action_and_stale_identity_fail_closed():
    cases = [
        ({'target_conflict': True}, 'target_conflict'),
        ({'front_before_terminal_limit': False}, 'terminal_limit_crossed'),
        ({'movement_action_active': True}, 'movement_action_active'),
        ({'identity_current': False}, 'identity_stale'),
    ]
    for change, reason in cases:
        p = ServiceMonitor().update(valid(**change), 0.0)
        assert p.state is ServiceState.WAITING
        assert p.reason == reason


def test_external_reset_discards_partial_dwell():
    m = ServiceMonitor(); m.update(valid(), 0.0); m.update(valid(), 1.8)
    m.reset('mission_paused')
    assert m.update(valid(), 2.0).dwell_s == 0.0
    assert not m.update(valid(), 4.19).completed_now
    assert m.update(valid(), 4.20).completed_now


def test_monotonic_time_backwards_fails_closed():
    m = ServiceMonitor(); m.update(valid(), 5.0)
    p = m.update(valid(), 4.0)
    assert p.state is ServiceState.WAITING
    assert p.reason == 'monotonic_time_went_backwards'


def test_nonfinite_speeds_fail_closed():
    assert ServiceMonitor().update(
        valid(measured_linear_speed_mps=math.nan), 0.0
    ).state is ServiceState.WAITING
