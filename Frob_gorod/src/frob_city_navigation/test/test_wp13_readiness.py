from frob_city_navigation.readiness import (
    ReadinessConfig, ReadinessSample, evaluate_readiness,
)


def good(**changes):
    values = dict(
        configuration_ok=True, topic_types_ok=True, tf_ok=True,
        odom_age_s=0.05, perception_age_s=0.05, map_age_s=0.05,
        map_ready=True, localization_healthy=True, perception_healthy=True,
        guard_age_s=0.05, guard_fault_latched=False, guard_stopped=True,
        nav2_ready=True, command_chain_ok=True, start_pose_ok=True,
        replay_isolated=True,
    )
    values.update(changes)
    return ReadinessSample(**values)


def test_all_healthy_is_ready():
    assert evaluate_readiness(ReadinessConfig(), good()).ready


def test_each_mandatory_health_class_fails_closed():
    cases = [
        {'configuration_ok': False}, {'topic_types_ok': False}, {'tf_ok': False},
        {'odom_age_s': 5.0}, {'perception_age_s': None}, {'map_ready': False},
        {'localization_healthy': False}, {'guard_fault_latched': True},
        {'nav2_ready': False}, {'command_chain_ok': False}, {'guard_stopped': False},
    ]
    for case in cases:
        assert not evaluate_readiness(ReadinessConfig(), good(**case)).ready, case


def test_perception_can_be_explicitly_nonmandatory_for_special_replay():
    cfg = ReadinessConfig(require_perception=False)
    assert evaluate_readiness(cfg, good(perception_age_s=None, perception_healthy=False)).ready


def test_start_pose_is_fail_closed_only_when_required():
    assert evaluate_readiness(ReadinessConfig(require_start_pose=False), good(start_pose_ok=False)).ready
    assert not evaluate_readiness(ReadinessConfig(require_start_pose=True), good(start_pose_ok=False)).ready


def test_replay_must_not_resolve_real_driver_topic():
    cfg = ReadinessConfig(replay_mode=True)
    assert evaluate_readiness(cfg, good(replay_isolated=True)).ready
    decision = evaluate_readiness(cfg, good(replay_isolated=False))
    assert not decision.ready
    assert decision.checks['replay_isolation'].reason == 'replay_real_driver_topic_active'


def test_missing_and_stale_have_distinct_reasons():
    missing = evaluate_readiness(ReadinessConfig(), good(odom_age_s=None))
    stale = evaluate_readiness(ReadinessConfig(), good(odom_age_s=9.0))
    assert missing.checks['odometry'].reason == 'odometry_missing'
    assert stale.checks['odometry'].reason.startswith('odometry_stale:')
