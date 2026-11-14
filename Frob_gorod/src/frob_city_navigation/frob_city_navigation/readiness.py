"""Pure readiness decision logic for the Frob City headless stack (WP13)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True, slots=True)
class ReadinessConfig:
    odom_max_age_s: float = 0.40
    perception_max_age_s: float = 1.00
    map_max_age_s: float = 1.25
    guard_max_age_s: float = 0.50
    require_perception: bool = True
    require_tf: bool = True
    require_nav2: bool = True
    require_command_chain: bool = True
    require_start_pose: bool = False
    require_driver_subscriber: bool = True
    replay_mode: bool = False


@dataclass(frozen=True, slots=True)
class ReadinessSample:
    configuration_ok: bool
    topic_types_ok: bool
    tf_ok: bool
    odom_age_s: float | None
    perception_age_s: float | None
    map_age_s: float | None
    map_ready: bool
    localization_healthy: bool
    perception_healthy: bool
    guard_age_s: float | None
    guard_fault_latched: bool
    guard_stopped: bool
    nav2_ready: bool
    command_chain_ok: bool
    start_pose_ok: bool
    replay_isolated: bool


@dataclass(frozen=True, slots=True)
class ReadinessCheck:
    ok: bool
    reason: str


@dataclass(frozen=True, slots=True)
class ReadinessDecision:
    ready: bool
    checks: Mapping[str, ReadinessCheck]


def _fresh(age_s: float | None, limit_s: float, missing: str, stale: str) -> ReadinessCheck:
    if age_s is None:
        return ReadinessCheck(False, missing)
    if age_s < 0.0:
        return ReadinessCheck(False, 'timestamp_from_future')
    if age_s > limit_s:
        return ReadinessCheck(False, f'{stale}:{age_s:.3f}>{limit_s:.3f}')
    return ReadinessCheck(True, f'fresh:{age_s:.3f}s')


def evaluate_readiness(config: ReadinessConfig, sample: ReadinessSample) -> ReadinessDecision:
    checks: dict[str, ReadinessCheck] = {}
    checks['configuration'] = ReadinessCheck(
        bool(sample.configuration_ok),
        'configuration_valid' if sample.configuration_ok else 'configuration_missing_or_changed',
    )
    checks['topic_types'] = ReadinessCheck(
        bool(sample.topic_types_ok),
        'topic_types_match' if sample.topic_types_ok else 'topic_type_mismatch_or_missing',
    )
    checks['tf'] = (
        ReadinessCheck(bool(sample.tf_ok), 'tf_available' if sample.tf_ok else 'tf_unavailable')
        if config.require_tf
        else ReadinessCheck(True, 'tf_not_required')
    )
    checks['odometry'] = _fresh(
        sample.odom_age_s, config.odom_max_age_s, 'odometry_missing', 'odometry_stale'
    )
    if config.require_perception:
        p = _fresh(
            sample.perception_age_s,
            config.perception_max_age_s,
            'perception_missing',
            'perception_stale',
        )
        if p.ok and not sample.perception_healthy:
            p = ReadinessCheck(False, 'perception_reported_unhealthy')
        checks['perception'] = p
    else:
        checks['perception'] = ReadinessCheck(True, 'perception_not_required')
    m = _fresh(sample.map_age_s, config.map_max_age_s, 'map_missing', 'map_stale')
    if m.ok and not sample.map_ready:
        m = ReadinessCheck(False, 'map_not_ready')
    if m.ok and not sample.localization_healthy:
        m = ReadinessCheck(False, 'map_localization_unhealthy')
    checks['map_planner'] = m
    g = _fresh(sample.guard_age_s, config.guard_max_age_s, 'guard_missing', 'guard_stale')
    if g.ok and sample.guard_fault_latched:
        g = ReadinessCheck(False, 'guard_fault_latched')
    if g.ok and not sample.guard_stopped:
        g = ReadinessCheck(False, 'guard_not_stopped')
    checks['guard'] = g
    checks['nav2'] = (
        ReadinessCheck(bool(sample.nav2_ready), 'follow_path_ready' if sample.nav2_ready else 'follow_path_unavailable')
        if config.require_nav2
        else ReadinessCheck(True, 'nav2_not_required')
    )
    checks['command_chain'] = (
        ReadinessCheck(
            bool(sample.command_chain_ok),
            'single_guard_driver_path' if sample.command_chain_ok else 'command_chain_ownership_invalid',
        )
        if config.require_command_chain
        else ReadinessCheck(True, 'command_chain_not_required')
    )
    checks['start_pose'] = (
        ReadinessCheck(
            bool(sample.start_pose_ok),
            'start_pose_consistent' if sample.start_pose_ok else 'start_pose_inconsistent',
        )
        if config.require_start_pose
        else ReadinessCheck(True, 'start_pose_not_required')
    )
    checks['replay_isolation'] = (
        ReadinessCheck(
            bool(sample.replay_isolated),
            'replay_driver_isolated' if sample.replay_isolated else 'replay_real_driver_topic_active',
        )
        if config.replay_mode
        else ReadinessCheck(True, 'autonomy_mode')
    )
    return ReadinessDecision(ready=all(v.ok for v in checks.values()), checks=checks)
