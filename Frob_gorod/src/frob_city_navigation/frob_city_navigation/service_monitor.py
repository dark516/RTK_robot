"""WP09 pure continuous stop/service dwell monitor.

ROS-independent. Time passed to this module is monotonic time in seconds.
The monitor never infers that a zero command means the robot is stopped;
measured motion/stopped state is part of the predicate.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math


class ServiceMode(IntEnum):
    BOARDING = 0
    PARKING = 1


class ServiceState(IntEnum):
    WAITING = 0
    DWELLING = 1
    COMPLETE = 2


@dataclass(frozen=True, slots=True)
class ServiceMonitorConfig:
    mode: ServiceMode = ServiceMode.BOARDING
    required_dwell_s: float = 2.2
    stationary_linear_max_mps: float = 0.01
    stationary_angular_max_radps: float = 0.03

    def __post_init__(self) -> None:
        for name, value in (
            ('required_dwell_s', self.required_dwell_s),
            ('stationary_linear_max_mps', self.stationary_linear_max_mps),
            ('stationary_angular_max_radps', self.stationary_angular_max_radps),
        ):
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f'{name} must be finite and >= 0')
        if self.mode is ServiceMode.BOARDING and self.required_dwell_s < 2.0:
            raise ValueError('boarding dwell must never be configured below 2.0 s')


@dataclass(frozen=True, slots=True)
class StopPredicateSample:
    run_id: str
    map_instance_id: str
    target_id: str
    target_confirmed: bool
    geometry_valid: bool
    correct_directed_approach: bool
    within_xy_tolerance: bool
    within_yaw_tolerance: bool
    front_before_terminal_limit: bool
    measured_linear_speed_mps: float
    measured_angular_speed_radps: float
    odometry_fresh: bool
    localization_fresh: bool
    guard_zero_output: bool
    guard_measured_stopped: bool
    movement_action_active: bool
    target_conflict: bool
    identity_current: bool


@dataclass(frozen=True, slots=True)
class ServiceProgress:
    state: ServiceState
    target_id: str
    valid_now: bool
    dwell_s: float
    completed_now: bool
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    reason: str


class ServiceMonitor:
    """Accumulates dwell only while every measured stop predicate stays true."""

    def __init__(self, config: ServiceMonitorConfig | None = None) -> None:
        self._config = config or ServiceMonitorConfig()
        self._target_id = ''
        self._run_id = ''
        self._map_instance_id = ''
        self._started_at_s: float | None = None
        self._last_now_s: float | None = None
        self._complete = False
        self._max_linear_speed_mps = 0.0
        self._max_angular_speed_radps = 0.0
        self._last_reason = 'not_started'

    def reset(self, reason: str) -> None:
        self._target_id = ''
        self._run_id = ''
        self._map_instance_id = ''
        self._started_at_s = None
        self._last_now_s = None
        self._complete = False
        self._max_linear_speed_mps = 0.0
        self._max_angular_speed_radps = 0.0
        self._last_reason = reason or 'reset'

    def update(self, sample: StopPredicateSample, now: float) -> ServiceProgress:
        now = float(now)
        if not math.isfinite(now):
            self._clear_dwell('monotonic_time_non_finite')
            return self._progress(False, 0.0, False, 'monotonic_time_non_finite')
        if self._last_now_s is not None and now < self._last_now_s:
            self._clear_dwell('monotonic_time_went_backwards')
            self._last_now_s = now
            return self._progress(False, 0.0, False, 'monotonic_time_went_backwards')
        self._last_now_s = now

        reason = self._invalid_reason(sample)
        if reason is not None:
            self._clear_dwell(reason)
            return self._progress(False, 0.0, False, reason)

        identity = (sample.run_id, sample.map_instance_id, sample.target_id)
        current = (self._run_id, self._map_instance_id, self._target_id)
        if self._started_at_s is None or identity != current:
            self._run_id, self._map_instance_id, self._target_id = identity
            self._started_at_s = now
            self._complete = False
            self._max_linear_speed_mps = abs(float(sample.measured_linear_speed_mps))
            self._max_angular_speed_radps = abs(float(sample.measured_angular_speed_radps))
            self._last_reason = 'dwell_started'
            return self._progress(True, 0.0, False, 'dwell_started')

        self._max_linear_speed_mps = max(
            self._max_linear_speed_mps, abs(float(sample.measured_linear_speed_mps))
        )
        self._max_angular_speed_radps = max(
            self._max_angular_speed_radps, abs(float(sample.measured_angular_speed_radps))
        )
        dwell = max(0.0, now - self._started_at_s)
        completed_now = (not self._complete) and (dwell + 1e-9) >= self._config.required_dwell_s
        if completed_now:
            self._complete = True
            self._last_reason = 'required_continuous_dwell_met'
        elif self._complete:
            self._last_reason = 'service_complete'
        else:
            self._last_reason = 'dwell_in_progress'
        return self._progress(True, dwell, completed_now, self._last_reason)

    def _clear_dwell(self, reason: str) -> None:
        self._target_id = ''
        self._run_id = ''
        self._map_instance_id = ''
        self._started_at_s = None
        self._complete = False
        self._max_linear_speed_mps = 0.0
        self._max_angular_speed_radps = 0.0
        self._last_reason = reason

    def _invalid_reason(self, s: StopPredicateSample) -> str | None:
        if not s.run_id or not s.map_instance_id or not s.target_id:
            return 'identity_missing'
        if not s.identity_current:
            return 'identity_stale'
        if not s.target_confirmed:
            return 'target_not_confirmed'
        if s.target_conflict:
            return 'target_conflict'
        if not s.geometry_valid:
            return 'service_geometry_invalid'
        if not s.correct_directed_approach:
            return 'wrong_directed_approach'
        if not s.within_xy_tolerance:
            return 'outside_xy_tolerance'
        if not s.within_yaw_tolerance:
            return 'outside_yaw_tolerance'
        if not s.front_before_terminal_limit:
            return 'terminal_limit_crossed'
        if not s.odometry_fresh:
            return 'odometry_stale'
        if not s.localization_fresh:
            return 'localization_stale'
        if not s.guard_zero_output:
            return 'guard_not_zero'
        if not s.guard_measured_stopped:
            return 'guard_not_measured_stopped'
        if s.movement_action_active:
            return 'movement_action_active'
        lin = float(s.measured_linear_speed_mps)
        ang = float(s.measured_angular_speed_radps)
        if not math.isfinite(lin) or not math.isfinite(ang):
            return 'measured_speed_non_finite'
        if abs(lin) > self._config.stationary_linear_max_mps:
            return 'linear_motion_detected'
        if abs(ang) > self._config.stationary_angular_max_radps:
            return 'angular_motion_detected'
        return None

    def _progress(self, valid: bool, dwell: float, completed_now: bool, reason: str) -> ServiceProgress:
        state = ServiceState.COMPLETE if self._complete else (
            ServiceState.DWELLING if self._started_at_s is not None else ServiceState.WAITING
        )
        return ServiceProgress(
            state=state,
            target_id=self._target_id,
            valid_now=valid,
            dwell_s=dwell,
            completed_now=completed_now,
            max_linear_speed_mps=self._max_linear_speed_mps,
            max_angular_speed_radps=self._max_angular_speed_radps,
            reason=reason,
        )
