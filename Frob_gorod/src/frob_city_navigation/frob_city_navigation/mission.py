"""WP09 pure mission state machine for Frob City.

This module is deliberately ROS/Nav2 independent.  It owns mission progress,
service receipts, objective selection, request/generation identity and phase /
execution state.  It does *not* calculate traffic legality or construct routes;
it emits commands for the future navigator wrapper to execute against the
map_planner contract.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable

from frob_city_navigation.models import Pose2D


class MissionPhase(IntEnum):
    IDLE = 0
    BOARDING = 1
    PARKING_PRIOR = 2
    PARKING_SEARCH = 3
    PARKING_APPROACH = 4
    DONE = 5
    PAUSED = 6
    ABORTED = 7


class ExecutionState(IntEnum):
    HOLDING = 0
    REQUESTING_PLAN = 1
    VALIDATING_SEGMENT = 2
    SENDING_GOAL = 3
    MOVING = 4
    CANCELING = 5
    OBSERVING = 6
    VERIFYING_STOP = 7
    DWELLING = 8


class ParkingPriorState(IntEnum):
    NOT_CHECKED = 0
    FOUND = 1
    CLEAR_ABSENT = 2
    INCONCLUSIVE = 3
    UNREACHABLE_NOW = 4
    INVALID = 5


class TargetKind(IntEnum):
    BUS_STOP = 6
    PARKING = 7


class PlanGoalKind(IntEnum):
    TARGET = 0
    EXPLORE_STOPS = 1
    EXPLORE_PARKING = 2
    CHECK_POI = 3


class PlanStatus(IntEnum):
    ROUTE_READY = 0
    NEED_OBSERVATION = 1
    ALREADY_AT_TERMINAL = 2
    NO_ROUTE = 3
    NO_REACHABLE_FRONTIER = 4
    INVALID_REQUEST = 5
    NOT_READY = 6
    STALE_SESSION = 7


class ValidationStatus(IntEnum):
    VALID = 0
    INVALID = 1
    NEED_OBSERVATION = 2
    NOT_READY = 3
    STALE_SESSION = 4


class ObservationOutcome(IntEnum):
    APPROACH_RESOLVED = 0
    TARGET_FOUND = 1
    CLEAR_ABSENT = 2
    INCONCLUSIVE = 3
    CONFLICT = 4
    NOT_IN_ZONE = 5
    STALE_SESSION = 6


class MissionEventKind(IntEnum):
    MAP_UPDATED = 0
    PLAN_RESULT = 1
    SEGMENT_VALIDATION_RESULT = 2
    GOAL_ACCEPTED = 3
    EXECUTION_COMPLETED = 4
    OBSERVATION_RESULT = 5
    SERVICE_COMPLETED = 6
    PARKING_STOP_VERIFIED = 7
    PAUSE = 8
    RESUME = 9
    ABORT = 10
    RESET = 11
    NO_PROGRESS = 12
    TAINT_ATTEMPT = 13


class MissionCommandKind(IntEnum):
    NONE = 0
    REQUEST_PLAN = 1
    REQUEST_VALIDATE_SEGMENT = 2
    SEND_SEGMENT = 3
    REQUEST_OBSERVE = 4
    VERIFY_STOP = 5
    START_DWELL = 6
    REPORT_EXECUTION = 7
    CANCEL_AND_HOLD = 8
    DISABLE_MOTION = 9
    COMPLETE = 10
    RESET_MAP = 11


@dataclass(frozen=True, slots=True)
class RunContext:
    run_id: str
    map_instance_id: str
    navigator_instance_id: str
    required_unique_bus_stops: int = 2
    parking_prior_enabled: bool = True
    parking_prior_valid: bool = True
    parking_prior_observation_id: str = ""
    known_bus_stop_ids: tuple[str, ...] = ()
    known_parking_target_id: str = ""
    map_revision: int = 0

    def __post_init__(self) -> None:
        if not self.run_id or not self.map_instance_id or not self.navigator_instance_id:
            raise ValueError("run/map/navigator identity must be non-empty")
        if self.required_unique_bus_stops < 1:
            raise ValueError("required_unique_bus_stops must be >= 1")
        if self.map_revision < 0:
            raise ValueError("map_revision must be >= 0")


@dataclass(frozen=True, slots=True)
class ServiceReceipt:
    run_id: str
    target_id: str
    stop_entry_s: float
    stop_exit_s: float
    dwell_duration_s: float
    final_pose: Pose2D
    max_measured_speed_mps: float
    map_revision: int

    def __post_init__(self) -> None:
        if not self.run_id or not self.target_id:
            raise ValueError("receipt identity must be non-empty")
        if self.stop_exit_s < self.stop_entry_s:
            raise ValueError("stop_exit_s must be >= stop_entry_s")
        if self.dwell_duration_s < 0.0:
            raise ValueError("dwell_duration_s must be >= 0")
        if self.max_measured_speed_mps < 0.0:
            raise ValueError("max_measured_speed_mps must be >= 0")
        if self.map_revision < 0:
            raise ValueError("map_revision must be >= 0")


@dataclass(frozen=True, slots=True)
class MissionCommand:
    kind: MissionCommandKind
    request_id: int = 0
    goal_kind: PlanGoalKind = PlanGoalKind.TARGET
    target_id: str = ""
    route_id: str = ""
    segment_id: str = ""
    observation_id: str = ""
    goal_generation: int = 0
    reason: str = ""


@dataclass(frozen=True, slots=True)
class MissionSnapshot:
    run_id: str
    map_instance_id: str
    navigator_instance_id: str
    phase: MissionPhase
    execution_state: ExecutionState
    previous_phase: MissionPhase
    objective_id: str
    route_id: str
    active_segment_id: str
    goal_generation: int
    current_request_id: int
    map_revision: int
    required_unique_bus_stops: int
    serviced_stop_ids: tuple[str, ...]
    receipts: tuple[ServiceReceipt, ...]
    known_bus_stop_ids: tuple[str, ...]
    known_parking_target_id: str
    parking_prior_state: ParkingPriorState
    attempt_tainted: bool
    reason: str


@dataclass(frozen=True, slots=True)
class MissionTransition:
    accepted: bool
    snapshot: MissionSnapshot
    commands: tuple[MissionCommand, ...] = ()
    reason: str = ""


@dataclass(frozen=True, slots=True)
class MissionEvent:
    kind: MissionEventKind
    run_id: str
    map_instance_id: str
    navigator_instance_id: str
    request_id: int = 0
    goal_generation: int = 0
    map_revision: int = 0
    route_id: str = ""
    segment_id: str = ""
    target_id: str = ""
    target_kind: TargetKind = TargetKind.BUS_STOP
    observation_id: str = ""
    plan_status: PlanStatus = PlanStatus.INVALID_REQUEST
    validation_status: ValidationStatus = ValidationStatus.INVALID
    observation_outcome: ObservationOutcome = ObservationOutcome.INCONCLUSIVE
    known_bus_stop_ids: tuple[str, ...] = ()
    known_parking_target_id: str = ""
    report_sequence: int = 0
    reached_objective: bool = False
    receipt: ServiceReceipt | None = None
    readiness_ok: bool = False
    parking_prior_valid: bool | None = None
    reason: str = ""


class MissionMachine:
    """Deterministic mission FSM with no ROS or planner implementation logic."""

    def __init__(self) -> None:
        self._clear_all()

    def start(self, run: RunContext) -> MissionTransition:
        self._clear_all()
        self._run_id = run.run_id
        self._map_instance_id = run.map_instance_id
        self._navigator_instance_id = run.navigator_instance_id
        self._required_unique_bus_stops = run.required_unique_bus_stops
        self._parking_prior_enabled = bool(run.parking_prior_enabled)
        self._parking_prior_valid = bool(run.parking_prior_valid)
        self._parking_prior_observation_id = run.parking_prior_observation_id
        self._known_bus_stop_ids = set(run.known_bus_stop_ids)
        self._known_parking_target_id = run.known_parking_target_id
        self._map_revision = run.map_revision
        self._phase = MissionPhase.BOARDING
        self._execution_state = ExecutionState.HOLDING
        self._reason = "mission_started"
        commands = self._request_boarding_plan("mission_started")
        return self._transition(True, commands, "mission_started")

    def handle(self, event: MissionEvent) -> MissionTransition:
        if self._phase == MissionPhase.IDLE:
            if event.kind == MissionEventKind.RESET:
                return self._handle_reset(event)
            return self._transition(False, (), "mission_not_started")

        if not self._event_identity_current(event):
            return self._transition(False, (), "stale_identity_ignored")

        if event.kind == MissionEventKind.MAP_UPDATED:
            return self._handle_map_updated(event)
        if event.kind == MissionEventKind.PLAN_RESULT:
            return self._handle_plan_result(event)
        if event.kind == MissionEventKind.SEGMENT_VALIDATION_RESULT:
            return self._handle_validation_result(event)
        if event.kind == MissionEventKind.GOAL_ACCEPTED:
            return self._handle_goal_accepted(event)
        if event.kind == MissionEventKind.EXECUTION_COMPLETED:
            return self._handle_execution_completed(event)
        if event.kind == MissionEventKind.OBSERVATION_RESULT:
            return self._handle_observation_result(event)
        if event.kind == MissionEventKind.SERVICE_COMPLETED:
            return self._handle_service_completed(event)
        if event.kind == MissionEventKind.PARKING_STOP_VERIFIED:
            return self._handle_parking_verified(event)
        if event.kind == MissionEventKind.PAUSE:
            return self._pause(event.reason or "pause_requested")
        if event.kind == MissionEventKind.RESUME:
            return self._resume(event)
        if event.kind == MissionEventKind.ABORT:
            return self._abort(event.reason or "abort_requested")
        if event.kind == MissionEventKind.RESET:
            return self._handle_reset(event)
        if event.kind == MissionEventKind.NO_PROGRESS:
            return self._pause(event.reason or "no_useful_progress")
        if event.kind == MissionEventKind.TAINT_ATTEMPT:
            self._attempt_tainted = True
            self._reason = event.reason or "attempt_tainted"
            return self._transition(True, (), self._reason)
        return self._transition(False, (), "unsupported_event")

    def state(self) -> MissionSnapshot:
        return MissionSnapshot(
            run_id=self._run_id,
            map_instance_id=self._map_instance_id,
            navigator_instance_id=self._navigator_instance_id,
            phase=self._phase,
            execution_state=self._execution_state,
            previous_phase=self._previous_phase,
            objective_id=self._objective_id,
            route_id=self._route_id,
            active_segment_id=self._active_segment_id,
            goal_generation=self._goal_generation,
            current_request_id=self._current_request_id,
            map_revision=self._map_revision,
            required_unique_bus_stops=self._required_unique_bus_stops,
            serviced_stop_ids=tuple(r.target_id for r in self._receipts),
            receipts=tuple(self._receipts),
            known_bus_stop_ids=tuple(sorted(self._known_bus_stop_ids)),
            known_parking_target_id=self._known_parking_target_id,
            parking_prior_state=self._parking_prior_state,
            attempt_tainted=self._attempt_tainted,
            reason=self._reason,
        )

    def _clear_all(self) -> None:
        self._run_id = ""
        self._map_instance_id = ""
        self._navigator_instance_id = ""
        self._required_unique_bus_stops = 2
        self._parking_prior_enabled = True
        self._parking_prior_valid = True
        self._parking_prior_observation_id = ""
        self._parking_prior_state = ParkingPriorState.NOT_CHECKED
        self._known_bus_stop_ids: set[str] = set()
        self._known_parking_target_id = ""
        self._receipts: list[ServiceReceipt] = []
        self._phase = MissionPhase.IDLE
        self._previous_phase = MissionPhase.IDLE
        self._execution_state = ExecutionState.HOLDING
        self._objective_id = ""
        self._route_id = ""
        self._active_segment_id = ""
        self._goal_generation = 0
        self._request_counter = 0
        self._current_request_id = 0
        self._map_revision = 0
        self._attempt_tainted = False
        self._handled_execution_reports: set[tuple[str, int]] = set()
        self._completed_segment_ids: set[str] = set()
        self._reason = "idle"

    def _event_identity_current(self, event: MissionEvent) -> bool:
        return (
            event.run_id == self._run_id
            and event.map_instance_id == self._map_instance_id
            and event.navigator_instance_id == self._navigator_instance_id
        )

    def _new_request(self) -> int:
        self._request_counter += 1
        self._current_request_id = self._request_counter
        return self._current_request_id

    def _request_plan(
        self,
        goal_kind: PlanGoalKind,
        reason: str,
        *,
        target_id: str = "",
        observation_id: str = "",
    ) -> tuple[MissionCommand, ...]:
        request_id = self._new_request()
        self._execution_state = ExecutionState.REQUESTING_PLAN
        self._route_id = ""
        self._active_segment_id = ""
        self._reason = reason
        return (
            MissionCommand(
                kind=MissionCommandKind.REQUEST_PLAN,
                request_id=request_id,
                goal_kind=goal_kind,
                target_id=target_id,
                observation_id=observation_id,
                reason=reason,
            ),
        )

    def _request_boarding_plan(self, reason: str) -> tuple[MissionCommand, ...]:
        serviced = {r.target_id for r in self._receipts}
        candidates = sorted(self._known_bus_stop_ids - serviced)
        self._phase = MissionPhase.BOARDING
        if candidates:
            self._objective_id = candidates[0]
            return self._request_plan(
                PlanGoalKind.TARGET,
                reason,
                target_id=self._objective_id,
            )
        self._objective_id = ""
        return self._request_plan(PlanGoalKind.EXPLORE_STOPS, reason)

    def _request_parking(self, reason: str) -> tuple[MissionCommand, ...]:
        if self._known_parking_target_id:
            self._phase = MissionPhase.PARKING_APPROACH
            self._objective_id = self._known_parking_target_id
            return self._request_plan(
                PlanGoalKind.TARGET,
                reason,
                target_id=self._known_parking_target_id,
            )
        if not self._parking_prior_enabled or not self._parking_prior_valid:
            if self._parking_prior_enabled and not self._parking_prior_valid:
                self._parking_prior_state = ParkingPriorState.INVALID
            self._phase = MissionPhase.PARKING_SEARCH
            self._objective_id = ""
            return self._request_plan(PlanGoalKind.EXPLORE_PARKING, reason)
        if self._parking_prior_state == ParkingPriorState.NOT_CHECKED:
            self._phase = MissionPhase.PARKING_PRIOR
            self._objective_id = self._parking_prior_observation_id
            return self._request_plan(
                PlanGoalKind.CHECK_POI,
                reason,
                observation_id=self._parking_prior_observation_id,
            )
        self._phase = MissionPhase.PARKING_SEARCH
        self._objective_id = ""
        return self._request_plan(PlanGoalKind.EXPLORE_PARKING, reason)

    def _request_current_objective(self, reason: str) -> tuple[MissionCommand, ...]:
        if self._phase == MissionPhase.BOARDING:
            if self._objective_id:
                return self._request_plan(
                    PlanGoalKind.TARGET, reason, target_id=self._objective_id
                )
            return self._request_boarding_plan(reason)
        if self._phase == MissionPhase.PARKING_APPROACH and self._known_parking_target_id:
            return self._request_plan(
                PlanGoalKind.TARGET,
                reason,
                target_id=self._known_parking_target_id,
            )
        if self._phase == MissionPhase.PARKING_PRIOR:
            return self._request_plan(
                PlanGoalKind.CHECK_POI,
                reason,
                observation_id=self._parking_prior_observation_id,
            )
        if self._phase == MissionPhase.PARKING_SEARCH:
            return self._request_plan(PlanGoalKind.EXPLORE_PARKING, reason)
        return ()

    def _handle_map_updated(self, event: MissionEvent) -> MissionTransition:
        self._map_revision = max(self._map_revision, event.map_revision)
        self._known_bus_stop_ids.update(event.known_bus_stop_ids)
        if event.known_parking_target_id:
            self._known_parking_target_id = event.known_parking_target_id
        if event.parking_prior_valid is not None:
            self._parking_prior_valid = bool(event.parking_prior_valid)
        # A heartbeat must not erase the fault that put the machine in PAUSED;
        # otherwise operators only see "map_updated" after a safety stop.
        if self._phase != MissionPhase.PAUSED:
            self._reason = event.reason or "map_updated"
        # Discovery changes facts, not service receipts and not an in-flight objective.
        return self._transition(True, (), self._reason)

    def _handle_plan_result(self, event: MissionEvent) -> MissionTransition:
        if event.request_id != self._current_request_id:
            return self._transition(False, (), "stale_plan_request_ignored")
        if event.plan_status == PlanStatus.STALE_SESSION:
            return self._transition(False, (), "stale_plan_session_ignored")
        self._map_revision = max(self._map_revision, event.map_revision)

        if event.plan_status == PlanStatus.ROUTE_READY:
            if not event.route_id or not event.segment_id:
                return self._pause("malformed_route_ready")
            self._route_id = event.route_id
            self._active_segment_id = event.segment_id
            self._execution_state = ExecutionState.VALIDATING_SEGMENT
            self._reason = "route_ready_validate_segment"
            return self._transition(
                True,
                (
                    MissionCommand(
                        MissionCommandKind.REQUEST_VALIDATE_SEGMENT,
                        route_id=self._route_id,
                        segment_id=self._active_segment_id,
                        reason=self._reason,
                    ),
                ),
                self._reason,
            )

        if event.plan_status == PlanStatus.NEED_OBSERVATION:
            if not event.observation_id:
                return self._pause("plan_needs_observation_without_id")
            self._route_id = event.route_id
            if event.segment_id:
                if not event.route_id:
                    return self._pause("plan_observation_prefix_without_route_id")
                self._active_segment_id = event.segment_id
                self._execution_state = ExecutionState.VALIDATING_SEGMENT
                self._reason = "observation_prefix_validate_segment"
                return self._transition(
                    True,
                    (
                        MissionCommand(
                            MissionCommandKind.REQUEST_VALIDATE_SEGMENT,
                            route_id=self._route_id,
                            segment_id=self._active_segment_id,
                            reason=self._reason,
                        ),
                    ),
                    self._reason,
                )
            self._execution_state = ExecutionState.OBSERVING
            self._reason = "plan_requires_observation"
            return self._transition(
                True,
                (
                    MissionCommand(
                        MissionCommandKind.REQUEST_OBSERVE,
                        route_id=event.route_id,
                        observation_id=event.observation_id,
                        reason=self._reason,
                    ),
                ),
                self._reason,
            )

        if event.plan_status == PlanStatus.ALREADY_AT_TERMINAL:
            # CHECK_POI/observation objectives still require perception; arriving
            # at the observation pose is not equivalent to a verified service stop.
            if self._phase == MissionPhase.PARKING_PRIOR or event.observation_id:
                observation_id = event.observation_id or self._parking_prior_observation_id
                if not observation_id:
                    return self._pause("terminal_observation_id_missing")
                self._execution_state = ExecutionState.OBSERVING
                self._reason = "already_at_observation_pose"
                return self._transition(
                    True,
                    (
                        MissionCommand(
                            MissionCommandKind.REQUEST_OBSERVE,
                            observation_id=observation_id,
                            reason=self._reason,
                        ),
                    ),
                    self._reason,
                )
            self._execution_state = ExecutionState.VERIFYING_STOP
            self._reason = "already_at_terminal_verify_stop"
            return self._transition(
                True,
                (
                    MissionCommand(
                        MissionCommandKind.VERIFY_STOP,
                        target_id=self._objective_id,
                        reason=self._reason,
                    ),
                ),
                self._reason,
            )

        if event.plan_status in (PlanStatus.NO_ROUTE, PlanStatus.NO_REACHABLE_FRONTIER):
            if self._phase == MissionPhase.PARKING_PRIOR:
                self._parking_prior_state = ParkingPriorState.UNREACHABLE_NOW
                commands = self._request_parking("parking_prior_unreachable")
                return self._transition(True, commands, "parking_prior_unreachable")
            if self._phase == MissionPhase.BOARDING:
                if event.plan_status == PlanStatus.NO_ROUTE and self._objective_id:
                    # Do not declare the mission impossible because one known target is
                    # presently unreachable; ask the planner to explore other stops.
                    self._objective_id = ""
                    commands = self._request_plan(
                        PlanGoalKind.EXPLORE_STOPS,
                        "known_stop_unreachable_explore_others",
                    )
                    return self._transition(True, commands, self._reason)
                return self._pause("boarding_frontiers_exhausted")
            if self._phase in (MissionPhase.PARKING_SEARCH, MissionPhase.PARKING_APPROACH):
                return self._pause("parking_frontiers_exhausted")

        if event.plan_status in (PlanStatus.NOT_READY, PlanStatus.INVALID_REQUEST):
            return self._pause("planner_not_ready_or_invalid")
        return self._pause("unhandled_plan_result")

    def _handle_validation_result(self, event: MissionEvent) -> MissionTransition:
        if event.route_id != self._route_id or event.segment_id != self._active_segment_id:
            return self._transition(False, (), "stale_segment_validation_ignored")
        if event.validation_status == ValidationStatus.STALE_SESSION:
            return self._transition(False, (), "stale_validation_session_ignored")
        if event.validation_status == ValidationStatus.VALID:
            self._goal_generation += 1
            self._execution_state = ExecutionState.SENDING_GOAL
            self._reason = "segment_valid_send_goal"
            return self._transition(
                True,
                (
                    MissionCommand(
                        MissionCommandKind.SEND_SEGMENT,
                        route_id=self._route_id,
                        segment_id=self._active_segment_id,
                        goal_generation=self._goal_generation,
                        reason=self._reason,
                    ),
                ),
                self._reason,
            )
        if event.validation_status == ValidationStatus.NEED_OBSERVATION:
            if not event.observation_id:
                return self._pause("validation_needs_observation_without_id")
            self._execution_state = ExecutionState.OBSERVING
            self._reason = "validation_requires_observation"
            return self._transition(
                True,
                (
                    MissionCommand(
                        MissionCommandKind.REQUEST_OBSERVE,
                        observation_id=event.observation_id,
                        reason=self._reason,
                    ),
                ),
                self._reason,
            )
        return self._pause("segment_validation_failed")

    def _handle_goal_accepted(self, event: MissionEvent) -> MissionTransition:
        if (
            event.goal_generation != self._goal_generation
            or event.route_id != self._route_id
            or event.segment_id != self._active_segment_id
        ):
            return self._transition(False, (), "stale_goal_acceptance_ignored")
        if self._execution_state != ExecutionState.SENDING_GOAL:
            return self._transition(False, (), "goal_acceptance_wrong_state_ignored")
        self._execution_state = ExecutionState.MOVING
        self._reason = "goal_active"
        return self._transition(True, (), self._reason)

    def _handle_execution_completed(self, event: MissionEvent) -> MissionTransition:
        if event.goal_generation != self._goal_generation:
            return self._transition(False, (), "old_generation_completion_ignored")
        if event.route_id != self._route_id or event.segment_id != self._active_segment_id:
            return self._transition(False, (), "stale_segment_completion_ignored")
        key = (event.navigator_instance_id, event.report_sequence)
        if event.report_sequence <= 0:
            return self._transition(False, (), "invalid_report_sequence")
        if key in self._handled_execution_reports or event.segment_id in self._completed_segment_ids:
            return self._transition(False, (), "duplicate_execution_completion_ignored")
        self._handled_execution_reports.add(key)
        self._completed_segment_ids.add(event.segment_id)
        self._execution_state = ExecutionState.HOLDING
        self._reason = "segment_completion_accepted"

        report = MissionCommand(
            MissionCommandKind.REPORT_EXECUTION,
            route_id=event.route_id,
            segment_id=event.segment_id,
            goal_generation=event.goal_generation,
            reason=self._reason,
        )
        if event.reached_objective:
            self._execution_state = ExecutionState.VERIFYING_STOP
            verify = MissionCommand(
                MissionCommandKind.VERIFY_STOP,
                target_id=self._objective_id,
                reason="objective_terminal_verify_stop",
            )
            return self._transition(True, (report, verify), "objective_terminal_verify_stop")
        commands = self._request_current_objective("segment_complete_continue_objective")
        return self._transition(True, (report,) + commands, self._reason)

    def _handle_observation_result(self, event: MissionEvent) -> MissionTransition:
        if event.observation_outcome == ObservationOutcome.STALE_SESSION:
            return self._transition(False, (), "stale_observation_ignored")
        self._map_revision = max(self._map_revision, event.map_revision)

        if event.observation_outcome == ObservationOutcome.CONFLICT:
            return self._pause("observation_conflict")
        if event.observation_outcome == ObservationOutcome.NOT_IN_ZONE:
            return self._pause("observation_not_in_zone")

        if event.observation_outcome == ObservationOutcome.TARGET_FOUND:
            if event.target_kind == TargetKind.PARKING:
                if event.target_id:
                    self._known_parking_target_id = event.target_id
                if self._phase == MissionPhase.PARKING_PRIOR:
                    self._parking_prior_state = ParkingPriorState.FOUND
                if self._phase in (
                    MissionPhase.PARKING_PRIOR,
                    MissionPhase.PARKING_SEARCH,
                    MissionPhase.PARKING_APPROACH,
                ):
                    commands = self._request_parking("parking_target_found")
                    return self._transition(True, commands, self._reason)
                # Parking discovered while boarding is remembered only.
                self._reason = "parking_remembered_during_boarding"
                commands = self._request_current_objective(self._reason)
                return self._transition(True, commands, self._reason)
            if event.target_id:
                self._known_bus_stop_ids.add(event.target_id)
            if self._phase == MissionPhase.BOARDING:
                if event.target_id and event.target_id not in {r.target_id for r in self._receipts}:
                    self._objective_id = event.target_id
                commands = self._request_boarding_plan("bus_target_found")
                return self._transition(True, commands, self._reason)

        if event.observation_outcome == ObservationOutcome.CLEAR_ABSENT:
            if self._phase == MissionPhase.PARKING_PRIOR:
                self._parking_prior_state = ParkingPriorState.CLEAR_ABSENT
                commands = self._request_parking("parking_prior_clear_absent")
                return self._transition(True, commands, self._reason)
            commands = self._request_current_objective("observation_clear_continue")
            return self._transition(True, commands, self._reason)

        if event.observation_outcome == ObservationOutcome.INCONCLUSIVE:
            if self._phase == MissionPhase.PARKING_PRIOR:
                self._parking_prior_state = ParkingPriorState.INCONCLUSIVE
                commands = self._request_parking("parking_prior_inconclusive")
                return self._transition(True, commands, self._reason)
            return self._pause("observation_inconclusive")

        if event.observation_outcome == ObservationOutcome.APPROACH_RESOLVED:
            commands = self._request_current_objective("approach_resolved_replan")
            return self._transition(True, commands, self._reason)

        return self._pause("unhandled_observation_result")

    def _handle_service_completed(self, event: MissionEvent) -> MissionTransition:
        receipt = event.receipt
        if receipt is None:
            return self._transition(False, (), "service_completion_without_receipt")
        if receipt.run_id != self._run_id:
            return self._transition(False, (), "stale_receipt_run_ignored")
        if not receipt.target_id:
            return self._transition(False, (), "receipt_target_missing")
        if any(r.target_id == receipt.target_id for r in self._receipts):
            return self._transition(False, (), "duplicate_service_receipt_ignored")
        if receipt.target_id not in self._known_bus_stop_ids:
            return self._transition(False, (), "receipt_target_not_known_bus_stop")
        self._receipts.append(receipt)
        self._execution_state = ExecutionState.HOLDING
        self._reason = "service_receipt_appended"
        if len(self._receipts) >= self._required_unique_bus_stops:
            commands = self._request_parking("boarding_requirement_complete")
            return self._transition(True, commands, self._reason)
        commands = self._request_boarding_plan("service_complete_continue_boarding")
        return self._transition(True, commands, self._reason)

    def _handle_parking_verified(self, event: MissionEvent) -> MissionTransition:
        if len(self._receipts) < self._required_unique_bus_stops:
            return self._transition(False, (), "parking_before_boarding_complete_rejected")
        if not self._known_parking_target_id or event.target_id != self._known_parking_target_id:
            return self._transition(False, (), "wrong_parking_target_rejected")
        if self._phase != MissionPhase.PARKING_APPROACH:
            return self._transition(False, (), "parking_stop_wrong_phase_rejected")
        self._phase = MissionPhase.DONE
        self._execution_state = ExecutionState.HOLDING
        self._objective_id = self._known_parking_target_id
        self._route_id = ""
        self._active_segment_id = ""
        self._reason = "mission_complete"
        return self._transition(
            True,
            (
                MissionCommand(MissionCommandKind.DISABLE_MOTION, reason=self._reason),
                MissionCommand(MissionCommandKind.COMPLETE, reason=self._reason),
            ),
            self._reason,
        )

    def _pause(self, reason: str) -> MissionTransition:
        if self._phase not in (MissionPhase.PAUSED, MissionPhase.DONE, MissionPhase.ABORTED):
            self._previous_phase = self._phase
        self._phase = MissionPhase.PAUSED
        self._execution_state = ExecutionState.CANCELING
        self._reason = reason
        return self._transition(
            True,
            (
                MissionCommand(MissionCommandKind.CANCEL_AND_HOLD, reason=reason),
                MissionCommand(MissionCommandKind.DISABLE_MOTION, reason=reason),
            ),
            reason,
        )

    def _resume(self, event: MissionEvent) -> MissionTransition:
        if self._phase != MissionPhase.PAUSED:
            return self._transition(False, (), "resume_only_valid_from_paused")
        if not event.readiness_ok:
            return self._transition(False, (), "resume_readiness_failed")
        if self._previous_phase in (MissionPhase.IDLE, MissionPhase.DONE, MissionPhase.ABORTED):
            return self._transition(False, (), "resume_previous_phase_invalid")
        self._phase = self._previous_phase
        self._execution_state = ExecutionState.HOLDING
        commands = self._request_current_objective("resume_replan")
        return self._transition(True, commands, self._reason)

    def _abort(self, reason: str) -> MissionTransition:
        self._phase = MissionPhase.ABORTED
        self._execution_state = ExecutionState.HOLDING
        self._reason = reason
        return self._transition(
            True,
            (
                MissionCommand(MissionCommandKind.CANCEL_AND_HOLD, reason=reason),
                MissionCommand(MissionCommandKind.DISABLE_MOTION, reason=reason),
            ),
            reason,
        )

    def _handle_reset(self, event: MissionEvent) -> MissionTransition:
        self._clear_all()
        self._reason = event.reason or "mission_reset"
        return self._transition(
            True,
            (
                MissionCommand(MissionCommandKind.DISABLE_MOTION, reason=self._reason),
                MissionCommand(MissionCommandKind.RESET_MAP, reason=self._reason),
            ),
            self._reason,
        )

    def _transition(
        self,
        accepted: bool,
        commands: Iterable[MissionCommand],
        reason: str,
    ) -> MissionTransition:
        return MissionTransition(
            accepted=accepted,
            snapshot=self.state(),
            commands=tuple(commands),
            reason=reason,
        )
