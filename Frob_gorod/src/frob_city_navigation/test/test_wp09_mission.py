from frob_city_navigation.models import Pose2D
from frob_city_navigation.mission import (
    ExecutionState,
    MissionCommandKind,
    MissionEvent,
    MissionEventKind,
    MissionMachine,
    MissionPhase,
    ObservationOutcome,
    ParkingPriorState,
    PlanGoalKind,
    PlanStatus,
    RunContext,
    ServiceReceipt,
    TargetKind,
    ValidationStatus,
)


def ctx(**changes):
    values = dict(
        run_id='RUN',
        map_instance_id='MAP',
        navigator_instance_id='NAV',
        required_unique_bus_stops=2,
        parking_prior_enabled=True,
        parking_prior_valid=True,
        parking_prior_observation_id='PRIOR_VIEW',
    )
    values.update(changes)
    return RunContext(**values)


def ev(kind, **changes):
    values = dict(
        kind=kind,
        run_id='RUN',
        map_instance_id='MAP',
        navigator_instance_id='NAV',
    )
    values.update(changes)
    return MissionEvent(**values)


def receipt(target, *, idx=0):
    return ServiceReceipt(
        run_id='RUN',
        target_id=target,
        stop_entry_s=10.0 + idx * 10,
        stop_exit_s=12.2 + idx * 10,
        dwell_duration_s=2.2,
        final_pose=Pose2D(float(idx), 0.0, 0.0),
        max_measured_speed_mps=0.005,
        map_revision=idx + 1,
    )


def only_command(t):
    assert len(t.commands) == 1
    return t.commands[0]


def test_start_enters_boarding_and_explores_when_no_known_stop():
    m = MissionMachine()
    t = m.start(ctx())
    assert t.snapshot.phase is MissionPhase.BOARDING
    assert t.snapshot.execution_state is ExecutionState.REQUESTING_PLAN
    c = only_command(t)
    assert c.kind is MissionCommandKind.REQUEST_PLAN
    assert c.goal_kind is PlanGoalKind.EXPLORE_STOPS
    assert c.request_id == 1


def test_start_prefers_known_unserviced_stop_but_does_not_service_it():
    m = MissionMachine()
    t = m.start(ctx(known_bus_stop_ids=('BUS:B:F', 'BUS:A:F')))
    assert t.snapshot.serviced_stop_ids == ()
    c = only_command(t)
    assert c.goal_kind is PlanGoalKind.TARGET
    assert c.target_id == 'BUS:A:F'


def test_two_discovered_stops_still_zero_receipts():
    m = MissionMachine(); m.start(ctx())
    m.handle(ev(
        MissionEventKind.MAP_UPDATED,
        known_bus_stop_ids=('BUS:A:F', 'BUS:B:F'),
        map_revision=4,
    ))
    s = m.state()
    assert s.known_bus_stop_ids == ('BUS:A:F', 'BUS:B:F')
    assert s.serviced_stop_ids == ()
    assert s.phase is MissionPhase.BOARDING


def test_early_parking_discovery_is_remembered_without_ending_boarding():
    m = MissionMachine(); m.start(ctx())
    t = m.handle(ev(
        MissionEventKind.MAP_UPDATED,
        known_parking_target_id='PARK:P:F',
        map_revision=2,
    ))
    assert t.snapshot.phase is MissionPhase.BOARDING
    assert t.snapshot.known_parking_target_id == 'PARK:P:F'
    assert t.snapshot.serviced_stop_ids == ()


def test_stale_identity_is_ignored():
    m = MissionMachine(); m.start(ctx())
    before = m.state()
    t = m.handle(MissionEvent(
        kind=MissionEventKind.MAP_UPDATED,
        run_id='OLD', map_instance_id='MAP', navigator_instance_id='NAV',
        known_bus_stop_ids=('BUS:A:F',),
    ))
    assert not t.accepted
    assert m.state() == before


def test_stale_plan_request_is_ignored():
    m = MissionMachine(); start = m.start(ctx())
    current = start.snapshot.current_request_id
    t = m.handle(ev(
        MissionEventKind.PLAN_RESULT,
        request_id=current - 1,
        plan_status=PlanStatus.ROUTE_READY,
        route_id='R', segment_id='S',
    ))
    assert not t.accepted
    assert m.state().route_id == ''


def test_route_ready_validate_send_goal_and_goal_acceptance_trace():
    m = MissionMachine(); start = m.start(ctx())
    req = start.snapshot.current_request_id
    t = m.handle(ev(
        MissionEventKind.PLAN_RESULT,
        request_id=req,
        plan_status=PlanStatus.ROUTE_READY,
        route_id='R1', segment_id='S1',
    ))
    assert t.snapshot.execution_state is ExecutionState.VALIDATING_SEGMENT
    assert only_command(t).kind is MissionCommandKind.REQUEST_VALIDATE_SEGMENT

    t = m.handle(ev(
        MissionEventKind.SEGMENT_VALIDATION_RESULT,
        route_id='R1', segment_id='S1',
        validation_status=ValidationStatus.VALID,
    ))
    assert t.snapshot.execution_state is ExecutionState.SENDING_GOAL
    cmd = only_command(t)
    assert cmd.kind is MissionCommandKind.SEND_SEGMENT
    assert cmd.goal_generation == 1

    t = m.handle(ev(
        MissionEventKind.GOAL_ACCEPTED,
        route_id='R1', segment_id='S1', goal_generation=1,
    ))
    assert t.snapshot.execution_state is ExecutionState.MOVING


def test_old_generation_success_does_not_advance():
    m = MissionMachine(); s = m.start(ctx(known_bus_stop_ids=('BUS:A:F',)))
    req = s.snapshot.current_request_id
    m.handle(ev(MissionEventKind.PLAN_RESULT, request_id=req,
                plan_status=PlanStatus.ROUTE_READY, route_id='R', segment_id='S'))
    m.handle(ev(MissionEventKind.SEGMENT_VALIDATION_RESULT, route_id='R', segment_id='S',
                validation_status=ValidationStatus.VALID))
    before = m.state()
    t = m.handle(ev(MissionEventKind.EXECUTION_COMPLETED,
                    route_id='R', segment_id='S', goal_generation=0,
                    report_sequence=1, reached_objective=True))
    assert not t.accepted
    assert m.state() == before


def test_duplicate_execution_report_does_not_advance_twice():
    m = MissionMachine(); s = m.start(ctx(known_bus_stop_ids=('BUS:A:F',)))
    req = s.snapshot.current_request_id
    m.handle(ev(MissionEventKind.PLAN_RESULT, request_id=req,
                plan_status=PlanStatus.ROUTE_READY, route_id='R', segment_id='S'))
    m.handle(ev(MissionEventKind.SEGMENT_VALIDATION_RESULT, route_id='R', segment_id='S',
                validation_status=ValidationStatus.VALID))
    m.handle(ev(MissionEventKind.GOAL_ACCEPTED, route_id='R', segment_id='S', goal_generation=1))
    first = m.handle(ev(MissionEventKind.EXECUTION_COMPLETED,
                        route_id='R', segment_id='S', goal_generation=1,
                        report_sequence=10, reached_objective=True))
    assert first.accepted
    before = m.state()
    second = m.handle(ev(MissionEventKind.EXECUTION_COMPLETED,
                         route_id='R', segment_id='S', goal_generation=1,
                         report_sequence=10, reached_objective=True))
    assert not second.accepted
    assert m.state() == before


def test_same_stop_receipt_twice_counts_once():
    m = MissionMachine(); m.start(ctx(known_bus_stop_ids=('BUS:A:F',)))
    first = m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:A:F')))
    assert first.accepted
    assert m.state().serviced_stop_ids == ('BUS:A:F',)
    second = m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:A:F', idx=1)))
    assert not second.accepted
    assert m.state().serviced_stop_ids == ('BUS:A:F',)


def test_two_distinct_services_then_known_parking_then_done():
    m = MissionMachine()
    m.start(ctx(
        known_bus_stop_ids=('BUS:A:F', 'BUS:B:F'),
        known_parking_target_id='PARK:P:F',
    ))
    t = m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:A:F')))
    assert t.snapshot.phase is MissionPhase.BOARDING
    assert t.snapshot.serviced_stop_ids == ('BUS:A:F',)

    t = m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:B:F', idx=1)))
    assert t.snapshot.phase is MissionPhase.PARKING_APPROACH
    c = only_command(t)
    assert c.goal_kind is PlanGoalKind.TARGET
    assert c.target_id == 'PARK:P:F'

    t = m.handle(ev(MissionEventKind.PARKING_STOP_VERIFIED, target_id='PARK:P:F'))
    assert t.snapshot.phase is MissionPhase.DONE
    assert t.snapshot.execution_state is ExecutionState.HOLDING
    assert [c.kind for c in t.commands] == [
        MissionCommandKind.DISABLE_MOTION,
        MissionCommandKind.COMPLETE,
    ]


def test_boarding_complete_with_no_parking_requests_prior_check():
    m = MissionMachine(); m.start(ctx(known_bus_stop_ids=('BUS:A:F', 'BUS:B:F')))
    m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:A:F')))
    t = m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:B:F', idx=1)))
    assert t.snapshot.phase is MissionPhase.PARKING_PRIOR
    c = only_command(t)
    assert c.goal_kind is PlanGoalKind.CHECK_POI
    assert c.observation_id == 'PRIOR_VIEW'


def test_invalid_prior_skips_directly_to_parking_search():
    m = MissionMachine(); m.start(ctx(
        parking_prior_valid=False,
        known_bus_stop_ids=('BUS:A:F', 'BUS:B:F'),
    ))
    m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:A:F')))
    t = m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:B:F', idx=1)))
    assert t.snapshot.phase is MissionPhase.PARKING_SEARCH
    assert t.snapshot.parking_prior_state is ParkingPriorState.INVALID
    assert only_command(t).goal_kind is PlanGoalKind.EXPLORE_PARKING


def _reach_parking_prior(m):
    m.start(ctx(known_bus_stop_ids=('BUS:A:F', 'BUS:B:F')))
    m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:A:F')))
    return m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:B:F', idx=1)))


def test_prior_clear_absent_goes_to_search():
    m = MissionMachine(); _reach_parking_prior(m)
    t = m.handle(ev(MissionEventKind.OBSERVATION_RESULT,
                    observation_outcome=ObservationOutcome.CLEAR_ABSENT))
    assert t.snapshot.parking_prior_state is ParkingPriorState.CLEAR_ABSENT
    assert t.snapshot.phase is MissionPhase.PARKING_SEARCH
    assert only_command(t).goal_kind is PlanGoalKind.EXPLORE_PARKING


def test_prior_inconclusive_goes_to_search_not_found():
    m = MissionMachine(); _reach_parking_prior(m)
    t = m.handle(ev(MissionEventKind.OBSERVATION_RESULT,
                    observation_outcome=ObservationOutcome.INCONCLUSIVE))
    assert t.snapshot.parking_prior_state is ParkingPriorState.INCONCLUSIVE
    assert t.snapshot.phase is MissionPhase.PARKING_SEARCH


def test_prior_unreachable_goes_to_search():
    m = MissionMachine(); t = _reach_parking_prior(m)
    req = t.snapshot.current_request_id
    t = m.handle(ev(MissionEventKind.PLAN_RESULT,
                    request_id=req, plan_status=PlanStatus.NO_ROUTE))
    assert t.snapshot.parking_prior_state is ParkingPriorState.UNREACHABLE_NOW
    assert t.snapshot.phase is MissionPhase.PARKING_SEARCH


def test_prior_found_goes_to_parking_approach():
    m = MissionMachine(); _reach_parking_prior(m)
    t = m.handle(ev(MissionEventKind.OBSERVATION_RESULT,
                    observation_outcome=ObservationOutcome.TARGET_FOUND,
                    target_kind=TargetKind.PARKING,
                    target_id='PARK:P:F'))
    assert t.snapshot.parking_prior_state is ParkingPriorState.FOUND
    assert t.snapshot.phase is MissionPhase.PARKING_APPROACH
    assert only_command(t).target_id == 'PARK:P:F'


def test_parking_stop_before_required_receipts_is_rejected():
    m = MissionMachine(); m.start(ctx(known_parking_target_id='PARK:P:F'))
    t = m.handle(ev(MissionEventKind.PARKING_STOP_VERIFIED, target_id='PARK:P:F'))
    assert not t.accepted
    assert t.snapshot.phase is MissionPhase.BOARDING


def test_pause_resume_requires_readiness_and_replans():
    m = MissionMachine(); m.start(ctx(known_bus_stop_ids=('BUS:A:F',)))
    t = m.handle(ev(MissionEventKind.PAUSE, reason='temporary_block'))
    assert t.snapshot.phase is MissionPhase.PAUSED
    assert t.snapshot.execution_state is ExecutionState.CANCELING
    assert MissionCommandKind.DISABLE_MOTION in [c.kind for c in t.commands]

    failed = m.handle(ev(MissionEventKind.RESUME, readiness_ok=False))
    assert not failed.accepted
    assert m.state().phase is MissionPhase.PAUSED

    resumed = m.handle(ev(MissionEventKind.RESUME, readiness_ok=True))
    assert resumed.accepted
    assert resumed.snapshot.phase is MissionPhase.BOARDING
    assert resumed.snapshot.execution_state is ExecutionState.REQUESTING_PLAN


def test_map_heartbeat_does_not_erase_pause_reason():
    m = MissionMachine(); m.start(ctx(known_bus_stop_ids=('BUS:A:F',)))
    m.handle(ev(MissionEventKind.PAUSE, reason='guard_activation_timeout'))

    updated = m.handle(ev(
        MissionEventKind.MAP_UPDATED,
        map_revision=2,
        reason='map_updated',
    ))

    assert updated.snapshot.phase is MissionPhase.PAUSED
    assert updated.snapshot.reason == 'guard_activation_timeout'


def test_abort_is_persistent_held_state():
    m = MissionMachine(); m.start(ctx())
    t = m.handle(ev(MissionEventKind.ABORT, reason='operator_abort'))
    assert t.snapshot.phase is MissionPhase.ABORTED
    assert t.snapshot.execution_state is ExecutionState.HOLDING
    assert MissionCommandKind.DISABLE_MOTION in [c.kind for c in t.commands]


def test_no_progress_pauses_instead_of_false_done():
    m = MissionMachine(); m.start(ctx())
    t = m.handle(ev(MissionEventKind.NO_PROGRESS, reason='frontiers_exhausted'))
    assert t.snapshot.phase is MissionPhase.PAUSED
    assert t.snapshot.phase is not MissionPhase.DONE


def test_attempt_taint_never_clears_during_attempt():
    m = MissionMachine(); m.start(ctx())
    m.handle(ev(MissionEventKind.TAINT_ATTEMPT, reason='possible_rule_violation'))
    assert m.state().attempt_tainted
    m.handle(ev(MissionEventKind.MAP_UPDATED, map_revision=10))
    assert m.state().attempt_tainted


def test_reset_clears_receipts_and_returns_idle():
    m = MissionMachine(); m.start(ctx(known_bus_stop_ids=('BUS:A:F',)))
    m.handle(ev(MissionEventKind.SERVICE_COMPLETED, receipt=receipt('BUS:A:F')))
    t = m.handle(ev(MissionEventKind.RESET, reason='new_attempt'))
    assert t.snapshot.phase is MissionPhase.IDLE
    assert t.snapshot.serviced_stop_ids == ()
    assert [c.kind for c in t.commands] == [
        MissionCommandKind.DISABLE_MOTION,
        MissionCommandKind.RESET_MAP,
    ]


def test_parking_prior_already_at_terminal_still_requests_observation():
    m = MissionMachine(); t = _reach_parking_prior(m)
    req = t.snapshot.current_request_id
    t = m.handle(ev(
        MissionEventKind.PLAN_RESULT,
        request_id=req,
        plan_status=PlanStatus.ALREADY_AT_TERMINAL,
        observation_id='PRIOR_VIEW',
    ))
    assert t.snapshot.phase is MissionPhase.PARKING_PRIOR
    assert t.snapshot.execution_state is ExecutionState.OBSERVING
    c = only_command(t)
    assert c.kind is MissionCommandKind.REQUEST_OBSERVE
    assert c.observation_id == 'PRIOR_VIEW'


def test_plan_need_observation_never_sends_motion():
    m = MissionMachine(); start = m.start(ctx())
    t = m.handle(ev(
        MissionEventKind.PLAN_RESULT,
        request_id=start.snapshot.current_request_id,
        plan_status=PlanStatus.NEED_OBSERVATION,
        route_id='R1', observation_id='OBS_GATE',
    ))
    assert t.snapshot.execution_state is ExecutionState.OBSERVING
    assert [c.kind for c in t.commands] == [MissionCommandKind.REQUEST_OBSERVE]


def test_plan_need_observation_executes_available_prefix_before_observing():
    m = MissionMachine(); start = m.start(ctx())
    t = m.handle(ev(
        MissionEventKind.PLAN_RESULT,
        request_id=start.snapshot.current_request_id,
        plan_status=PlanStatus.NEED_OBSERVATION,
        route_id='R1', segment_id='SEG_TO_GATE', observation_id='OBS_GATE',
    ))
    assert t.snapshot.execution_state is ExecutionState.VALIDATING_SEGMENT
    command = only_command(t)
    assert command.kind is MissionCommandKind.REQUEST_VALIDATE_SEGMENT
    assert command.route_id == 'R1'
    assert command.segment_id == 'SEG_TO_GATE'
