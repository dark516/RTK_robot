from frob_city_navigation.models import Pose2D
from frob_city_navigation.mission import (
    MissionCommandKind,
    MissionEvent,
    MissionEventKind,
    MissionMachine,
    MissionPhase,
    RunContext,
    ServiceReceipt,
)
from frob_city_navigation.service_monitor import ServiceMonitor
from fakes.wp09_fakes import FakeClock, FakeGuard, FakeMapPlannerClient, FakeOdometryStream


def test_fake_clock_ros_and_monotonic_advance_independently():
    c = FakeClock(ros_s=10.0, monotonic_s=20.0)
    c.advance_ros(1.5)
    assert c.ros_s == 11.5
    assert c.monotonic_s == 20.0
    c.advance_monotonic(2.0)
    assert c.ros_s == 11.5
    assert c.monotonic_s == 22.0


def test_fake_map_planner_preserves_request_identity():
    m = MissionMachine()
    t = m.start(RunContext('RUN', 'MAP', 'NAV'))
    cmd = t.commands[0]
    fake = FakeMapPlannerClient('RUN', 'MAP', 'NAV')
    event = fake.route_ready(cmd)
    assert event.request_id == cmd.request_id
    t = m.handle(event)
    assert t.accepted
    assert t.commands[0].kind is MissionCommandKind.REQUEST_VALIDATE_SEGMENT


def test_full_two_stop_parking_trace_with_fake_clock_guard_and_odom():
    machine = MissionMachine()
    machine.start(RunContext(
        run_id='RUN', map_instance_id='MAP', navigator_instance_id='NAV',
        known_bus_stop_ids=('BUS:A:F', 'BUS:B:F'),
        known_parking_target_id='PARK:P:F',
    ))

    clock = FakeClock(monotonic_s=100.0)
    guard = FakeGuard(zero_output=True, stopped=True)
    odom = FakeOdometryStream()

    for index, target in enumerate(('BUS:A:F', 'BUS:B:F')):
        monitor = ServiceMonitor()
        sample = odom.service_sample(
            guard, run_id='RUN', map_instance_id='MAP', target_id=target
        )
        monitor.update(sample, clock.monotonic_s)
        clock.advance_monotonic(2.2)
        progress = monitor.update(sample, clock.monotonic_s)
        assert progress.completed_now
        rec = ServiceReceipt(
            run_id='RUN', target_id=target,
            stop_entry_s=clock.monotonic_s - 2.2,
            stop_exit_s=clock.monotonic_s,
            dwell_duration_s=progress.dwell_s,
            final_pose=Pose2D(float(index), 0.0, 0.0),
            max_measured_speed_mps=progress.max_linear_speed_mps,
            map_revision=index + 1,
        )
        transition = machine.handle(MissionEvent(
            kind=MissionEventKind.SERVICE_COMPLETED,
            run_id='RUN', map_instance_id='MAP', navigator_instance_id='NAV',
            receipt=rec,
        ))
        assert transition.accepted
        clock.advance_monotonic(1.0)

    assert machine.state().phase is MissionPhase.PARKING_APPROACH
    done = machine.handle(MissionEvent(
        kind=MissionEventKind.PARKING_STOP_VERIFIED,
        run_id='RUN', map_instance_id='MAP', navigator_instance_id='NAV',
        target_id='PARK:P:F',
    ))
    assert done.snapshot.phase is MissionPhase.DONE
    assert done.snapshot.serviced_stop_ids == ('BUS:A:F', 'BUS:B:F')
