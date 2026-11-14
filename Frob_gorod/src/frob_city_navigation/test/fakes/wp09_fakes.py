from dataclasses import dataclass

from frob_city_navigation.mission import (
    MissionCommand,
    MissionCommandKind,
    MissionEvent,
    MissionEventKind,
    PlanStatus,
)
from frob_city_navigation.service_monitor import StopPredicateSample


class FakeClock:
    def __init__(self, *, ros_s: float = 0.0, monotonic_s: float = 0.0) -> None:
        self.ros_s = float(ros_s)
        self.monotonic_s = float(monotonic_s)

    def advance_ros(self, dt: float) -> None:
        self.ros_s += float(dt)

    def advance_monotonic(self, dt: float) -> None:
        self.monotonic_s += float(dt)

    def advance(self, dt: float) -> None:
        self.advance_ros(dt)
        self.advance_monotonic(dt)


@dataclass
class FakeGuard:
    zero_output: bool = True
    stopped: bool = True


@dataclass
class FakeOdometryStream:
    linear_speed_mps: float = 0.0
    angular_speed_radps: float = 0.0
    odometry_fresh: bool = True
    localization_fresh: bool = True

    def service_sample(
        self,
        guard: FakeGuard,
        *,
        run_id: str,
        map_instance_id: str,
        target_id: str,
    ) -> StopPredicateSample:
        return StopPredicateSample(
            run_id=run_id,
            map_instance_id=map_instance_id,
            target_id=target_id,
            target_confirmed=True,
            geometry_valid=True,
            correct_directed_approach=True,
            within_xy_tolerance=True,
            within_yaw_tolerance=True,
            front_before_terminal_limit=True,
            measured_linear_speed_mps=self.linear_speed_mps,
            measured_angular_speed_radps=self.angular_speed_radps,
            odometry_fresh=self.odometry_fresh,
            localization_fresh=self.localization_fresh,
            guard_zero_output=guard.zero_output,
            guard_measured_stopped=guard.stopped,
            movement_action_active=False,
            target_conflict=False,
            identity_current=True,
        )


class FakeMapPlannerClient:
    """Small deterministic fake that converts REQUEST_PLAN commands into events."""

    def __init__(self, run_id: str, map_instance_id: str, navigator_instance_id: str) -> None:
        self.run_id = run_id
        self.map_instance_id = map_instance_id
        self.navigator_instance_id = navigator_instance_id
        self.requests: list[MissionCommand] = []
        self._counter = 0

    def route_ready(self, command: MissionCommand) -> MissionEvent:
        if command.kind is not MissionCommandKind.REQUEST_PLAN:
            raise ValueError('expected REQUEST_PLAN command')
        self.requests.append(command)
        self._counter += 1
        return MissionEvent(
            kind=MissionEventKind.PLAN_RESULT,
            run_id=self.run_id,
            map_instance_id=self.map_instance_id,
            navigator_instance_id=self.navigator_instance_id,
            request_id=command.request_id,
            plan_status=PlanStatus.ROUTE_READY,
            route_id=f'ROUTE_{self._counter}',
            segment_id=f'SEG_{self._counter}',
        )
