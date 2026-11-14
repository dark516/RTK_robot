"""ROS 2 mission/execution orchestrator for Frob City (WP12).

The navigator owns WHAT executes now, never route legality.  It consumes the
verified WP09 MissionMachine, calls map_planner for plan/validation/reporting,
executes explicit RouteSegment paths through Nav2 FollowPath, and publishes a
short-lived MotionPermit heartbeat only while the matching Nav2 goal is active.

Safety ordering is deliberate: every cancel/fault path disables MotionPermit
before requesting Nav2 cancellation.  Nav2 SUCCESS is only a transport result;
a segment is reported COMPLETED only after measured terminal pose + measured
stop + current guard/run/generation checks pass.
"""
from __future__ import annotations

from copy import deepcopy
import math
import threading
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from frob_city_interfaces.action import Observe
from frob_city_interfaces.msg import GuardState, MapState, MissionState, MotionPermit, RouteSegment
from frob_city_interfaces.srv import ControlMission, PlanRoute, ReportExecution, ResetMap, ValidateSegment

from .mission import (
    MissionCommand,
    MissionCommandKind,
    MissionEvent,
    MissionEventKind,
    MissionMachine,
    MissionPhase,
    ObservationOutcome,
    PlanStatus,
    RunContext,
    ServiceReceipt,
    TargetKind,
    ValidationStatus,
)
from .nav2_adapter import ExecutionToken, Nav2Adapter, Nav2Completion, Nav2Result
from .ros_execution_conversions import (
    build_motion_permit_msg,
    mission_state_to_msg,
    route_segment_to_nav_path,
)
from .service_monitor import (
    ServiceMode,
    ServiceMonitor,
    ServiceMonitorConfig,
    StopPredicateSample,
)
from .models import Pose2D


def _guard_denial_is_fatal(
    *,
    permit_enabled: bool,
    message_generation: int,
    active_generation: int,
    motion_allowed: bool,
    previously_allowed_generation: int,
) -> bool:
    """Distinguish guard activation handshake from loss of authorization.

    On a new goal generation the guard deliberately rejects the cached raw
    command until a command received after the permit arrives.  That initial
    fail-closed state is expected and must not cancel the just-accepted Nav2
    goal.  Once the guard has authorized the generation, any later denial is
    an immediate fault for the navigator.
    """
    return (
        permit_enabled
        and message_generation == active_generation
        and not motion_allowed
        and previously_allowed_generation == active_generation
    )


class NavigatorNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('navigator', parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._lock = threading.RLock()
        self._state_group = MutuallyExclusiveCallbackGroup()
        self._io_group = ReentrantCallbackGroup()

        self._navigator_instance_id = f'navigator-{uuid.uuid4().hex[:12]}'
        self._mission = MissionMachine()
        self._route_segments: dict[tuple[str, str], RouteSegment] = {}
        self._active_segment_msg: RouteSegment | None = None
        self._active_token: ExecutionToken | None = None
        self._permit_enabled = False
        self._permit_enabled_started_s: float | None = None
        self._guard_allowed_generation = 0
        self._permit_sequence = 0
        self._report_sequence = 0
        self._terminal_verify_started_s: float | None = None
        self._terminal_verify_pending = False
        self._verification_target_id = ''
        self._current_dwell_s = 0.0
        self._deferred_commands: tuple[MissionCommand, ...] = ()
        self._revalidation_inflight = False
        self._pending_start_run_id = ''
        self._pending_reset_field_id = ''
        self._last_reason = 'idle'
        self._pending_send_command: MissionCommand | None = None
        self._pending_send_started_s: float | None = None

        self._latest_map: MapState | None = None
        self._latest_map_rx_s = -math.inf
        self._latest_guard: GuardState | None = None
        self._latest_guard_rx_s = -math.inf
        self._latest_odom: Odometry | None = None
        self._latest_odom_rx_s = -math.inf
        self._latest_system_ready: bool | None = None
        self._latest_system_ready_rx_s = -math.inf
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._boarding_monitor = ServiceMonitor(
            ServiceMonitorConfig(
                mode=ServiceMode.BOARDING,
                required_dwell_s=float(self.get_parameter('boarding_dwell_s').value),
                stationary_linear_max_mps=float(
                    self.get_parameter('stationary_linear_max_mps').value
                ),
                stationary_angular_max_radps=float(
                    self.get_parameter('stationary_angular_max_radps').value
                ),
            )
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._permit_pub = self.create_publisher(
            MotionPermit,
            str(self.get_parameter('permit_topic').value),
            10,
        )
        self._mission_pub = self.create_publisher(
            MissionState,
            str(self.get_parameter('mission_state_topic').value),
            map_qos,
        )
        self.create_subscription(
            MapState,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            map_qos,
            callback_group=self._state_group,
        )
        self.create_subscription(
            GuardState,
            str(self.get_parameter('guard_state_topic').value),
            self._on_guard,
            10,
            callback_group=self._state_group,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            20,
            callback_group=self._state_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('system_ready_topic').value),
            self._on_system_ready,
            10,
            callback_group=self._state_group,
        )

        self._control_srv = self.create_service(
            ControlMission,
            str(self.get_parameter('control_service').value),
            self._on_control,
            callback_group=self._state_group,
        )
        self._reset_client = self.create_client(
            ResetMap,
            str(self.get_parameter('reset_map_service').value),
            callback_group=self._io_group,
        )
        self._plan_client = self.create_client(
            PlanRoute,
            str(self.get_parameter('plan_service').value),
            callback_group=self._io_group,
        )
        self._validate_client = self.create_client(
            ValidateSegment,
            str(self.get_parameter('validate_service').value),
            callback_group=self._io_group,
        )
        self._report_client = self.create_client(
            ReportExecution,
            str(self.get_parameter('report_service').value),
            callback_group=self._io_group,
        )
        self._observe_client = ActionClient(
            self,
            Observe,
            str(self.get_parameter('observe_action').value),
            callback_group=self._io_group,
        )
        self._nav2 = Nav2Adapter(
            self,
            action_name=str(self.get_parameter('follow_path_action').value),
            callback_group=self._io_group,
        )

        self.create_timer(
            1.0 / float(self.get_parameter('permit_hz').value),
            self._permit_tick,
            callback_group=self._state_group,
        )
        self.create_timer(
            1.0 / float(self.get_parameter('state_hz').value),
            self._state_tick,
            callback_group=self._state_group,
        )
        self.create_timer(
            1.0 / float(self.get_parameter('verification_hz').value),
            self._verification_tick,
            callback_group=self._state_group,
        )
        self.get_logger().info(
            f'WP12 navigator ready; instance={self._navigator_instance_id}; '
            'motion remains disabled until an accepted, validated FollowPath goal.'
        )

    def _declare_parameters(self) -> None:
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'map_topic': '/city/map_state',
            'odom_topic': '/odom',
            'guard_state_topic': '/city/guard_state',
            'permit_topic': '/city/motion_permit',
            'mission_state_topic': '/city/mission_state',
            'system_ready_topic': '/city/system_ready',
            'require_system_ready': False,
            'system_ready_max_age_s': 0.50,
            'control_service': '/city/control_mission',
            'reset_map_service': '/city/reset_map',
            'plan_service': '/city/plan_route',
            'validate_service': '/city/validate_segment',
            'report_service': '/city/report_execution',
            'observe_action': '/city/observe',
            'follow_path_action': '/follow_path',
            'controller_id': '',
            'goal_checker_id': '',
            'progress_checker_id': '',
            'reset_map_on_start': True,
            'required_unique_bus_stops': 2,
            'parking_prior_enabled': False,
            'parking_prior_observation_id': '',
            'permit_hz': 10.0,
            'permit_valid_for_s': 0.20,
            'state_hz': 5.0,
            'verification_hz': 20.0,
            'map_max_age_s': 1.0,
            'odom_max_age_s': 0.30,
            'guard_max_age_s': 0.30,
            'terminal_verify_timeout_s': 2.0,
            'execution_readiness_timeout_s': 2.0,
            'boarding_dwell_s': 2.2,
            'stationary_linear_max_mps': 0.01,
            'stationary_angular_max_radps': 0.03,
            'observation_duration_s': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    # ------------------------------------------------------------------ inputs
    def _on_map(self, msg: MapState) -> None:
        with self._lock:
            previous_revision = self._latest_map.revision if self._latest_map else None
            self._latest_map = msg
            self._latest_map_rx_s = time.monotonic()
            state = self._mission.state()
            if state.phase == MissionPhase.IDLE:
                return
            if msg.run_id != state.run_id or msg.map_instance_id != state.map_instance_id:
                self._safe_pause('map_identity_changed')
                return
            if self._active_token is not None and (not msg.ready or not msg.localization_healthy):
                self._safe_pause('map_or_localization_not_ready_during_motion')
                return
            buses, parking = self._known_targets(msg)
            tr = self._mission.handle(
                self._event(
                    MissionEventKind.MAP_UPDATED,
                    map_revision=int(msg.revision),
                    known_bus_stop_ids=buses,
                    known_parking_target_id=parking,
                    reason='map_updated',
                )
            )
            self._dispatch_transition(tr)
            if (
                previous_revision is not None
                and int(msg.revision) != int(previous_revision)
                and self._active_token is not None
                and self._permit_enabled
            ):
                self._request_live_revalidation()

    def _on_guard(self, msg: GuardState) -> None:
        with self._lock:
            self._latest_guard = msg
            self._latest_guard_rx_s = time.monotonic()
            if self._active_token is None:
                return
            token = self._active_token
            if msg.run_id and msg.run_id != token.run_id:
                self._safe_pause('guard_run_identity_changed')
                return
            if msg.map_instance_id and msg.map_instance_id != token.map_instance_id:
                self._safe_pause('guard_map_identity_changed')
                return
            if msg.navigator_instance_id and msg.navigator_instance_id != token.navigator_instance_id:
                self._safe_pause('guard_navigator_identity_changed')
                return
            if bool(msg.fault_latched):
                self._safe_pause(f'guard_fault:{msg.reason}')
                return
            current_guard = (
                msg.navigator_instance_id == token.navigator_instance_id
                and int(msg.goal_generation) == int(token.goal_generation)
            )
            if current_guard and bool(msg.motion_allowed):
                self._guard_allowed_generation = int(token.goal_generation)
            if current_guard and _guard_denial_is_fatal(
                permit_enabled=self._permit_enabled,
                message_generation=int(msg.goal_generation),
                active_generation=int(token.goal_generation),
                motion_allowed=bool(msg.motion_allowed),
                previously_allowed_generation=self._guard_allowed_generation,
            ):
                self._safe_pause(f'guard_denied_current_generation:{msg.reason}')

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg
            self._latest_odom_rx_s = time.monotonic()

    def _on_system_ready(self, msg: Bool) -> None:
        with self._lock:
            self._latest_system_ready = bool(msg.data)
            self._latest_system_ready_rx_s = time.monotonic()
            if (
                bool(self.get_parameter('require_system_ready').value)
                and not self._latest_system_ready
                and (self._active_token is not None or self._permit_enabled)
            ):
                self._safe_pause('system_readiness_lost')

    def _system_ready_gate_ok(self) -> bool:
        if not bool(self.get_parameter('require_system_ready').value):
            return True
        if self._latest_system_ready is not True:
            return False
        return (
            time.monotonic() - self._latest_system_ready_rx_s
            <= float(self.get_parameter('system_ready_max_age_s').value)
        )

    # -------------------------------------------------------------- control API
    def _on_control(self, request, response):
        with self._lock:
            command = int(request.command)
            if command == int(ControlMission.Request.START):
                return self._control_start(request, response)
            if command == int(ControlMission.Request.PAUSE):
                tr = self._mission.handle(self._event(MissionEventKind.PAUSE, reason=request.reason))
            elif command == int(ControlMission.Request.RESUME):
                tr = self._mission.handle(
                    self._event(
                        MissionEventKind.RESUME,
                        readiness_ok=self._readiness_ok(),
                        reason=request.reason,
                    )
                )
            elif command == int(ControlMission.Request.ABORT):
                tr = self._mission.handle(self._event(MissionEventKind.ABORT, reason=request.reason))
            elif command == int(ControlMission.Request.RESET):
                tr = self._mission.handle(self._event(MissionEventKind.RESET, reason=request.reason))
            else:
                response.accepted = False
                response.run_id = self._mission.state().run_id
                response.phase = int(self._mission.state().phase)
                response.reason = 'unknown_control_command'
                return response
            self._dispatch_transition(tr)
            response.accepted = bool(tr.accepted)
            response.run_id = tr.snapshot.run_id
            response.phase = int(tr.snapshot.phase)
            response.reason = tr.reason
            return response

    def _control_start(self, request, response):
        state = self._mission.state()
        if state.phase != MissionPhase.IDLE:
            response.accepted = False
            response.run_id = state.run_id
            response.phase = int(state.phase)
            response.reason = 'mission_already_started'
            return response
        if self._latest_map is None:
            response.accepted = False
            response.run_id = ''
            response.phase = int(MissionPhase.IDLE)
            response.reason = 'map_state_required_before_start'
            return response
        new_run = f'run-{uuid.uuid4().hex[:12]}'
        field_id = str(self._latest_map.field_id)
        if not field_id:
            response.accepted = False
            response.run_id = ''
            response.phase = int(MissionPhase.IDLE)
            response.reason = 'field_id_missing'
            return response
        if not self._system_ready_gate_ok():
            response.accepted = False
            response.run_id = ''
            response.phase = int(MissionPhase.IDLE)
            response.reason = 'system_not_ready'
            return response
        self._force_motion_disabled('mission_start')
        if bool(self.get_parameter('reset_map_on_start').value):
            if not self._reset_client.service_is_ready():
                response.accepted = False
                response.run_id = ''
                response.phase = int(MissionPhase.IDLE)
                response.reason = 'reset_map_service_not_ready'
                return response
            self._pending_start_run_id = new_run
            self._pending_reset_field_id = field_id
            req = ResetMap.Request()
            req.new_run_id = new_run
            req.field_id = field_id
            future = self._reset_client.call_async(req)
            future.add_done_callback(self._on_start_reset_result)
            response.accepted = True
            response.run_id = new_run
            response.phase = int(MissionPhase.IDLE)
            response.reason = 'start_pending_map_reset'
            return response
        self._start_machine(new_run, str(self._latest_map.map_instance_id), request.reason)
        response.accepted = True
        response.run_id = new_run
        response.phase = int(self._mission.state().phase)
        response.reason = 'mission_started_without_reset'
        return response

    def _on_start_reset_result(self, future) -> None:
        with self._lock:
            run_id = self._pending_start_run_id
            self._pending_start_run_id = ''
            try:
                result = future.result()
            except Exception as exc:
                self._last_reason = f'reset_map_exception:{exc}'
                return
            if not result.accepted or result.run_id != run_id or not result.map_instance_id:
                self._last_reason = f'reset_map_rejected:{result.reason}'
                return
            self._start_machine(run_id, str(result.map_instance_id), 'map_reset_complete')

    def _start_machine(self, run_id: str, map_instance_id: str, reason: str) -> None:
        buses: tuple[str, ...] = ()
        parking = ''
        revision = 0
        if (
            self._latest_map is not None
            and self._latest_map.run_id == run_id
            and self._latest_map.map_instance_id == map_instance_id
        ):
            buses, parking = self._known_targets(self._latest_map)
            revision = int(self._latest_map.revision)
        tr = self._mission.start(
            RunContext(
                run_id=run_id,
                map_instance_id=map_instance_id,
                navigator_instance_id=self._navigator_instance_id,
                required_unique_bus_stops=int(
                    self.get_parameter('required_unique_bus_stops').value
                ),
                parking_prior_enabled=bool(
                    self.get_parameter('parking_prior_enabled').value
                ),
                parking_prior_valid=True,
                parking_prior_observation_id=str(
                    self.get_parameter('parking_prior_observation_id').value
                ),
                known_bus_stop_ids=buses,
                known_parking_target_id=parking,
                map_revision=revision,
            )
        )
        self._last_reason = reason
        self._dispatch_transition(tr)

    # ---------------------------------------------------------- mission commands
    def _dispatch_transition(self, transition) -> None:
        self._last_reason = transition.reason
        self._dispatch_commands(transition.commands)

    def _dispatch_commands(self, commands: tuple[MissionCommand, ...]) -> None:
        if not commands:
            return
        # Safety invariant: any transition that wants cancel/disable first
        # revokes permission before interacting with Nav2.
        if any(c.kind in (MissionCommandKind.CANCEL_AND_HOLD, MissionCommandKind.DISABLE_MOTION) for c in commands):
            self._force_motion_disabled('mission_transition_hold')
        report_index = next((i for i, c in enumerate(commands) if c.kind == MissionCommandKind.REPORT_EXECUTION), None)
        if report_index is not None:
            self._deferred_commands = tuple(commands[report_index + 1:])
            self._execute_command(commands[report_index])
            for command in commands[:report_index]:
                self._execute_command(command)
            return
        for command in commands:
            self._execute_command(command)

    def _execute_command(self, command: MissionCommand) -> None:
        kind = command.kind
        if kind == MissionCommandKind.REQUEST_PLAN:
            self._request_plan(command)
        elif kind == MissionCommandKind.REQUEST_VALIDATE_SEGMENT:
            self._request_validate(command, live_revalidation=False)
        elif kind == MissionCommandKind.SEND_SEGMENT:
            self._send_segment(command)
        elif kind == MissionCommandKind.REQUEST_OBSERVE:
            self._request_observe(command)
        elif kind == MissionCommandKind.REPORT_EXECUTION:
            self._report_execution(command)
        elif kind == MissionCommandKind.VERIFY_STOP:
            self._begin_service_verification(command.target_id)
        elif kind == MissionCommandKind.CANCEL_AND_HOLD:
            self._nav2.cancel_active()
        elif kind == MissionCommandKind.DISABLE_MOTION:
            self._force_motion_disabled(command.reason or 'mission_disable')
        elif kind == MissionCommandKind.RESET_MAP:
            self._reset_after_mission_reset()
        elif kind in (MissionCommandKind.COMPLETE, MissionCommandKind.NONE, MissionCommandKind.START_DWELL):
            return

    def _request_plan(self, command: MissionCommand) -> None:
        if not self._plan_client.service_is_ready():
            self._safe_pause('plan_service_not_ready')
            return
        state = self._mission.state()
        req = PlanRoute.Request()
        req.run_id = state.run_id
        req.map_instance_id = state.map_instance_id
        req.request_id = int(command.request_id)
        req.minimum_revision = int(state.map_revision)
        req.goal_kind = int(command.goal_kind)
        req.target_id = str(command.target_id)
        req.exclude_target_ids = list(state.serviced_stop_ids)
        future = self._plan_client.call_async(req)
        future.add_done_callback(lambda f, rid=command.request_id: self._on_plan_result(f, rid))

    def _on_plan_result(self, future, request_id: int) -> None:
        with self._lock:
            state = self._mission.state()
            if int(request_id) != int(state.current_request_id):
                return
            try:
                result = future.result()
            except Exception as exc:
                self._safe_pause(f'plan_exception:{exc}')
                return
            if result.run_id != state.run_id or result.map_instance_id != state.map_instance_id:
                self._safe_pause('plan_result_identity_mismatch')
                return
            segment_id = ''
            if result.executable_prefix:
                for segment in result.executable_prefix:
                    self._route_segments[(str(result.route_id), str(segment.segment_id))] = deepcopy(segment)
                segment_id = str(result.executable_prefix[0].segment_id)
            tr = self._mission.handle(
                self._event(
                    MissionEventKind.PLAN_RESULT,
                    request_id=int(result.request_id),
                    map_revision=int(result.revision),
                    route_id=str(result.route_id),
                    segment_id=segment_id,
                    plan_status=PlanStatus(int(result.status)),
                    observation_id=str(result.terminal_observation_id),
                    reason=str(result.reason),
                )
            )
            self._dispatch_transition(tr)

    def _request_validate(self, command: MissionCommand, *, live_revalidation: bool) -> None:
        if not self._validate_client.service_is_ready():
            self._safe_pause('validate_service_not_ready')
            return
        state = self._mission.state()
        req = ValidateSegment.Request()
        req.run_id = state.run_id
        req.map_instance_id = state.map_instance_id
        req.route_id = command.route_id or state.route_id
        req.segment_id = command.segment_id or state.active_segment_id
        future = self._validate_client.call_async(req)
        future.add_done_callback(
            lambda f, lr=live_revalidation, route=req.route_id, segment=req.segment_id:
            self._on_validate_result(f, lr, route, segment)
        )

    def _on_validate_result(self, future, live_revalidation: bool, route_id: str, segment_id: str) -> None:
        with self._lock:
            if live_revalidation:
                self._revalidation_inflight = False
            try:
                result = future.result()
            except Exception as exc:
                self._safe_pause(f'validate_exception:{exc}')
                return
            if live_revalidation:
                if self._active_token is None:
                    return
                if self._active_token.route_id != route_id or self._active_token.segment_id != segment_id:
                    return
                if int(result.status) != int(ValidateSegment.Response.VALID):
                    self._safe_pause(f'live_revalidation_failed:{result.reason}')
                return
            tr = self._mission.handle(
                self._event(
                    MissionEventKind.SEGMENT_VALIDATION_RESULT,
                    route_id=route_id,
                    segment_id=segment_id,
                    validation_status=ValidationStatus(int(result.status)),
                    observation_id=str(result.observation_id),
                    map_revision=int(result.current_revision),
                    reason=str(result.reason),
                )
            )
            self._dispatch_transition(tr)

    def _request_live_revalidation(self) -> None:
        if self._revalidation_inflight or self._active_token is None:
            return
        self._revalidation_inflight = True
        token = self._active_token
        self._request_validate(
            MissionCommand(
                MissionCommandKind.REQUEST_VALIDATE_SEGMENT,
                route_id=token.route_id,
                segment_id=token.segment_id,
                reason='map_revision_changed_live_revalidate',
            ),
            live_revalidation=True,
        )

    def _send_segment(self, command: MissionCommand) -> None:
        state = self._mission.state()
        key = (command.route_id, command.segment_id)
        segment = self._route_segments.get(key)
        if segment is None:
            self._safe_pause('validated_segment_missing_from_route_cache')
            return
        if self._active_token is not None or self._nav2.has_active_goal:
            self._safe_pause('second_movement_goal_rejected')
            return
        if not self._readiness_ok() or not self._nav2.server_ready():
            if self._pending_send_command is None:
                self._pending_send_command = command
                self._pending_send_started_s = time.monotonic()
            self._last_reason = 'waiting_execution_readiness'
            return
        self._pending_send_command = None
        self._pending_send_started_s = None
        try:
            path = route_segment_to_nav_path(
                segment,
                map_frame=str(self.get_parameter('map_frame').value),
            )
        except Exception as exc:
            self._safe_pause(f'route_path_conversion_failed:{exc}')
            return
        token = ExecutionToken(
            run_id=state.run_id,
            map_instance_id=state.map_instance_id,
            navigator_instance_id=self._navigator_instance_id,
            route_id=command.route_id,
            segment_id=command.segment_id,
            goal_generation=int(command.goal_generation),
        )
        self._active_segment_msg = deepcopy(segment)
        self._active_token = token
        self._force_motion_disabled('before_follow_path_send', preserve_token=True)
        sent = self._nav2.send_path(
            path=path,
            token=token,
            controller_id=str(self.get_parameter('controller_id').value),
            goal_checker_id=str(self.get_parameter('goal_checker_id').value),
            progress_checker_id=str(self.get_parameter('progress_checker_id').value),
            on_accepted=self._on_nav2_accepted,
            on_result=self._on_nav2_result,
        )
        if not sent:
            self._safe_pause('nav2_adapter_busy')

    def _on_nav2_accepted(self, token: ExecutionToken, accepted: bool, reason: str) -> None:
        with self._lock:
            if token != self._active_token or not self._token_current(token):
                return
            if not accepted:
                self._force_motion_disabled(reason)
                self._active_token = None
                self._active_segment_msg = None
                self._safe_pause(reason)
                return
            # Permit is enabled only after Nav2 has accepted the exact generation.
            self._guard_allowed_generation = 0
            self._permit_enabled_started_s = time.monotonic()
            self._permit_enabled = True
            self._publish_permit(enabled=True)
            tr = self._mission.handle(
                self._event(
                    MissionEventKind.GOAL_ACCEPTED,
                    goal_generation=token.goal_generation,
                    route_id=token.route_id,
                    segment_id=token.segment_id,
                    reason='nav2_goal_accepted',
                )
            )
            self._dispatch_transition(tr)

    def _on_nav2_result(self, result: Nav2Result) -> None:
        with self._lock:
            token = result.token
            if token != self._active_token or not self._token_current(token):
                return
            # Invariant: revoke permission before processing any Nav2 result.
            self._force_motion_disabled('nav2_result', preserve_token=True)
            if result.completion != Nav2Completion.SUCCEEDED:
                self._active_token = None
                self._active_segment_msg = None
                self._safe_pause(
                    f'nav2_not_success:{result.completion.name}:{result.error_code}:{result.error_msg}'
                )
                return
            self._terminal_verify_pending = True
            self._terminal_verify_started_s = time.monotonic()
            self._last_reason = 'nav2_success_waiting_terminal_verification'

    def _request_observe(self, command: MissionCommand) -> None:
        self._force_motion_disabled('before_observe')
        if not self._observe_client.server_is_ready():
            self._safe_pause('observe_action_not_ready')
            return
        state = self._mission.state()
        goal = Observe.Goal()
        goal.run_id = state.run_id
        goal.map_instance_id = state.map_instance_id
        goal.observation_id = command.observation_id
        goal.max_duration_s = float(self.get_parameter('observation_duration_s').value)
        future = self._observe_client.send_goal_async(goal)
        future.add_done_callback(self._on_observe_goal_response)

    def _on_observe_goal_response(self, future) -> None:
        with self._lock:
            try:
                handle = future.result()
            except Exception as exc:
                self._safe_pause(f'observe_goal_exception:{exc}')
                return
            if handle is None or not handle.accepted:
                self._safe_pause('observe_goal_rejected')
                return
            rf = handle.get_result_async()
            rf.add_done_callback(self._on_observe_result)

    def _on_observe_result(self, future) -> None:
        with self._lock:
            try:
                wrapped = future.result()
                result = wrapped.result
            except Exception as exc:
                self._safe_pause(f'observe_result_exception:{exc}')
                return
            outcome = ObservationOutcome(int(result.outcome))
            target_id = str(result.confirmed_target_ids[0]) if result.confirmed_target_ids else ''
            target_kind = self._target_kind(target_id)
            tr = self._mission.handle(
                self._event(
                    MissionEventKind.OBSERVATION_RESULT,
                    map_revision=int(result.revision),
                    observation_outcome=outcome,
                    target_id=target_id,
                    target_kind=target_kind,
                    reason=str(result.reason),
                )
            )
            self._dispatch_transition(tr)

    def _report_execution(self, command: MissionCommand) -> None:
        if not self._report_client.service_is_ready():
            self._deferred_commands = ()
            self._safe_pause('report_service_not_ready')
            return
        state = self._mission.state()
        self._report_sequence += 1
        req = ReportExecution.Request()
        req.run_id = state.run_id
        req.map_instance_id = state.map_instance_id
        req.navigator_instance_id = self._navigator_instance_id
        req.report_sequence = self._report_sequence
        req.route_id = command.route_id
        req.segment_id = command.segment_id
        req.outcome = int(ReportExecution.Request.COMPLETED)
        measured_pose = self._current_pose_stamped()
        if measured_pose is None:
            self._deferred_commands = ()
            self._safe_pause('map_to_base_tf_missing_for_execution_report')
            return
        req.measured_terminal_pose = measured_pose
        req.reason = command.reason or 'terminal_verified'
        future = self._report_client.call_async(req)
        future.add_done_callback(self._on_report_result)

    def _on_report_result(self, future) -> None:
        with self._lock:
            try:
                result = future.result()
            except Exception as exc:
                self._deferred_commands = ()
                self._safe_pause(f'report_exception:{exc}')
                return
            if not result.accepted and not result.duplicate:
                self._deferred_commands = ()
                self._safe_pause(f'execution_report_rejected:{result.reason}')
                return
            commands = self._deferred_commands
            self._deferred_commands = ()
            self._dispatch_commands(commands)

    # ---------------------------------------------------------- terminal/service
    def _verification_tick(self) -> None:
        with self._lock:
            now = time.monotonic()
            if self._pending_send_command is not None:
                if self._readiness_ok() and self._nav2.server_ready():
                    pending = self._pending_send_command
                    self._pending_send_command = None
                    self._pending_send_started_s = None
                    self._send_segment(pending)
                elif (
                    self._pending_send_started_s is not None
                    and now - self._pending_send_started_s
                    > float(self.get_parameter('execution_readiness_timeout_s').value)
                ):
                    self._safe_pause('execution_readiness_timeout')
                return
            if self._active_token is not None and self._permit_enabled:
                if self._guard_allowed_generation != self._active_token.goal_generation:
                    if (
                        self._permit_enabled_started_s is not None
                        and now - self._permit_enabled_started_s
                        > float(self.get_parameter('execution_readiness_timeout_s').value)
                    ):
                        self._safe_pause('guard_activation_timeout')
                    return
                if not self._runtime_inputs_fresh(now):
                    self._safe_pause('runtime_input_stale_during_motion')
                    return
            if self._terminal_verify_pending:
                if self._terminal_verified(now):
                    self._complete_verified_segment()
                elif (
                    self._terminal_verify_started_s is not None
                    and now - self._terminal_verify_started_s
                    > float(self.get_parameter('terminal_verify_timeout_s').value)
                ):
                    self._safe_pause('terminal_verification_timeout')
                return
            if self._verification_target_id:
                self._service_tick(now)

    def _terminal_verified(self, now: float) -> bool:
        token = self._active_token
        segment = self._active_segment_msg
        if token is None or segment is None or not self._token_current(token):
            return False
        if not self._runtime_inputs_fresh(now):
            return False
        guard = self._latest_guard
        if guard is None or guard.fault_latched or not guard.stopped:
            return False
        if guard.run_id and guard.run_id != token.run_id:
            return False
        if guard.map_instance_id and guard.map_instance_id != token.map_instance_id:
            return False
        if guard.navigator_instance_id and guard.navigator_instance_id != token.navigator_instance_id:
            return False
        if guard.goal_generation not in (0, token.goal_generation):
            return False
        pose = self._odom_pose()
        if pose is None:
            return False
        terminal = segment.terminal_limit.pose
        tx, ty, tyaw = self._pose_components(terminal)
        px, py, pyaw = pose
        if math.hypot(px - tx, py - ty) > float(segment.xy_tolerance_m) + 1e-6:
            return False
        if abs(self._angle_delta(pyaw, tyaw)) > float(segment.yaw_tolerance_rad) + 1e-6:
            return False
        return True

    def _complete_verified_segment(self) -> None:
        token = self._active_token
        segment = self._active_segment_msg
        if token is None or segment is None:
            return
        self._terminal_verify_pending = False
        self._terminal_verify_started_s = None
        self._report_sequence += 1
        reached = int(segment.terminal_kind) in (
            int(RouteSegment.BUS_SERVICE),
            int(RouteSegment.PARK),
        )
        tr = self._mission.handle(
            self._event(
                MissionEventKind.EXECUTION_COMPLETED,
                goal_generation=token.goal_generation,
                route_id=token.route_id,
                segment_id=token.segment_id,
                report_sequence=self._report_sequence,
                reached_objective=reached,
                reason='nav2_success_and_terminal_verified',
            )
        )
        # Mission has consumed the generation.  Keep terminal segment available
        # for service verification, but it no longer authorizes movement.
        self._active_token = None
        self._nav2.abandon(token)
        self._dispatch_transition(tr)

    def _begin_service_verification(self, target_id: str) -> None:
        self._verification_target_id = str(target_id)
        self._boarding_monitor.reset('begin_service_verification')
        self._current_dwell_s = 0.0

    def _service_tick(self, now: float) -> None:
        state = self._mission.state()
        target = self._find_target(self._verification_target_id)
        if target is None or self._active_segment_msg is None:
            return
        pose = self._odom_pose()
        if pose is None:
            return
        px, py, pyaw = pose
        service = target.service_pose.pose
        sx, sy, syaw = self._pose_components(service)
        terminal = target.terminal_limit.pose
        tx, ty, tyaw = self._pose_components(terminal)
        odom = self._latest_odom
        guard = self._latest_guard
        if odom is None or guard is None:
            return
        lin = math.hypot(float(odom.twist.twist.linear.x), float(odom.twist.twist.linear.y))
        ang = abs(float(odom.twist.twist.angular.z))
        long_from_terminal = (px - tx) * math.cos(tyaw) + (py - ty) * math.sin(tyaw)
        sample = StopPredicateSample(
            run_id=state.run_id,
            map_instance_id=state.map_instance_id,
            target_id=self._verification_target_id,
            target_confirmed=int(target.state) == 1,
            geometry_valid=bool(target.geometry_valid),
            correct_directed_approach=(
                str(self._active_segment_msg.terminal_id) == self._verification_target_id
            ),
            within_xy_tolerance=math.hypot(px - sx, py - sy) <= float(target.xy_tolerance_m) + 1e-6,
            within_yaw_tolerance=abs(self._angle_delta(pyaw, syaw)) <= float(target.yaw_tolerance_rad) + 1e-6,
            front_before_terminal_limit=long_from_terminal <= 0.02,
            measured_linear_speed_mps=lin,
            measured_angular_speed_radps=ang,
            odometry_fresh=(now - self._latest_odom_rx_s) <= float(self.get_parameter('odom_max_age_s').value),
            localization_fresh=(now - self._latest_map_rx_s) <= float(self.get_parameter('map_max_age_s').value),
            guard_zero_output=(not bool(guard.motion_allowed)),
            guard_measured_stopped=bool(guard.stopped),
            movement_action_active=(self._active_token is not None),
            target_conflict=int(target.state) == 2,
            identity_current=(
                target.target_id == self._verification_target_id
                and state.run_id == self._latest_map.run_id
                and state.map_instance_id == self._latest_map.map_instance_id
            ),
        )
        if int(target.kind) == int(TargetKind.PARKING):
            if self._parking_predicate_valid(sample):
                self._verification_target_id = ''
                tr = self._mission.handle(
                    self._event(
                        MissionEventKind.PARKING_STOP_VERIFIED,
                        target_id=target.target_id,
                        reason='parking_stop_verified',
                    )
                )
                self._dispatch_transition(tr)
            return
        progress = self._boarding_monitor.update(sample, now)
        self._current_dwell_s = float(progress.dwell_s)
        if progress.completed_now:
            receipt = ServiceReceipt(
                run_id=state.run_id,
                target_id=target.target_id,
                stop_entry_s=now - progress.dwell_s,
                stop_exit_s=now,
                dwell_duration_s=progress.dwell_s,
                final_pose=Pose2D(px, py, pyaw),
                max_measured_speed_mps=progress.max_linear_speed_mps,
                map_revision=state.map_revision,
            )
            self._verification_target_id = ''
            self._current_dwell_s = 0.0
            tr = self._mission.handle(
                self._event(
                    MissionEventKind.SERVICE_COMPLETED,
                    target_id=target.target_id,
                    receipt=receipt,
                    reason='boarding_dwell_complete',
                )
            )
            self._dispatch_transition(tr)

    @staticmethod
    def _parking_predicate_valid(sample: StopPredicateSample) -> bool:
        return all((
            sample.identity_current,
            sample.target_confirmed,
            sample.geometry_valid,
            sample.correct_directed_approach,
            sample.within_xy_tolerance,
            sample.within_yaw_tolerance,
            sample.front_before_terminal_limit,
            sample.odometry_fresh,
            sample.localization_fresh,
            sample.guard_zero_output,
            sample.guard_measured_stopped,
            not sample.movement_action_active,
            not sample.target_conflict,
        ))

    # --------------------------------------------------------------- heartbeat
    def _permit_tick(self) -> None:
        with self._lock:
            if self._permit_enabled:
                self._publish_permit(enabled=True)

    def _publish_permit(self, *, enabled: bool) -> None:
        self._permit_sequence += 1
        state = self._mission.state()
        field_id = self._latest_map.field_id if self._latest_map is not None else ''
        generation = self._active_token.goal_generation if self._active_token else state.goal_generation
        msg = build_motion_permit_msg(
            now_msg=self.get_clock().now().to_msg(),
            run_id=state.run_id,
            map_instance_id=state.map_instance_id,
            navigator_instance_id=self._navigator_instance_id,
            field_id=str(field_id),
            goal_generation=int(generation),
            sequence=self._permit_sequence,
            enabled=enabled,
            valid_for_s=float(self.get_parameter('permit_valid_for_s').value),
            segment_msg=self._active_segment_msg if enabled else None,
        )
        self._permit_pub.publish(msg)

    def _force_motion_disabled(self, reason: str, *, preserve_token: bool = False) -> None:
        was_enabled = self._permit_enabled
        self._permit_enabled = False
        self._permit_enabled_started_s = None
        if was_enabled or self._mission.state().run_id:
            self._publish_permit(enabled=False)
        self._last_reason = reason
        if not preserve_token:
            self._terminal_verify_pending = False
            self._terminal_verify_started_s = None

    def _state_tick(self) -> None:
        with self._lock:
            snapshot = self._mission.state()
            msg = mission_state_to_msg(
                snapshot,
                now_msg=self.get_clock().now().to_msg(),
                current_dwell_s=self._current_dwell_s,
            )
            self._mission_pub.publish(msg)

    # ------------------------------------------------------------- fail-closed
    def _safe_pause(self, reason: str) -> None:
        # Required ordering: permission OFF first, cancel second.
        self._force_motion_disabled(reason)
        self._nav2.cancel_active()
        self._nav2.abandon()
        self._active_token = None
        self._active_segment_msg = None
        self._terminal_verify_pending = False
        self._verification_target_id = ''
        self._pending_send_command = None
        self._pending_send_started_s = None
        state = self._mission.state()
        if state.phase not in (MissionPhase.IDLE, MissionPhase.PAUSED, MissionPhase.DONE, MissionPhase.ABORTED):
            tr = self._mission.handle(self._event(MissionEventKind.PAUSE, reason=reason))
            # Do not recursively execute CANCEL/DISABLE; those actions are done.
            self._last_reason = tr.reason
        else:
            self._last_reason = reason

    def _reset_after_mission_reset(self) -> None:
        self._force_motion_disabled('mission_reset')
        self._nav2.cancel_active()
        self._nav2.abandon()
        self._route_segments.clear()
        self._active_token = None
        self._active_segment_msg = None
        self._verification_target_id = ''
        self._pending_send_command = None
        self._pending_send_started_s = None
        self._boarding_monitor.reset('mission_reset')

    # ----------------------------------------------------------------- helpers
    def _event(self, kind: MissionEventKind, **kwargs) -> MissionEvent:
        state = self._mission.state()
        kwargs.setdefault('run_id', state.run_id)
        kwargs.setdefault('map_instance_id', state.map_instance_id)
        kwargs.setdefault('navigator_instance_id', self._navigator_instance_id)
        return MissionEvent(kind=kind, **kwargs)

    def _token_current(self, token: ExecutionToken) -> bool:
        state = self._mission.state()
        return (
            token.run_id == state.run_id
            and token.map_instance_id == state.map_instance_id
            and token.navigator_instance_id == self._navigator_instance_id
            and token.goal_generation == state.goal_generation
            and token.route_id == state.route_id
            and token.segment_id == state.active_segment_id
        )

    def _readiness_ok(self) -> bool:
        now = time.monotonic()
        if not self._system_ready_gate_ok():
            return False
        state = self._mission.state()
        if self._latest_map is None or not bool(self._latest_map.ready):
            return False
        if (
            self._latest_map.run_id != state.run_id
            or self._latest_map.map_instance_id != state.map_instance_id
        ):
            return False
        if not bool(self._latest_map.localization_healthy):
            return False
        if not self._runtime_inputs_fresh(now):
            return False
        if self._latest_guard is not None:
            if bool(self._latest_guard.fault_latched):
                return False
            if self._latest_guard.run_id and self._latest_guard.run_id != state.run_id:
                return False
            if (
                self._latest_guard.map_instance_id
                and self._latest_guard.map_instance_id != state.map_instance_id
            ):
                return False
        return True

    def _runtime_inputs_fresh(self, now: float) -> bool:
        return (
            self._latest_map is not None
            and self._latest_odom is not None
            and self._latest_guard is not None
            and now - self._latest_map_rx_s <= float(self.get_parameter('map_max_age_s').value)
            and now - self._latest_odom_rx_s <= float(self.get_parameter('odom_max_age_s').value)
            and now - self._latest_guard_rx_s <= float(self.get_parameter('guard_max_age_s').value)
        )

    def _known_targets(self, msg: MapState) -> tuple[tuple[str, ...], str]:
        buses = []
        parking = ''
        for target in msg.targets:
            if int(target.state) != 1:
                continue
            if int(target.kind) == int(TargetKind.BUS_STOP):
                buses.append(str(target.target_id))
            elif int(target.kind) == int(TargetKind.PARKING) and not parking:
                parking = str(target.target_id)
        return tuple(sorted(set(buses))), parking

    def _target_kind(self, target_id: str) -> TargetKind:
        target = self._find_target(target_id)
        if target is not None and int(target.kind) == int(TargetKind.PARKING):
            return TargetKind.PARKING
        return TargetKind.BUS_STOP

    def _find_target(self, target_id: str):
        if self._latest_map is None:
            return None
        return next((t for t in self._latest_map.targets if t.target_id == target_id), None)

    def _odom_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                Time(),
            )
        except Exception:
            return None
        return self._pose_components(transform.transform)

    def _current_pose_stamped(self) -> PoseStamped | None:
        pose = self._odom_pose()
        if pose is None:
            return None
        x_m, y_m, yaw_rad = pose
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = str(self.get_parameter('map_frame').value)
        p.pose.position.x = x_m
        p.pose.position.y = y_m
        p.pose.orientation.z = math.sin(yaw_rad * 0.5)
        p.pose.orientation.w = math.cos(yaw_rad * 0.5)
        return p

    @staticmethod
    def _pose_components(pose) -> tuple[float, float, float]:
        if hasattr(pose, 'translation'):
            position = pose.translation
            q = pose.rotation
        else:
            position = pose.position
            q = pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(position.x), float(position.y), math.atan2(siny, cosy)

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def destroy_node(self):
        with self._lock:
            self._force_motion_disabled('navigator_shutdown')
            self._nav2.cancel_active()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
