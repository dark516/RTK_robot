"""ROS 2 wrapper for the verified Frob City map/planning stack.

WP11B integrates the frozen WP06 perception/observation core with WP07 semantic
state and WP08 route planning.  ROS callbacks stay at the boundary: capture-time
pose history, detector conversion, observation action wiring, map mutation, and
service/action transport live here while the domain modules remain ROS-free.
"""

from __future__ import annotations

from collections import Counter, OrderedDict, deque
from dataclasses import dataclass
import math
from pathlib import Path
from threading import RLock
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from frob_city_interfaces.action import Observe
from frob_city_interfaces.msg import MapState, SignDetectionArray
from frob_city_interfaces.srv import PlanRoute, ReportExecution, ResetMap, ValidateSegment

from .config_loader import canonical_field_digest, load_project_config
from .field_model import FieldModel
from .models import (
    ConfigPaths,
    CorePlanResult,
    ExecutionEvent,
    ExecutionEventKind,
    ExecutionOutcome,
    ObserveOutcome,
    PlanStatus,
    PoseSample,
    SegmentValidation,
    SegmentValidationStatus,
)
from .path_builder import validate_segment
from .planner import RoutePlanner
from .ros_map_conversions import (
    detection_array_to_core,
    fill_plan_response,
    fill_validation_response,
    map_snapshot_to_msg,
    odometry_to_pose_sample,
    plan_request_from_ros,
    pose_stamped_to_pose2d,
    transform_to_pose2d,
)
from .semantic_map import SemanticMap
from .sign_fusion import (
    CanonicalDetection,
    CanonicalDetectionFrame,
    ObservationContext,
    SignFusion,
    SignFusionConfig,
    SiteCandidate,
)
from .observation import ObservationFrame, ObservationSession


MAP_STATE_TOPIC = "/city/map_state"
DETECTION_TOPIC = "/traffic_sign/detections"
ODOM_TOPIC = "/odom"
PLAN_SERVICE = "/city/plan_route"
VALIDATE_SERVICE = "/city/validate_segment"
REPORT_SERVICE = "/city/report_execution"
RESET_SERVICE = "/city/reset_map"
OBSERVE_ACTION = "/city/observe"
DIAGNOSTICS_TOPIC = "/diagnostics"


@dataclass(frozen=True, slots=True)
class _PoseLookupResult:
    ok: bool
    reason: str
    sample: PoseSample | None = None


class _PoseHistoryPort:
    """Bounded capture-time pose provider for the pure WP06 fusion core."""

    def __init__(self, max_samples: int = 512, max_gap_s: float = 0.20) -> None:
        self._samples: deque[PoseSample] = deque(maxlen=max_samples)
        self._max_gap_s = float(max_gap_s)

    def clear(self) -> None:
        self._samples.clear()

    def add(self, sample: PoseSample) -> None:
        if self._samples and sample.timestamp_s < self._samples[-1].timestamp_s:
            # Preserve deterministic ordering after localization resets/out-of-order callbacks.
            merged = list(self._samples) + [sample]
            merged.sort(key=lambda item: item.timestamp_s)
            self._samples = deque(merged[-self._samples.maxlen :], maxlen=self._samples.maxlen)
            return
        self._samples.append(sample)

    def pose_at(self, stamp_s: float) -> _PoseLookupResult:
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s):
            return _PoseLookupResult(False, "capture_timestamp_non_finite")
        samples = tuple(self._samples)
        if not samples:
            return _PoseLookupResult(False, "pose_history_empty")

        for item in samples:
            if abs(item.timestamp_s - stamp_s) <= 1e-6:
                return _PoseLookupResult(True, "exact", item)

        before = None
        after = None
        for item in samples:
            if item.timestamp_s < stamp_s:
                before = item
                continue
            if item.timestamp_s > stamp_s:
                after = item
                break

        if before is None or after is None:
            return _PoseLookupResult(False, "capture_pose_not_bracketed")
        gap = after.timestamp_s - before.timestamp_s
        if gap <= 0.0 or gap > self._max_gap_s:
            return _PoseLookupResult(False, "capture_pose_gap_too_large")

        alpha = (stamp_s - before.timestamp_s) / gap
        yaw_delta = math.atan2(
            math.sin(after.pose.yaw_rad - before.pose.yaw_rad),
            math.cos(after.pose.yaw_rad - before.pose.yaw_rad),
        )
        from .models import Pose2D

        pose = Pose2D(
            x_m=before.pose.x_m + alpha * (after.pose.x_m - before.pose.x_m),
            y_m=before.pose.y_m + alpha * (after.pose.y_m - before.pose.y_m),
            yaw_rad=before.pose.yaw_rad + alpha * yaw_delta,
        )
        sample = PoseSample(
            timestamp_s=stamp_s,
            pose=pose,
            linear_speed_mps=max(before.linear_speed_mps, after.linear_speed_mps),
            angular_speed_radps=max(before.angular_speed_radps, after.angular_speed_radps),
            covariance_xy_m2=max(
                [v for v in (before.covariance_xy_m2, after.covariance_xy_m2) if v is not None],
                default=None,
            ),
            covariance_yaw_rad2=max(
                [v for v in (before.covariance_yaw_rad2, after.covariance_yaw_rad2) if v is not None],
                default=None,
            ),
        )
        return _PoseLookupResult(True, "interpolated", sample)


class _FieldQueryAdapter:
    """Associate image detections with calibrated physical sign faces.

    Detector range is optical-Z, so candidate depth is compared along the
    robot/camera forward axis.  Horizontal bbox position supplies the missing
    bearing.  The adapter deliberately returns more than one usable candidate
    when geometry cannot distinguish them; SignFusion then fails closed rather
    than assigning a sign to an arbitrary fixture.
    """

    def __init__(
        self,
        field: FieldModel,
        *,
        focal_length_x_px: float,
        principal_point_x_px: float,
        max_detection_range_m: float,
        max_bearing_error_rad: float,
        max_depth_error_m: float,
        face_view_tolerance_rad: float,
        min_right_lateral_m: float,
        score_margin: float,
    ) -> None:
        self._field = field
        self._fx_px = float(focal_length_x_px)
        self._cx_px = float(principal_point_x_px)
        self._max_detection_range_m = float(max_detection_range_m)
        self._max_bearing_error_rad = float(max_bearing_error_rad)
        self._max_depth_error_m = float(max_depth_error_m)
        self._face_view_tolerance_rad = float(face_view_tolerance_rad)
        self._min_right_lateral_m = float(min_right_lateral_m)
        self._score_margin = float(score_margin)
        if self._fx_px <= 0.0:
            raise ValueError("sign_camera_focal_length_x_px must be > 0")

    @staticmethod
    def _angle_delta(a_rad: float, b_rad: float) -> float:
        return math.atan2(math.sin(a_rad - b_rad), math.cos(a_rad - b_rad))

    def candidates(self, *, pose: PoseSample, detection) -> tuple[SiteCandidate, ...]:
        x1, _, x2, _ = detection.bbox_xyxy_px
        image_u_px = 0.5 * (x1 + x2)
        # Image +u is camera-right while the robot planar convention is +left.
        measured_bearing = -math.atan2(image_u_px - self._cx_px, self._fx_px)
        measured_depth = detection.range_m
        optical_z_range = detection.range_convention.name == "OPTICAL_Z"
        measured_ray_range = None
        if measured_depth is not None:
            if optical_z_range:
                measured_ray_range = measured_depth / max(0.20, math.cos(measured_bearing))
            else:
                measured_ray_range = measured_depth

        candidates: list[SiteCandidate] = []
        for site_id in self._field.sign_site_ids:
            site = self._field.sign_site(site_id)
            point = self._field.field_to_map_point(site.position)
            dx = point.x_m - pose.pose.x_m
            dy = point.y_m - pose.pose.y_m
            cos_yaw = math.cos(pose.pose.yaw_rad)
            sin_yaw = math.sin(pose.pose.yaw_rad)
            forward_m = cos_yaw * dx + sin_yaw * dy
            left_m = -sin_yaw * dx + cos_yaw * dy
            ray_range_m = math.hypot(forward_m, left_m)
            bearing = math.atan2(left_m, forward_m)
            bearing_error = abs(self._angle_delta(bearing, measured_bearing))
            candidate_range_m = forward_m if optical_z_range else ray_range_m
            depth_error = (
                abs(candidate_range_m - measured_depth)
                if measured_depth is not None
                else 0.0
            )
            for face in site.faces:
                face_yaw_map = face.yaw_rad + self._field.config.field_to_map.yaw_rad
                site_to_robot_yaw = math.atan2(-dy, -dx)
                face_error = abs(self._angle_delta(site_to_robot_yaw, face_yaw_map))
                within_distance = (
                    forward_m > 0.05
                    and ray_range_m <= self._max_detection_range_m
                    and (
                        measured_ray_range is None
                        or measured_ray_range <= self._max_detection_range_m
                    )
                )
                score = (
                    bearing_error / max(self._max_bearing_error_rad, 1e-6)
                    + depth_error / max(self._max_depth_error_m, 1e-6)
                )
                candidates.append(
                    SiteCandidate(
                        site_id=site.site_id,
                        face_id=face.face_id,
                        controlled_approach_id=face.controlled_approach_id,
                        bearing_rad=bearing,
                        # SignFusion must compare the same convention published
                        # by the detector (OPTICAL_Z on the live robot).
                        range_m=candidate_range_m,
                        face_visible=face_error <= self._face_view_tolerance_rad,
                        right_roadside=left_m <= -self._min_right_lateral_m,
                        current_junction=within_distance,
                        bearing_consistent=bearing_error <= self._max_bearing_error_rad,
                        association_score=score,
                    )
                )

        usable = [
            item
            for item in candidates
            if item.face_visible
            and item.right_roadside
            and item.current_junction
            and item.bearing_consistent
            and (
                measured_depth is None
                or abs(item.range_m - measured_depth) <= self._max_depth_error_m
            )
        ]
        if len(usable) <= 1:
            return tuple(usable) if usable else tuple(candidates)

        ranked = sorted(usable, key=lambda item: (item.association_score, item.face_key))
        if ranked[1].association_score - ranked[0].association_score >= self._score_margin:
            return (ranked[0],)
        return tuple(ranked)


@dataclass(slots=True)
class _ActiveObservation:
    run_id: str
    map_instance_id: str
    observation_id: str
    approach_id: str
    started_ros_s: float
    deadline_monotonic: float
    session: ObservationSession
    accepted_frames: int = 0
    first_capture_s: float | None = None
    last_capture_s: float | None = None
    last_reason: str = "waiting_for_frames"
    seen_site_ids: set[str] | None = None

    def __post_init__(self) -> None:
        if self.seen_site_ids is None:
            self.seen_site_ids = set()


class MapPlannerNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("map_planner", parameter_overrides=parameter_overrides)
        self._lock = RLock()
        self._mutation_group = MutuallyExclusiveCallbackGroup()
        self._read_group = ReentrantCallbackGroup()
        self._observe_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self._config = load_project_config(self._config_paths())
        self._field = FieldModel.from_config(self._config.field)
        self._field_id = canonical_field_digest(self._config.field)
        self._map = SemanticMap(self._config.field)
        self._planner = RoutePlanner(self._field, self._config.navigation)

        initial_run_id = str(self.get_parameter("initial_run_id").value).strip()
        if not initial_run_id:
            initial_run_id = f"startup-{uuid4()}"
        self._map.begin_run(initial_run_id, self._field_id)

        self._latest_pose: PoseSample | None = None
        self._latest_pose_received_monotonic: float | None = None
        # TF normally trails odometry.  Keep a short FIFO instead of replacing
        # one pending sample on every odometry callback: replacing it meant the
        # requested timestamp was perpetually newer than TF, so pose history
        # could remain empty while all components were otherwise healthy.
        self._pending_odom: deque[tuple[Odometry, float]] = deque(maxlen=128)
        self._last_tf_warning_monotonic = 0.0
        self._latest_detection_received_monotonic: float | None = None
        self._last_detection_frame_valid = False
        self._last_detection_reason = "no canonical detection frame received"
        self._detection_frames_total = 0
        self._detection_frames_with_objects = 0
        self._evidence_reason_counts: Counter[str] = Counter()

        self._pose_history = _PoseHistoryPort(
            max_gap_s=max(0.05, self._config.navigation.timeouts.tf_lookup_s * 2.0)
        )
        self._field_query = _FieldQueryAdapter(
            self._field,
            focal_length_x_px=float(
                self.get_parameter("sign_camera_focal_length_x_px").value
            ),
            principal_point_x_px=float(
                self.get_parameter("sign_camera_principal_point_x_px").value
            ),
            max_detection_range_m=float(
                self.get_parameter("sign_max_detection_range_m").value
            ),
            max_bearing_error_rad=float(
                self.get_parameter("sign_max_bearing_error_rad").value
            ),
            max_depth_error_m=float(
                self.get_parameter("sign_max_depth_error_m").value
            ),
            face_view_tolerance_rad=float(
                self.get_parameter("sign_face_view_tolerance_rad").value
            ),
            min_right_lateral_m=float(
                self.get_parameter("sign_min_right_lateral_m").value
            ),
            score_margin=float(
                self.get_parameter("sign_candidate_score_margin").value
            ),
        )
        self._fusion = SignFusion(
            self._pose_history,
            self._field_query,
            SignFusionConfig(
                required_positive_frames=int(
                    self.get_parameter("sign_required_positive_frames").value
                ),
                positive_window_s=float(
                    self.get_parameter("sign_positive_window_s").value
                ),
                positive_min_span_s=float(
                    self.get_parameter("sign_positive_min_span_s").value
                ),
                max_range_error_m=float(
                    self.get_parameter("sign_max_depth_error_m").value
                ),
                max_mapping_angular_speed_radps=float(
                    self.get_parameter("sign_max_mapping_angular_speed_radps").value
                )
            ),
        )
        self._fusion.begin_run(initial_run_id)
        self._active_observation: _ActiveObservation | None = None
        self._wp06_integrated = True

        self._routes: OrderedDict[str, dict[str, object]] = OrderedDict()
        self._route_cache_limit = int(self.get_parameter("route_cache_size").value)
        self._reported_execution_keys: set[tuple[str, str, int]] = set()
        self._completed_segment_keys: set[tuple[str, str, str, str]] = set()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._map_pub = self.create_publisher(MapState, MAP_STATE_TOPIC, map_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC, 10)

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            sensor_qos,
            callback_group=self._mutation_group,
        )
        self.create_subscription(
            SignDetectionArray,
            str(self.get_parameter("detections_topic").value),
            self._on_detections,
            sensor_qos,
            callback_group=self._mutation_group,
        )

        self.create_service(
            PlanRoute, PLAN_SERVICE, self._handle_plan, callback_group=self._mutation_group
        )
        self.create_service(
            ValidateSegment,
            VALIDATE_SERVICE,
            self._handle_validate,
            callback_group=self._mutation_group,
        )
        self.create_service(
            ReportExecution,
            REPORT_SERVICE,
            self._handle_report,
            callback_group=self._mutation_group,
        )
        self.create_service(
            ResetMap, RESET_SERVICE, self._handle_reset, callback_group=self._mutation_group
        )

        self._observe_server = ActionServer(
            self,
            Observe,
            OBSERVE_ACTION,
            execute_callback=self._execute_observe,
            goal_callback=self._observe_goal,
            cancel_callback=self._observe_cancel,
            callback_group=self._observe_group,
        )

        heartbeat_hz = float(self.get_parameter("heartbeat_hz").value)
        diagnostic_hz = float(self.get_parameter("diagnostic_hz").value)
        self.create_timer(1.0 / heartbeat_hz, self._publish_snapshot, callback_group=self._read_group)
        self.create_timer(1.0 / diagnostic_hz, self._publish_diagnostics, callback_group=self._read_group)
        # TF commonly trails odometry by a few tens of milliseconds.  Do not
        # substitute the latest transform: retain the newest odometry sample and
        # retry its exact capture timestamp asynchronously for at most tf_lookup_s.
        tf_retry_period_s = min(0.02, max(0.005, self._config.navigation.timeouts.tf_lookup_s / 4.0))
        self.create_timer(tf_retry_period_s, self._retry_pending_odom, callback_group=self._mutation_group)

        self._publish_snapshot()
        self._publish_diagnostics()
        self.get_logger().info(
            "WP11B map_planner started with WP06 fusion/observation integration enabled; "
            "readiness still remains fail-closed until geometry and localization are valid."
        )

    # ------------------------------------------------------------------
    # Configuration / readiness
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        share = Path(get_package_share_directory("frob_city_navigation"))
        config = share / "config"
        self.declare_parameter("field_config_path", str(config / "wp11_test_field.yaml"))
        self.declare_parameter("event_config_path", str(config / "wp11_test_event.yaml"))
        self.declare_parameter("perception_config_path", str(config / "wp11_test_perception.yaml"))
        self.declare_parameter("navigation_config_path", str(config / "wp11_test_navigation.yaml"))
        self.declare_parameter("initial_run_id", "wp11-component-run")
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("diagnostic_hz", 1.0)
        self.declare_parameter("route_cache_size", 64)
        self.declare_parameter("odom_topic", ODOM_TOPIC)
        self.declare_parameter("detections_topic", DETECTION_TOPIC)
        # Camera/association values match the calibrated 1280x720 detector.
        # They remain ROS parameters so future camera extrinsic calibration can
        # replace them without changing the field schema.
        self.declare_parameter("sign_camera_focal_length_x_px", 1500.0)
        self.declare_parameter("sign_camera_principal_point_x_px", 640.0)
        self.declare_parameter("sign_max_detection_range_m", 1.80)
        self.declare_parameter("sign_max_bearing_error_rad", 0.50)
        self.declare_parameter("sign_max_depth_error_m", 0.55)
        # A full field site has four possible faces.  Keep only the cardinal
        # face actually looking toward the camera (about +/-49 degrees).
        self.declare_parameter("sign_face_view_tolerance_rad", 0.85)
        self.declare_parameter("sign_min_right_lateral_m", 0.0)
        self.declare_parameter("sign_candidate_score_margin", 0.15)
        self.declare_parameter("sign_required_positive_frames", 2)
        self.declare_parameter("sign_positive_window_s", 6.0)
        self.declare_parameter("sign_positive_min_span_s", 0.12)
        self.declare_parameter("sign_max_mapping_angular_speed_radps", 0.15)

    def _config_paths(self) -> ConfigPaths:
        return ConfigPaths(
            field=Path(str(self.get_parameter("field_config_path").value)),
            event=Path(str(self.get_parameter("event_config_path").value)),
            perception=Path(str(self.get_parameter("perception_config_path").value)),
            navigation=Path(str(self.get_parameter("navigation_config_path").value)),
        )

    def _ros_now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    async def _executor_sleep(self, delay_s: float) -> None:
        """Yield an action coroutine without depending on an asyncio event loop.

        rclpy drives async callbacks through its Executor/Task machinery rather
        than a Python asyncio loop.  Await an executor-owned Future completed by
        a ROS timer so subscriptions, cancellation, and other callbacks can keep
        running while Observe is active, including under SingleThreadedExecutor.
        """
        executor = self.executor
        if executor is None:
            raise RuntimeError("map_planner is not attached to an rclpy executor")

        future = executor.create_future()

        def _wake() -> None:
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(
            max(0.001, float(delay_s)),
            _wake,
            callback_group=self._observe_group,
        )
        try:
            await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    def _pose_fresh(self) -> bool:
        if self._latest_pose is None:
            return False
        age = self._ros_now_s() - self._latest_pose.timestamp_s
        return 0.0 <= age <= self._config.navigation.timeouts.pose_max_age_s

    def _operational_ready(self) -> bool:
        return (
            self._wp06_integrated
            and self._config.field.geometry_calibrated
            and self._pose_fresh()
        )

    def _readiness_reason(self) -> str:
        reasons: list[str] = []
        if not self._wp06_integrated:
            reasons.append("WP06_NOT_INTEGRATED")
        if not self._config.field.geometry_calibrated:
            reasons.append("GEOMETRY_UNCALIBRATED")
        if not self._pose_fresh():
            reasons.append("POSE_STALE_OR_MISSING")
        return ";".join(reasons)

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        """Accept map-frame odometry immediately or defer exact-time TF lookup.

        The robot's TF stream may arrive slightly after odometry.  We never use
        the latest transform as a substitute for capture-time TF.  Instead the
        newest odometry sample is retried asynchronously for the configured
        bounded ``tf_lookup_s`` interval.
        """
        if msg.header.frame_id == self._config.navigation.map_frame:
            try:
                sample = odometry_to_pose_sample(msg, self._config.navigation.map_frame)
            except ValueError as exc:
                self._throttled_tf_warning(f"Rejected map-frame odometry sample: {exc}")
                return
            self._accept_pose_sample(sample)
            return

        with self._lock:
            self._pending_odom.append(
                (msg, time.monotonic() + self._config.navigation.timeouts.tf_lookup_s)
            )
        self._retry_pending_odom()

    def _retry_pending_odom(self) -> None:
        # Drain every transformable/expired sample in timestamp order.  Stop at
        # the first sample that may still become transformable; later samples
        # cannot be ready before it and must not overtake it.
        for _ in range(32):
            with self._lock:
                if not self._pending_odom:
                    return
                msg, deadline = self._pending_odom[0]

            try:
                stamp = Time.from_msg(msg.header.stamp)
                transform = self._tf_buffer.lookup_transform(
                    self._config.navigation.map_frame,
                    self._config.navigation.base_frame,
                    stamp,
                    timeout=Duration(seconds=0.0),
                )
                pose = transform_to_pose2d(transform)
                lin = msg.twist.twist.linear
                linear_speed = math.hypot(float(lin.x), float(lin.y))
                angular_speed = abs(float(msg.twist.twist.angular.z))
                if not math.isfinite(linear_speed) or not math.isfinite(angular_speed):
                    raise ValueError("non-finite odometry velocity")
                sample = PoseSample(
                    timestamp_s=stamp.nanoseconds * 1e-9,
                    pose=pose,
                    linear_speed_mps=linear_speed,
                    angular_speed_radps=angular_speed,
                )
            except TransformException as exc:
                if time.monotonic() < deadline:
                    return
                with self._lock:
                    if self._pending_odom and self._pending_odom[0][0] is msg:
                        self._pending_odom.popleft()
                self._throttled_tf_warning(
                    "TF unavailable at odometry capture time within "
                    f"{self._config.navigation.timeouts.tf_lookup_s:.3f}s; dropping sample: {exc}"
                )
                continue
            except ValueError as exc:
                with self._lock:
                    if self._pending_odom and self._pending_odom[0][0] is msg:
                        self._pending_odom.popleft()
                self._throttled_tf_warning(f"Rejected odometry sample: {exc}")
                continue

            with self._lock:
                if self._pending_odom and self._pending_odom[0][0] is msg:
                    self._pending_odom.popleft()
            self._accept_pose_sample(sample)

    def _accept_pose_sample(self, sample: PoseSample) -> None:
        with self._lock:
            self._latest_pose = sample
            self._latest_pose_received_monotonic = time.monotonic()
            self._pose_history.add(sample)

    def _throttled_tf_warning(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_tf_warning_monotonic >= 1.0:
            self._last_tf_warning_monotonic = now
            self.get_logger().warning(message)

    def _on_detections(self, msg: SignDetectionArray) -> None:
        """Fuse signs continuously; an Observe action additionally qualifies absence."""
        try:
            observations = detection_array_to_core(msg)
        except ValueError as exc:
            with self._lock:
                self._detection_frames_total += 1
                self._evidence_reason_counts["invalid_detection_frame"] += 1
                self._last_detection_frame_valid = False
                self._last_detection_reason = str(exc)
                active = self._active_observation
                if active is not None:
                    active.last_reason = f"invalid_detection_frame:{exc}"
            return

        with self._lock:
            self._detection_frames_total += 1
            if observations:
                self._detection_frames_with_objects += 1
            self._latest_detection_received_monotonic = time.monotonic()
            self._last_detection_frame_valid = bool(msg.frame_valid)
            active = self._active_observation
            if not msg.frame_valid:
                self._last_detection_reason = msg.invalid_reason or "invalid detector frame"
            else:
                self._last_detection_reason = f"canonical frame accepted ({len(observations)} detections)"

        capture_stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose_result = self._pose_history.pose_at(capture_stamp_s)
        if not pose_result.ok or pose_result.sample is None:
            with self._lock:
                self._evidence_reason_counts[f"capture_pose:{pose_result.reason}"] += 1
                self._last_detection_reason = f"capture_pose:{pose_result.reason}"
                if active is not None and self._active_observation is active:
                    active.last_reason = f"capture_pose:{pose_result.reason}"
            return

        capture_s = pose_result.sample.timestamp_s
        wp06_detections = tuple(
            CanonicalDetection(
                sign_class=item.sign_class,
                confidence=item.confidence,
                bbox_xyxy_px=(
                    float(item.bbox.x_min_px),
                    float(item.bbox.y_min_px),
                    float(item.bbox.x_min_px + item.bbox.width_px),
                    float(item.bbox.y_min_px + item.bbox.height_px),
                ),
                range_m=item.range_m if item.range_valid else None,
                range_convention=item.range_convention,
                range_stddev_m=item.range_stddev_m if item.range_valid else None,
                track_id=item.detector_track_id or None,
            )
            for item in observations
        )
        frame = CanonicalDetectionFrame(
            sequence=int(msg.frame_sequence),
            capture_timestamp_s=capture_s,
            frame_id=msg.header.frame_id,
            detections=wp06_detections if msg.frame_valid else (),
        )

        changed = False
        with self._lock:
            # The action runs in a re-entrant callback group, therefore the active
            # session/fusion/map are protected explicitly against finish/reset races.
            active = self._active_observation
            snapshot = self._map.snapshot()
            if active is not None and (
                active.run_id != snapshot.run_id
                or active.map_instance_id != snapshot.map_instance_id
            ):
                active.last_reason = "stale_session"
                return

            context = ObservationContext(
                run_id=snapshot.run_id,
                # Blank observation/approach means passive moving fusion.  The
                # calibrated geometry chooses the physical face and approach;
                # moving frames never qualify an area as sign-free.
                observation_id=active.observation_id if active is not None else "",
                approach_id=active.approach_id if active is not None else "",
                map_instance_id=snapshot.map_instance_id,
                route_revision=str(snapshot.revision),
            )
            evidence = self._fusion.ingest(frame, context) if msg.frame_valid else ()

            if not msg.frame_valid:
                self._evidence_reason_counts["invalid_detector_frame"] += 1
            elif not observations:
                self._evidence_reason_counts["no_detection"] += 1
            for event in evidence:
                self._evidence_reason_counts[event.reason] += 1

            for event in evidence:
                if active is not None and event.site_id:
                    assert active.seen_site_ids is not None
                    active.seen_site_ids.add(event.site_id)
                # Unconfirmed per-frame INCONCLUSIVE events are observation evidence,
                # not completed observation failures.  Apply terminal failure once
                # when the action finishes instead.
                if event.kind.name in {"POSITIVE", "CONFLICT", "PRECAUTIONARY_HOLD"}:
                    update = self._map.apply(event)
                    changed = changed or bool(update.changed)

            mode = "observe" if active is not None else "moving_scan"
            evidence_summary = ",".join(
                sorted({f"{event.kind.name}:{event.reason}" for event in evidence})
            )
            self._last_detection_reason = (
                f"{mode}:{evidence_summary}"
                if evidence_summary
                else f"{mode}:no_detection"
            )

            if active is not None:
                obs_frame = ObservationFrame(
                    run_id=active.run_id,
                    observation_id=active.observation_id,
                    sequence=int(msg.frame_sequence),
                    capture_timestamp_s=capture_s,
                    healthy=bool(msg.frame_valid),
                    covered_sectors=(
                        frozenset({"left", "center", "right"})
                        if msg.frame_valid
                        else frozenset()
                    ),
                    evidence=tuple(evidence),
                )
                progress = active.session.ingest(
                    obs_frame, pose_result.sample, self._ros_now_s()
                )
                if progress.accepted:
                    active.accepted_frames = int(progress.distinct_healthy_frames)
                    if active.first_capture_s is None:
                        active.first_capture_s = capture_s
                    active.last_capture_s = capture_s
                active.last_reason = progress.reason

        if changed:
            self._publish_snapshot()

    # ------------------------------------------------------------------
    # Snapshot / diagnostics
    # ------------------------------------------------------------------

    def _publish_snapshot(self) -> None:
        with self._lock:
            snapshot = self._map.snapshot()
            now = self.get_clock().now().to_msg()
            msg = map_snapshot_to_msg(
                snapshot,
                frame_id=self._config.navigation.map_frame,
                stamp=now,
                ready_override=self._operational_ready(),
                perception_healthy_override=self._wp06_integrated and self._last_detection_frame_valid,
                localization_healthy_override=self._pose_fresh(),
                reason_override=self._readiness_reason(),
                field_to_map=self._config.field.field_to_map,
            )
        self._map_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "frob_city/map_planner"
        status.hardware_id = "map_planner"
        ready = self._operational_ready()
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.WARN
        status.message = "ready" if ready else self._readiness_reason()
        snapshot = self._map.snapshot()
        status.values = [
            KeyValue(key="run_id", value=snapshot.run_id),
            KeyValue(key="map_instance_id", value=snapshot.map_instance_id),
            KeyValue(key="field_id", value=self._field_id),
            KeyValue(key="revision", value=str(snapshot.revision)),
            KeyValue(key="wp06_integrated", value=str(self._wp06_integrated).lower()),
            KeyValue(key="geometry_calibrated", value=str(self._config.field.geometry_calibrated).lower()),
            KeyValue(key="pose_fresh", value=str(self._pose_fresh()).lower()),
            KeyValue(key="detection_state", value=self._last_detection_reason),
            KeyValue(key="detection_frames_total", value=str(self._detection_frames_total)),
            KeyValue(
                key="detection_frames_with_objects",
                value=str(self._detection_frames_with_objects),
            ),
            KeyValue(
                key="mapping.required_frames",
                value=str(self.get_parameter("sign_required_positive_frames").value),
            ),
            KeyValue(
                key="mapping.max_range_m",
                value=str(self.get_parameter("sign_max_detection_range_m").value),
            ),
            KeyValue(
                key="mapping.max_depth_error_m",
                value=str(self.get_parameter("sign_max_depth_error_m").value),
            ),
            KeyValue(
                key="mapping.max_bearing_error_rad",
                value=str(self.get_parameter("sign_max_bearing_error_rad").value),
            ),
            KeyValue(
                key="mapping.max_angular_speed_radps",
                value=str(
                    self.get_parameter("sign_max_mapping_angular_speed_radps").value
                ),
            ),
            KeyValue(key="route_cache_routes", value=str(len(self._routes))),
        ]
        status.values.extend(
            KeyValue(key=f"evidence.{reason}", value=str(count))
            for reason, count in sorted(self._evidence_reason_counts.items())
        )
        msg.status.append(status)
        self._diag_pub.publish(msg)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _fill_not_ready_plan(self, request, response, reason: str):
        snapshot = self._map.snapshot()
        response.status = int(PlanStatus.NOT_READY)
        response.request_id = int(request.request_id)
        response.run_id = snapshot.run_id
        response.map_instance_id = snapshot.map_instance_id
        response.revision = int(snapshot.revision)
        response.route_id = ""
        response.selected_target_id = ""
        response.reaches_requested_target = False
        response.estimated_total_cost_s = 0.0
        response.cost_uses_unknown_gates = False
        response.executable_prefix = []
        response.terminal_observation_id = ""
        response.reason = reason
        if self._latest_pose is not None:
            from .ros_map_conversions import pose2d_to_pose_stamped
            response.start_pose = pose2d_to_pose_stamped(
                self._latest_pose.pose,
                self._config.navigation.map_frame,
                self.get_clock().now().to_msg(),
            )
        return response

    def _handle_plan(self, request, response):
        with self._lock:
            if not self._operational_ready():
                return self._fill_not_ready_plan(request, response, self._readiness_reason())
            assert self._latest_pose is not None
            try:
                core_request = plan_request_from_ros(request)
            except ValueError as exc:
                snapshot = self._map.snapshot()
                result = CorePlanResult(
                    status=PlanStatus.INVALID_REQUEST,
                    request_id=int(request.request_id),
                    run_id=snapshot.run_id,
                    map_instance_id=snapshot.map_instance_id,
                    revision=snapshot.revision,
                    route_id="",
                    selected_target_id="",
                    start_pose=self._latest_pose.pose,
                    reaches_requested_target=False,
                    estimated_total_cost_s=0.0,
                    cost_uses_unknown_gates=False,
                    executable_prefix=(),
                    reason=str(exc),
                )
            else:
                result = self._planner.plan(core_request, self._map.snapshot(), self._latest_pose)
                if result.route_id and result.executable_prefix:
                    self._cache_route(result.route_id, result.executable_prefix)
            return fill_plan_response(
                result,
                response,
                frame_id=self._config.navigation.map_frame,
                stamp=self.get_clock().now().to_msg(),
                field_to_map=self._config.field.field_to_map,
            )

    def _handle_validate(self, request, response):
        with self._lock:
            snapshot = self._map.snapshot()
            checked_pose = self._latest_pose.pose if self._latest_pose is not None else self._field.anchor(self._field.anchor_ids[0]).pose
            if request.run_id != snapshot.run_id or request.map_instance_id != snapshot.map_instance_id:
                result = SegmentValidation(
                    SegmentValidationStatus.STALE_SESSION,
                    snapshot.revision,
                    checked_pose,
                    reason="run/map identity mismatch",
                )
            elif not self._operational_ready() or self._latest_pose is None:
                result = SegmentValidation(
                    SegmentValidationStatus.NOT_READY,
                    snapshot.revision,
                    checked_pose,
                    reason=self._readiness_reason(),
                )
            else:
                segment = self._cached_segment(request.route_id, request.segment_id)
                if segment is None:
                    result = SegmentValidation(
                        SegmentValidationStatus.INVALID,
                        snapshot.revision,
                        self._latest_pose.pose,
                        reason="unknown route/segment id",
                    )
                else:
                    result = validate_segment(segment, snapshot, self._latest_pose, self._field)
            return fill_validation_response(
                result,
                response,
                frame_id=self._config.navigation.map_frame,
                stamp=self.get_clock().now().to_msg(),
            )

    def _handle_report(self, request, response):
        with self._lock:
            snapshot = self._map.snapshot()
            response.revision = int(snapshot.revision)
            if request.run_id != snapshot.run_id or request.map_instance_id != snapshot.map_instance_id:
                response.accepted = False
                response.duplicate = False
                response.reason = "run/map identity mismatch"
                return response

            report_key = (request.run_id, request.navigator_instance_id, int(request.report_sequence))
            if report_key in self._reported_execution_keys:
                response.accepted = True
                response.duplicate = True
                response.reason = "duplicate execution report"
                return response

            segment = self._cached_segment(request.route_id, request.segment_id)
            if segment is None:
                response.accepted = False
                response.duplicate = False
                response.reason = "unknown route/segment id"
                return response
            if not request.navigator_instance_id:
                response.accepted = False
                response.duplicate = False
                response.reason = "navigator_instance_id is empty"
                return response
            try:
                outcome = ExecutionOutcome(int(request.outcome))
                terminal_pose = pose_stamped_to_pose2d(
                    request.measured_terminal_pose, self._config.navigation.map_frame
                )
            except (ValueError, TypeError) as exc:
                response.accepted = False
                response.duplicate = False
                response.reason = str(exc)
                return response

            completion_key = (
                request.run_id,
                request.navigator_instance_id,
                request.route_id,
                request.segment_id,
            )
            if outcome is ExecutionOutcome.COMPLETED and completion_key in self._completed_segment_keys:
                self._reported_execution_keys.add(report_key)
                response.accepted = True
                response.duplicate = True
                response.reason = "segment completion already recorded"
                return response

            kind = {
                ExecutionOutcome.COMPLETED: ExecutionEventKind.SEGMENT_COMPLETED,
                ExecutionOutcome.FAILED: ExecutionEventKind.SEGMENT_FAILED,
                ExecutionOutcome.CANCELED: ExecutionEventKind.SEGMENT_CANCELED,
            }[outcome]
            event = ExecutionEvent(
                run_id=request.run_id,
                navigator_instance_id=request.navigator_instance_id,
                report_sequence=int(request.report_sequence),
                segment_id=request.segment_id,
                kind=kind,
                terminal_pose=terminal_pose,
                timestamp_s=self._ros_now_s(),
                reason=request.reason,
            )
            update = self._map.apply_execution_with_edges(event, segment.traversed_edge_ids)
            if not update.accepted:
                response.accepted = False
                response.duplicate = bool(update.duplicate)
                response.revision = int(update.revision)
                response.reason = update.reason
                return response

            self._reported_execution_keys.add(report_key)
            if outcome is ExecutionOutcome.COMPLETED:
                self._completed_segment_keys.add(completion_key)
            response.accepted = True
            response.duplicate = bool(update.duplicate)
            response.revision = int(update.revision)
            response.reason = update.reason
            if update.changed:
                self._publish_snapshot()
            return response

    def _handle_reset(self, request, response):
        with self._lock:
            if not request.new_run_id.strip():
                response.accepted = False
                response.run_id = self._map.snapshot().run_id
                response.map_instance_id = self._map.map_instance_id
                response.revision = self._map.snapshot().revision
                response.reason = "new_run_id is empty"
                return response
            if request.field_id != self._field_id:
                response.accepted = False
                response.run_id = self._map.snapshot().run_id
                response.map_instance_id = self._map.map_instance_id
                response.revision = self._map.snapshot().revision
                response.reason = "field_id mismatch"
                return response

            snapshot = self._map.begin_run(request.new_run_id, self._field_id)
            self._routes.clear()
            self._reported_execution_keys.clear()
            self._completed_segment_keys.clear()
            self._fusion.begin_run(request.new_run_id)
            self._active_observation = None
            response.accepted = True
            response.run_id = snapshot.run_id
            response.map_instance_id = snapshot.map_instance_id
            response.revision = snapshot.revision
            response.reason = ""
            self._publish_snapshot()
            return response

    # ------------------------------------------------------------------
    # Observation action
    # ------------------------------------------------------------------

    def _observe_goal(self, goal_request) -> GoalResponse:
        if not math.isfinite(float(goal_request.max_duration_s)) or goal_request.max_duration_s <= 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _observe_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _approach_for_observation(self, observation_id: str) -> str:
        region = self._field.observation_region(observation_id)
        approaches = sorted(
            {
                movement.approach_id
                for movement in self._field.movements_from(region.anchor_id)
                if movement.approach_id
            }
        )
        if len(approaches) != 1:
            return ""
        return approaches[0]

    def _alternate_view_available(self, observation_id: str) -> bool:
        region = self._field.observation_region(observation_id)
        return any(
            item.observation_id != observation_id
            for item in self._field.observation_regions_at_anchor(region.anchor_id)
        )

    async def _execute_observe(self, goal_handle):
        request = goal_handle.request
        result = Observe.Result()

        with self._lock:
            snapshot = self._map.snapshot()
            result.revision = int(snapshot.revision)
            result.confirmed_target_ids = []
            result.alternate_view_available = False
            if request.run_id != snapshot.run_id or request.map_instance_id != snapshot.map_instance_id:
                result.outcome = int(ObserveOutcome.STALE_SESSION)
                result.reason = "run/map identity mismatch"
                goal_handle.abort()
                return result
            if request.observation_id not in self._field.observation_ids:
                result.outcome = int(ObserveOutcome.NOT_IN_ZONE)
                result.reason = "unknown observation_id"
                goal_handle.succeed()
                return result
            if self._latest_pose is None or not self._pose_fresh():
                result.outcome = int(ObserveOutcome.INCONCLUSIVE)
                result.reason = "pose stale or missing"
                goal_handle.succeed()
                return result
            if not self._is_in_observation_zone(request.observation_id, self._latest_pose):
                result.outcome = int(ObserveOutcome.NOT_IN_ZONE)
                result.reason = "robot is not stationary inside the approved observation region/heading"
                goal_handle.succeed()
                return result
            approach_id = self._approach_for_observation(request.observation_id)
            if not approach_id:
                result.outcome = int(ObserveOutcome.INCONCLUSIVE)
                result.reason = "observation does not resolve to exactly one controlled approach"
                goal_handle.succeed()
                return result
            if self._active_observation is not None:
                result.outcome = int(ObserveOutcome.INCONCLUSIVE)
                result.reason = "another observation is already active"
                goal_handle.abort()
                return result

            session = ObservationSession()
            session.begin(snapshot.run_id, request.observation_id)
            self._fusion.reset_unconfirmed("new_observation_session")
            active = _ActiveObservation(
                run_id=snapshot.run_id,
                map_instance_id=snapshot.map_instance_id,
                observation_id=request.observation_id,
                approach_id=approach_id,
                started_ros_s=self._ros_now_s(),
                deadline_monotonic=time.monotonic() + float(request.max_duration_s),
                session=session,
            )
            self._active_observation = active

        try:
            while time.monotonic() < active.deadline_monotonic:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.outcome = int(ObserveOutcome.INCONCLUSIVE)
                    result.reason = "observation canceled"
                    return result
                feedback = Observe.Feedback()
                feedback.accepted_frames = int(active.accepted_frames)
                feedback.qualified_window_s = float(
                    0.0
                    if active.first_capture_s is None or active.last_capture_s is None
                    else max(0.0, active.last_capture_s - active.first_capture_s)
                )
                feedback.observation_id = active.observation_id
                feedback.reason = active.last_reason
                goal_handle.publish_feedback(feedback)
                await self._executor_sleep(0.02)

            with self._lock:
                if self._active_observation is not active:
                    result.outcome = int(ObserveOutcome.INCONCLUSIVE)
                    result.reason = "observation session replaced"
                    goal_handle.abort()
                    return result
                outcome = active.session.finish(self._ros_now_s())
                snapshot = self._map.snapshot()
                if snapshot.run_id != active.run_id or snapshot.map_instance_id != active.map_instance_id:
                    result.outcome = int(ObserveOutcome.STALE_SESSION)
                    result.revision = int(snapshot.revision)
                    result.reason = "session changed during observation"
                    goal_handle.abort()
                    return result

                kind_name = outcome.kind.name
                span_s = (
                    0.0
                    if active.first_capture_s is None or active.last_capture_s is None
                    else max(0.0, active.last_capture_s - active.first_capture_s)
                )
                self.get_logger().info(
                    "Observation finished: "
                    f"id={active.observation_id} kind={kind_name} "
                    f"reason={outcome.reason!r} "
                    f"accepted_frames={active.accepted_frames} "
                    f"qualified_window_s={span_s:.3f}"
                )
                if kind_name == "QUALIFIED_CLEAR":
                    from .models import EvidenceEvent, EvidenceKind
                    event = EvidenceEvent(
                        run_id=active.run_id,
                        event_id=f"observe-clear:{active.observation_id}:{uuid4()}",
                        timestamp_s=self._ros_now_s(),
                        kind=EvidenceKind.QUALIFIED_CLEAR,
                        approach_id=active.approach_id,
                        observation_id=active.observation_id,
                        reason=outcome.reason,
                    )
                    update = self._map.apply(event)
                    result.outcome = int(ObserveOutcome.CLEAR_ABSENT)
                    result.reason = (
                        outcome.reason
                        if not update.reason
                        else f"{outcome.reason}; map:{update.reason}"
                    )
                elif kind_name == "POSITIVE":
                    matching = [
                        target.target_id
                        for target in self._map.snapshot().targets
                        if target.site_id in active.seen_site_ids
                        or any(
                            face_key.startswith(f"{target.site_id}/")
                            for face_key in active.seen_site_ids
                        )
                    ]
                    result.confirmed_target_ids = sorted(matching)
                    result.outcome = int(
                        ObserveOutcome.TARGET_FOUND
                        if matching
                        else ObserveOutcome.APPROACH_RESOLVED
                    )
                    result.reason = outcome.reason
                elif kind_name == "CONFLICT":
                    result.outcome = int(ObserveOutcome.CONFLICT)
                    result.reason = outcome.reason
                else:
                    from .models import EvidenceEvent, EvidenceKind
                    event = EvidenceEvent(
                        run_id=active.run_id,
                        event_id=f"observe-inconclusive:{active.observation_id}:{uuid4()}",
                        timestamp_s=self._ros_now_s(),
                        kind=EvidenceKind.INCONCLUSIVE,
                        approach_id=active.approach_id,
                        observation_id=active.observation_id,
                        reason=outcome.reason,
                    )
                    update = self._map.apply(event)
                    result.outcome = int(ObserveOutcome.INCONCLUSIVE)
                    result.reason = (
                        outcome.reason
                        if not update.reason
                        else f"{outcome.reason}; map:{update.reason}"
                    )

                result.revision = int(self._map.snapshot().revision)
                result.alternate_view_available = self._alternate_view_available(active.observation_id)
                self._publish_snapshot()
                goal_handle.succeed()
                return result
        finally:
            with self._lock:
                if self._active_observation is active:
                    self._active_observation = None

    def _is_in_observation_zone(self, observation_id: str, sample: PoseSample) -> bool:
        from shapely.geometry import Point, Polygon

        region = self._field.observation_region(observation_id)
        # Observation geometry is authored in the conceptual field frame,
        # while localization samples are expressed in the Nav2 map frame.
        # Identity transforms in unit fixtures used to hide this mismatch.
        field_pose = self._field.map_to_field_pose(sample.pose)
        polygon = Polygon([(p.x_m, p.y_m) for p in region.polygon.points])
        point = Point(field_pose.x_m, field_pose.y_m)
        if not polygon.buffer(1e-9).covers(point):
            return False
        heading_error = math.atan2(
            math.sin(field_pose.yaw_rad - region.nominal_heading_rad),
            math.cos(field_pose.yaw_rad - region.nominal_heading_rad),
        )
        if abs(heading_error) > region.heading_tolerance_rad:
            return False
        return (
            sample.linear_speed_mps <= self._config.navigation.stationary_linear_mps
            and sample.angular_speed_radps <= self._config.navigation.stationary_angular_radps
        )

    # ------------------------------------------------------------------
    # Route cache
    # ------------------------------------------------------------------

    def _cache_route(self, route_id: str, segments) -> None:
        self._routes[route_id] = {segment.segment_id: segment for segment in segments}
        self._routes.move_to_end(route_id)
        while len(self._routes) > self._route_cache_limit:
            self._routes.popitem(last=False)

    def _cached_segment(self, route_id: str, segment_id: str):
        route = self._routes.get(route_id)
        if route is None:
            return None
        return route.get(segment_id)

    def destroy_node(self):
        self._observe_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MapPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # SIGINT may already have shut the context down through rclpy's signal
        # handler.  Avoid a second shutdown call and the resulting RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
