"""ROS-independent immutable core contracts for Frob City.

All physical units in this module are SI unless the field name explicitly says
otherwise: metres, radians, seconds, metres/second and radians/second.

This module MUST remain importable on a machine with no ROS installation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
from pathlib import Path
from typing import Optional, Tuple


CONTRACT_VERSION = "frob-city-v2-core-1"


class SignClass(IntEnum):
    UNKNOWN = 0
    STRAIGHT_ONLY = 1
    LEFT_ONLY = 2
    RIGHT_ONLY = 3
    NO_LEFT = 4
    NO_RIGHT = 5
    BUS_STOP = 6
    PARKING = 7
    HAZARD_MARKER = 8


class RangeConvention(IntEnum):
    RANGE_UNKNOWN = 0
    OPTICAL_Z = 1
    RAY_RANGE = 2


class MovementMask(IntFlag):
    NONE = 0
    STRAIGHT = 1
    LEFT = 2
    RIGHT = 4


ALL_MOVEMENTS = MovementMask.STRAIGHT | MovementMask.LEFT | MovementMask.RIGHT


class RuleDecisionCode(IntEnum):
    ALLOWED = 0
    UNRESOLVED_APPROACH = 1
    APPROACH_CONFLICT = 2
    APPROACH_BLOCKED = 3
    UNKNOWN_CONFIRMED_SIGN = 4
    RULE_CONFLICT_OR_NO_LEGAL_EXIT = 5


@dataclass(frozen=True, slots=True)
class TrafficPolicy:
    """Event-level movement policy before sign/topology intersection.

    Version 1 deliberately has no U-turn bit. `permitted` therefore contains
    only STRAIGHT/LEFT/RIGHT.
    """

    permitted: MovementMask = ALL_MOVEMENTS


@dataclass(frozen=True, slots=True)
class RuleDecision:
    allowed: MovementMask
    code: RuleDecisionCode
    reason: str
    applied_directional_signs: Tuple[SignClass, ...] = ()
    nondirectional_signs: Tuple[SignClass, ...] = ()


class ApproachResolution(IntEnum):
    UNKNOWN = 0
    OBSERVING = 1
    RESOLVED_CLEAR = 2
    RESOLVED_SIGNED = 3
    CONFLICT = 4
    BLOCKED = 5


class TargetResolution(IntEnum):
    PROVISIONAL = 0
    CONFIRMED = 1
    CONFLICT = 2


class EdgeBlockState(IntEnum):
    OPEN = 0
    TEMP_BLOCKED = 1
    HAZARD_BLOCKED = 2


class CoverageResolution(IntEnum):
    UNKNOWN = 0
    COMPLETE = 1
    INCONCLUSIVE = 2
    UNREACHABLE_NOW = 3


class SegmentKind(IntEnum):
    CORRIDOR = 0
    JUNCTION = 1
    SERVICE_APPROACH = 2
    OBSERVATION_ADJUST = 3


class TerminalKind(IntEnum):
    HOLD = 0
    OBSERVE = 1
    BUS_SERVICE = 2
    PARK = 3


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


class SegmentValidationStatus(IntEnum):
    VALID = 0
    INVALID = 1
    NEED_OBSERVATION = 2
    NOT_READY = 3
    STALE_SESSION = 4


class ExecutionOutcome(IntEnum):
    COMPLETED = 0
    FAILED = 1
    CANCELED = 2


class ObserveOutcome(IntEnum):
    APPROACH_RESOLVED = 0
    TARGET_FOUND = 1
    CLEAR_ABSENT = 2
    INCONCLUSIVE = 3
    CONFLICT = 4
    NOT_IN_ZONE = 5
    STALE_SESSION = 6


class MissionCommand(IntEnum):
    START = 0
    PAUSE = 1
    RESUME = 2
    ABORT = 3
    RESET = 4


class EvidenceKind(IntEnum):
    POSITIVE = 0
    QUALIFIED_CLEAR = 1
    INCONCLUSIVE = 2
    CONFLICT = 3
    PRECAUTIONARY_HOLD = 4


class ExecutionEventKind(IntEnum):
    SEGMENT_COMPLETED = 0
    SEGMENT_FAILED = 1
    SEGMENT_CANCELED = 2


class GuardDecisionKind(IntEnum):
    ALLOW = 0
    HOLD = 1
    FAULT = 2


@dataclass(frozen=True, slots=True)
class Point2D:
    x_m: float
    y_m: float


@dataclass(frozen=True, slots=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True, slots=True)
class Polygon2D:
    points: Tuple[Point2D, ...]


@dataclass(frozen=True, slots=True)
class MetricPath:
    poses: Tuple[Pose2D, ...]


@dataclass(frozen=True, slots=True)
class HealthSample:
    timestamp_s: float
    healthy: bool
    reason: str = ""


@dataclass(frozen=True, slots=True)
class PoseSample:
    timestamp_s: float
    pose: Pose2D
    linear_speed_mps: float = 0.0
    angular_speed_radps: float = 0.0
    covariance_xy_m2: Optional[float] = None
    covariance_yaw_rad2: Optional[float] = None


@dataclass(frozen=True, slots=True)
class BoundingBox:
    x_min_px: int
    y_min_px: int
    width_px: int
    height_px: int


@dataclass(frozen=True, slots=True)
class DetectionObservation:
    timestamp_s: float
    detector_instance_id: str
    frame_sequence: int
    optical_frame: str
    sign_class: SignClass
    confidence: float
    bbox: BoundingBox
    range_valid: bool
    range_convention: RangeConvention
    range_m: Optional[float]
    range_stddev_m: Optional[float]
    detector_track_id: str = ""


@dataclass(frozen=True, slots=True)
class DirectedAnchor:
    anchor_id: str
    cell_id: str
    pose: Pose2D


@dataclass(frozen=True, slots=True)
class DirectedEdge:
    edge_id: str
    from_anchor_id: str
    to_anchor_id: str
    movement: MovementMask
    path: MetricPath
    permitted_region_ids: Tuple[str, ...]
    approach_id: str = ""
    commitment_gate_id: Optional[str] = None


@dataclass(frozen=True, slots=True)
class ObservationRegion:
    observation_id: str
    polygon: Polygon2D
    anchor_id: str
    nominal_heading_rad: float
    heading_tolerance_rad: float


@dataclass(frozen=True, slots=True)
class Gate:
    gate_id: str
    pose: Pose2D


@dataclass(frozen=True, slots=True)
class SignFace:
    face_id: str
    yaw_rad: float
    controlled_approach_id: str = ""


@dataclass(frozen=True, slots=True)
class SignSite:
    site_id: str
    position: Point2D
    faces: Tuple[SignFace, ...]


@dataclass(frozen=True, slots=True)
class GeometryMargins:
    localization_margin_m: float
    tracking_margin_m: float
    clearance_margin_m: float


@dataclass(frozen=True, slots=True)
class FieldTransform:
    x_m: float
    y_m: float
    yaw_rad: float
    valid: bool


@dataclass(frozen=True, slots=True)
class NamedPolygon:
    polygon_id: str
    polygon: Polygon2D


@dataclass(frozen=True, slots=True)
class DirectedAnchorConfig:
    anchor_id: str
    cell_id: str
    pose: Pose2D


@dataclass(frozen=True, slots=True)
class DirectedEdgeConfig:
    edge_id: str
    from_anchor_id: str
    to_anchor_id: str
    movement: MovementMask
    path: MetricPath
    permitted_region_ids: Tuple[str, ...]
    approach_id: str = ""
    commitment_gate_id: Optional[str] = None


@dataclass(frozen=True, slots=True)
class ObservationRegionConfig:
    observation_id: str
    polygon: Polygon2D
    anchor_id: str
    nominal_heading_rad: float
    heading_tolerance_rad: float


@dataclass(frozen=True, slots=True)
class GateConfig:
    gate_id: str
    pose: Pose2D


@dataclass(frozen=True, slots=True)
class SignFaceConfig:
    face_id: str
    yaw_rad: float
    controlled_approach_id: str = ""


@dataclass(frozen=True, slots=True)
class SignSiteConfig:
    site_id: str
    position: Point2D
    faces: Tuple[SignFaceConfig, ...]


@dataclass(frozen=True, slots=True)
class FieldConfig:
    schema_version: int
    frame_id: str
    width_m: float
    height_m: float
    geometry_calibrated: bool
    field_to_map: FieldTransform
    footprint: Polygon2D
    drivable_regions: Tuple[NamedPolygon, ...]
    platforms: Tuple[NamedPolygon, ...]
    anchors: Tuple[DirectedAnchorConfig, ...]
    edges: Tuple[DirectedEdgeConfig, ...]
    observation_regions: Tuple[ObservationRegionConfig, ...]
    sign_sites: Tuple[SignSiteConfig, ...]
    gates: Tuple[GateConfig, ...]


@dataclass(frozen=True, slots=True)
class StartConfig:
    cell_id: Optional[str]
    map_pose: Optional[Pose2D]
    outgoing_edge_id: Optional[str]


@dataclass(frozen=True, slots=True)
class ParkingPriorConfig:
    enabled: bool
    mode: str
    observation_id: Optional[str]


@dataclass(frozen=True, slots=True)
class EventConfig:
    schema_version: int
    required_unique_bus_stops: int
    bus_stop_dwell_s: float
    parking_settle_time_s: float
    semantic_bootstrap: str
    allow_reverse: bool
    allow_uturn: bool
    auto_start: bool
    official_attempt_limit_s: Optional[float]
    start: StartConfig
    parking_prior: ParkingPriorConfig
    event_inputs_confirmed: bool
    geometry_calibrated: bool


@dataclass(frozen=True, slots=True)
class PerceptionConfig:
    schema_version: int
    camera_frame: str
    calibration_id: str
    label_map: Tuple[Tuple[str, SignClass], ...]
    confidence_thresholds: Tuple[Tuple[SignClass, float], ...]


@dataclass(frozen=True, slots=True)
class NavigationTimeouts:
    permission_max_age_s: float
    map_max_age_s: float
    pose_max_age_s: float
    detection_max_age_s: float
    tf_lookup_s: float
    observation_action_s: float
    action_accept_s: float
    action_cancel_s: float
    no_progress_s: float


@dataclass(frozen=True, slots=True)
class NavigationConfig:
    schema_version: int
    map_frame: str
    odom_frame: str
    base_frame: str
    max_speed_mps: float
    max_yaw_rate_radps: float
    xy_tolerance_m: float
    yaw_tolerance_rad: float
    stationary_linear_mps: float
    stationary_angular_radps: float
    observation_attempts: int
    segment_retry_budget: int
    timeouts: NavigationTimeouts


@dataclass(frozen=True, slots=True)
class ConfigPaths:
    field: Path
    event: Path
    perception: Path
    navigation: Path


@dataclass(frozen=True, slots=True)
class ProjectConfig:
    contract_version: str
    field: FieldConfig
    event: EventConfig
    perception: PerceptionConfig
    navigation: NavigationConfig


@dataclass(frozen=True, slots=True)
class ValidationIssue:
    code: str
    path: str
    message: str


@dataclass(frozen=True, slots=True)
class EvidenceEvent:
    run_id: str
    event_id: str
    timestamp_s: float
    kind: EvidenceKind
    site_id: str = ""
    approach_id: str = ""
    sign_class: SignClass = SignClass.UNKNOWN
    confidence: float = 0.0
    observation_id: str = ""
    reason: str = ""


@dataclass(frozen=True, slots=True)
class ExecutionEvent:
    run_id: str
    navigator_instance_id: str
    report_sequence: int
    segment_id: str
    kind: ExecutionEventKind
    terminal_pose: Pose2D
    timestamp_s: float
    reason: str = ""


@dataclass(frozen=True, slots=True)
class ApproachRecord:
    approach_id: str
    version: int
    state: ApproachResolution
    confirmed_signs: frozenset[SignClass]
    allowed_movements: MovementMask
    last_qualified_observation_s: Optional[float]
    precautionary_hold: bool
    reason: str = ""


@dataclass(frozen=True, slots=True)
class TargetRecord:
    target_id: str
    version: int
    kind: SignClass
    state: TargetResolution
    site_id: str
    face_id: str
    directed_road_id: str
    service_anchor_id: str
    sign_pose: Pose2D
    geometry_valid: bool
    service_pose: Optional[Pose2D]
    terminal_limit: Optional[Pose2D]
    xy_tolerance_m: float
    yaw_tolerance_rad: float
    front_clearance_min_m: float
    last_seen_s: float


@dataclass(frozen=True, slots=True)
class KnownSignRecord:
    """A confirmed physical sign placement, independent of route semantics."""

    site_id: str
    face_id: str
    approach_id: str
    sign_class: SignClass
    version: int
    last_seen_s: float
    confidence: float


@dataclass(frozen=True, slots=True)
class EdgeRecord:
    edge_id: str
    version: int
    state: EdgeBlockState
    completed_traversals: int
    next_recheck_after_s: Optional[float]
    reason: str = ""


@dataclass(frozen=True, slots=True)
class CoverageRecord:
    coverage_id: str
    version: int
    state: CoverageResolution
    failed_attempts: int
    last_qualified_observation_s: Optional[float]
    reason: str = ""


@dataclass(frozen=True, slots=True)
class MapSnapshot:
    run_id: str
    map_instance_id: str
    field_id: str
    revision: int
    ready: bool
    perception_healthy: bool
    localization_healthy: bool
    precautionary_hold_ids: Tuple[str, ...]
    approaches: Tuple[ApproachRecord, ...]
    targets: Tuple[TargetRecord, ...]
    edges: Tuple[EdgeRecord, ...]
    coverage: Tuple[CoverageRecord, ...]
    reason: str = ""
    known_signs: Tuple[KnownSignRecord, ...] = ()


@dataclass(frozen=True, slots=True)
class RouteDependency:
    dependency_id: str
    version: int


@dataclass(frozen=True, slots=True)
class RouteSegment:
    segment_id: str
    kind: SegmentKind
    from_anchor_id: str
    to_anchor_id: str
    approach_id: str
    selected_movement: MovementMask
    path: MetricPath
    allowed_regions: Tuple[Polygon2D, ...]
    dependencies: Tuple[RouteDependency, ...]
    traversed_edge_ids: Tuple[str, ...]
    terminal_kind: TerminalKind
    terminal_id: str
    terminal_limit: Pose2D
    commitment_gate: Optional[Pose2D]
    max_speed_mps: float
    max_yaw_rate_radps: float
    xy_tolerance_m: float
    yaw_tolerance_rad: float
    estimated_duration_s: float


@dataclass(frozen=True, slots=True)
class CorePlanRequest:
    run_id: str
    map_instance_id: str
    request_id: int
    minimum_revision: int
    goal_kind: PlanGoalKind
    target_id: str = ""
    exclude_target_ids: Tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class CorePlanResult:
    status: PlanStatus
    request_id: int
    run_id: str
    map_instance_id: str
    revision: int
    route_id: str
    selected_target_id: str
    start_pose: Pose2D
    reaches_requested_target: bool
    estimated_total_cost_s: float
    cost_uses_unknown_gates: bool
    executable_prefix: Tuple[RouteSegment, ...]
    terminal_observation_id: str = ""
    reason: str = ""


@dataclass(frozen=True, slots=True)
class SegmentValidation:
    status: SegmentValidationStatus
    current_revision: int
    checked_robot_pose: Pose2D
    observation_id: str = ""
    reason: str = ""


@dataclass(frozen=True, slots=True)
class MissionEvent:
    timestamp_s: float
    command: MissionCommand
    reason: str = ""


@dataclass(frozen=True, slots=True)
class MissionStateRecord:
    run_id: str
    phase: MissionPhase
    execution_state: ExecutionState
    objective_id: str
    route_id: str
    active_segment_id: str
    goal_generation: int
    required_unique_bus_stops: int
    serviced_stop_ids: Tuple[str, ...]
    current_dwell_s: float
    parking_prior_state: ParkingPriorState
    attempt_tainted: bool
    reason: str = ""


@dataclass(frozen=True, slots=True)
class ServiceReceipt:
    run_id: str
    target_id: str
    accepted_at_s: float
    stationary_dwell_s: float
    pose: Pose2D


@dataclass(frozen=True, slots=True)
class GuardInput:
    now_s: float
    permission_received_s: Optional[float]
    permission_sequence: int
    raw_command_received_s: Optional[float]
    localization_received_s: Optional[float]
    measured_pose: Optional[Pose2D]
    measured_linear_speed_mps: Optional[float]
    measured_angular_speed_radps: Optional[float]
    emergency_stop: bool


@dataclass(frozen=True, slots=True)
class GuardDecision:
    kind: GuardDecisionKind
    motion_allowed: bool
    reason: str


# ---------------------------------------------------------------------------
# WP04 geometry / projection contracts
# ---------------------------------------------------------------------------

class ProjectionStatus(IntEnum):
    MATCHED = 0
    AMBIGUOUS = 1
    OUT_OF_FIELD = 2
    NO_MATCH = 3


class GeometryStatus(IntEnum):
    VALID = 0
    OUTSIDE_PERMITTED = 1
    INVALID_INPUT = 2


class ServiceGeometryStatus(IntEnum):
    VALID = 0
    UNKNOWN_ROAD = 1
    SIGN_NOT_PROJECTABLE = 2
    INSUFFICIENT_UPSTREAM_ROOM = 3


@dataclass(frozen=True, slots=True)
class MotionPrimitive:
    movement_id: str
    from_anchor_id: str
    to_anchor_id: str
    movement: MovementMask
    path: MetricPath
    permitted_regions: Tuple[Polygon2D, ...]
    approach_id: str = ""
    commitment_gate: Optional[Gate] = None


@dataclass(frozen=True, slots=True)
class PoseProjection:
    movement_id: str
    station_m: float
    path_length_m: float
    projected_pose: Pose2D
    cross_track_error_m: float
    heading_error_rad: float


@dataclass(frozen=True, slots=True)
class ProjectionResult:
    status: ProjectionStatus
    projection: Optional[PoseProjection]
    candidate_movement_ids: Tuple[str, ...] = ()
    reason: str = ""


@dataclass(frozen=True, slots=True)
class GeometryResult:
    status: GeometryStatus
    valid: bool
    swept_area_m2: float
    min_clearance_m: float
    reason: str = ""


@dataclass(frozen=True, slots=True)
class ServiceGeometryResult:
    status: ServiceGeometryStatus
    valid: bool
    directed_road_id: str
    sign_station_m: float
    service_station_m: float
    service_pose: Optional[Pose2D]
    terminal_limit: Optional[Pose2D]
    front_extent_m: float
    calibration_required: bool
    reason: str = ""

# ---------------------------------------------------------------------------
# WP07 semantic-map result contracts
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class MapUpdate:
    accepted: bool
    duplicate: bool
    changed: bool
    revision: int
    changed_dependency_ids: Tuple[str, ...]
    reason: str = ""


@dataclass(frozen=True, slots=True)
class DependencyCheck:
    matches: bool
    mismatched_ids: Tuple[str, ...]
    unknown_ids: Tuple[str, ...]
    reason: str = ""

# ---------------------------------------------------------------------------
# WP08 planner / path-builder contracts
# ---------------------------------------------------------------------------

class SegmentBuildStatus(IntEnum):
    BUILT = 0
    INVALID_TRANSITION = 1
    GEOMETRY_INVALID = 2
    DEPENDENCY_INVALID = 3
    START_JOIN_INVALID = 4
    TERMINAL_LIMIT_VIOLATION = 5


@dataclass(frozen=True, slots=True)
class GraphState:
    """Directed planner state: physical anchor plus edge used to arrive there."""

    anchor_id: str
    incoming_edge_id: str = ""


@dataclass(frozen=True, slots=True)
class GraphTransition:
    transition_id: str
    from_state: GraphState
    to_state: GraphState
    edge_id: str
    movement: MovementMask
    path: MetricPath
    permitted_regions: Tuple[Polygon2D, ...]
    approach_id: str
    executable: bool
    provisional: bool
    dependencies: Tuple[RouteDependency, ...]
    observation_id: str = ""
    target_id: str = ""
    segment_kind: SegmentKind = SegmentKind.CORRIDOR
    terminal_kind: TerminalKind = TerminalKind.HOLD
    terminal_limit: Optional[Pose2D] = None
    commitment_gate: Optional[Pose2D] = None
    estimated_cost_s: float = 0.0


@dataclass(frozen=True, slots=True)
class SegmentBuildResult:
    status: SegmentBuildStatus
    segment: Optional[RouteSegment]
    reason: str = ""
