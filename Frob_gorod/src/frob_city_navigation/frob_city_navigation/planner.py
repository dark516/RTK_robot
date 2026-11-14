"""Deterministic directed route planner for Frob City WP08."""

from __future__ import annotations

import hashlib
import heapq
import json
import math
from dataclasses import replace

from .field_model import FieldModel
from .geometry import path_length, pose_at_station, project_point_to_path
from .graph import DirectedGraph, build_transitions
from .models import (
    ApproachResolution,
    CorePlanRequest,
    CorePlanResult,
    CoverageResolution,
    GeometryMargins,
    GraphState,
    GraphTransition,
    MapSnapshot,
    MetricPath,
    MovementMask,
    NavigationConfig,
    PlanGoalKind,
    PlanStatus,
    Point2D,
    Pose2D,
    PoseSample,
    RouteDependency,
    SegmentKind,
    SignClass,
    TargetResolution,
    TerminalKind,
)
from .path_builder import PathBuilder
from .pose_projection import project_pose

_EPS = 1e-9


def _route_id(request: CorePlanRequest, snapshot: MapSnapshot, transitions: tuple[GraphTransition, ...]) -> str:
    data = {
        "run": snapshot.run_id,
        "map": snapshot.map_instance_id,
        "rev": snapshot.revision,
        "req": request.request_id,
        "goal": int(request.goal_kind),
        "target": request.target_id,
        "edges": [t.transition_id for t in transitions],
    }
    digest = hashlib.sha256(json.dumps(data, sort_keys=True, separators=(",", ":")).encode()).hexdigest()[:20]
    return f"route-{digest}"


def _index(snapshot: MapSnapshot):
    return (
        {x.approach_id: x for x in snapshot.approaches},
        {x.edge_id: x for x in snapshot.edges},
        {x.coverage_id: x for x in snapshot.coverage},
        {x.target_id: x for x in snapshot.targets},
    )


def _nearest_anchor(field: FieldModel, pose: Pose2D) -> str:
    distances = []
    for aid in field.anchor_ids:
        a = field.anchor(aid)
        distances.append((math.hypot(pose.x_m - a.pose.x_m, pose.y_m - a.pose.y_m), aid))
    if not distances:
        raise ValueError("field has no anchors")
    return min(distances)[1]


def _start_state(field: FieldModel, pose: PoseSample) -> GraphState:
    projection = project_pose(field, pose, None)
    if projection.projection is not None:
        primitive = field.movement_primitive(projection.projection.movement_id)
        # Keep the upstream anchor while travelling inside a primitive.  The
        # first segment is trimmed to the measured pose by PathBuilder.
        # Preserve the matched directed primitive as a heading constraint.
        # At an intersection this prevents the first plan from selecting a
        # different startup spoke that would require an in-place 90/180 turn.
        incoming = (
            primitive.movement_id
            if primitive.movement_id.startswith(("START_", "MANEUVER_"))
            else ""
        )
        return GraphState(primitive.from_anchor_id, incoming)
    anchor_id = _nearest_anchor(field, field.map_to_field_pose(pose.pose))
    # At an atomic road midpoint several maneuver endpoints overlap and pose
    # projection may be intentionally ambiguous.  Recover only the outgoing
    # branch tangent to the measured heading.
    if any(mid.startswith("START_") for mid in field.movement_ids):
        pose_field = field.map_to_field_pose(pose.pose)
        candidates = []
        for primitive in field.movements_from(anchor_id):
            yaw = primitive.path.poses[0].yaw_rad
            error = abs(math.atan2(
                math.sin(yaw - pose_field.yaw_rad),
                math.cos(yaw - pose_field.yaw_rad),
            ))
            if error <= math.radians(60.0):
                candidates.append((error, primitive.movement_id))
        if candidates:
            return GraphState(anchor_id, min(candidates)[1])
    return GraphState(anchor_id, "")


def _path_prefix_to_station(path: MetricPath, station_m: float) -> MetricPath:
    if station_m <= 0.0:
        p = pose_at_station(path, 0.0)
        return MetricPath((p, p))
    out = [path.poses[0]]
    cumulative = 0.0
    for a, b in zip(path.poses, path.poses[1:]):
        seg = math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)
        if seg <= _EPS:
            continue
        if cumulative + seg < station_m - _EPS:
            out.append(b)
            cumulative += seg
            continue
        out.append(pose_at_station(path, station_m))
        break
    if len(out) < 2:
        out.append(pose_at_station(path, station_m))
    return MetricPath(tuple(out))


def _target_transition(
    field: FieldModel,
    snapshot: MapSnapshot,
    navigation: NavigationConfig,
    target_id: str,
) -> GraphTransition | None:
    approaches, edges, _, targets = _index(snapshot)
    target = targets.get(target_id)
    if target is None or target.state is not TargetResolution.CONFIRMED:
        return None
    if not target.geometry_valid or target.service_pose is None or target.terminal_limit is None:
        return None
    try:
        primitive = field.movement_primitive(target.directed_road_id)
    except KeyError:
        return None

    projection = project_point_to_path(
        Point2D(target.service_pose.x_m, target.service_pose.y_m), primitive.path
    )
    path = _path_prefix_to_station(primitive.path, projection.station_m)
    edge_rec = edges.get(primitive.movement_id)
    edge_version = 0 if edge_rec is None else edge_rec.version
    edge_open = edge_rec is None or int(edge_rec.state) == 0
    deps = [RouteDependency(f"edge:{primitive.movement_id}", edge_version)]

    executable = edge_open
    provisional = edge_open
    observation_id = ""
    if primitive.approach_id:
        approach = approaches.get(primitive.approach_id)
        if approach is None:
            deps.append(RouteDependency(f"approach:{primitive.approach_id}", 0))
            executable = False
            provisional = edge_open
        else:
            deps.append(RouteDependency(f"approach:{primitive.approach_id}", approach.version))
            resolved = approach.state in (
                ApproachResolution.RESOLVED_CLEAR,
                ApproachResolution.RESOLVED_SIGNED,
            )
            legal = bool(approach.allowed_movements & primitive.movement)
            if not (resolved and legal and not approach.precautionary_hold and edge_open):
                executable = False
                provisional = edge_open and (
                    approach.state in (ApproachResolution.UNKNOWN, ApproachResolution.OBSERVING)
                    or approach.precautionary_hold
                )
        regs = field.observation_regions_at_anchor(primitive.from_anchor_id)
        observation_id = regs[0].observation_id if regs else ""

    deps.append(RouteDependency(f"target:{target.target_id}", target.version))
    terminal_kind = TerminalKind.BUS_SERVICE if target.kind is SignClass.BUS_STOP else TerminalKind.PARK
    cost = path_length(path) / max(navigation.max_speed_mps, 1e-6)
    if provisional and not executable:
        cost += navigation.timeouts.observation_action_s
    return GraphTransition(
        transition_id=f"service:{target.target_id}",
        from_state=GraphState(primitive.from_anchor_id, ""),
        to_state=GraphState(f"target:{target.target_id}", primitive.movement_id),
        edge_id=primitive.movement_id,
        movement=primitive.movement,
        path=path,
        permitted_regions=primitive.permitted_regions,
        approach_id=primitive.approach_id,
        executable=executable,
        provisional=provisional,
        dependencies=tuple(deps),
        observation_id=observation_id,
        target_id=target.target_id,
        segment_kind=SegmentKind.SERVICE_APPROACH,
        terminal_kind=terminal_kind,
        terminal_limit=target.terminal_limit,
        commitment_gate=(primitive.commitment_gate.pose if primitive.commitment_gate else None),
        estimated_cost_s=cost,
    )


def _add_transition(graph: DirectedGraph, transition: GraphTransition) -> DirectedGraph:
    return DirectedGraph(tuple(sorted((*graph.transitions, transition), key=lambda x: x.transition_id)))


def _search(
    graph: DirectedGraph,
    start: GraphState,
    goal_anchor: str,
    *,
    provisional: bool,
) -> tuple[GraphTransition, ...] | None:
    """Dijkstra with stable tie breaks: repeats, turns, then transition IDs."""
    counter = 0
    heap: list[tuple[float, int, int, tuple[str, ...], int, GraphState, tuple[GraphTransition, ...]]] = []
    heapq.heappush(heap, (0.0, 0, 0, (), counter, start, ()))
    best: dict[GraphState, tuple[float, int, int, tuple[str, ...]]] = {}

    while heap:
        cost, repeats, turns, ids, _, state, path = heapq.heappop(heap)
        rank = (cost, repeats, turns, ids)
        old = best.get(state)
        if old is not None and old <= rank:
            continue
        best[state] = rank
        if state.anchor_id == goal_anchor:
            return path

        used_edges = [t.edge_id for t in path if t.edge_id]
        for transition in graph.outgoing(state, provisional=provisional):
            next_state = GraphState(transition.to_state.anchor_id, transition.edge_id)
            next_repeats = repeats + used_edges.count(transition.edge_id)
            next_turns = turns + (1 if transition.movement in (MovementMask.LEFT, MovementMask.RIGHT) else 0)
            next_ids = (*ids, transition.transition_id)
            counter += 1
            heapq.heappush(
                heap,
                (
                    cost + transition.estimated_cost_s,
                    next_repeats,
                    next_turns,
                    next_ids,
                    counter,
                    next_state,
                    (*path, transition),
                ),
            )
    return None


def _reachable_cost(graph: DirectedGraph, start: GraphState, anchor: str) -> tuple[float, tuple[GraphTransition, ...]] | None:
    path = _search(graph, start, anchor, provisional=False)
    if path is None:
        return None
    return sum(t.estimated_cost_s for t in path), path


class RoutePlanner:
    def __init__(
        self,
        field: FieldModel,
        navigation: NavigationConfig,
        margins: GeometryMargins | None = None,
    ):
        self._field = field
        self._navigation = navigation
        self._builder = PathBuilder(field, navigation, margins)

    def plan(
        self,
        request: CorePlanRequest,
        snapshot: MapSnapshot,
        pose: PoseSample,
    ) -> CorePlanResult:
        base = dict(
            request_id=request.request_id,
            run_id=snapshot.run_id,
            map_instance_id=snapshot.map_instance_id,
            revision=snapshot.revision,
            start_pose=pose.pose,
        )
        if not snapshot.ready:
            return CorePlanResult(PlanStatus.NOT_READY, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="map not ready")
        if request.run_id != snapshot.run_id or request.map_instance_id != snapshot.map_instance_id:
            return CorePlanResult(PlanStatus.STALE_SESSION, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="run/map identity mismatch")
        if request.minimum_revision > snapshot.revision:
            return CorePlanResult(PlanStatus.NOT_READY, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="snapshot older than requested minimum revision")

        start = _start_state(self._field, pose)
        graph = DirectedGraph(build_transitions(self._field, snapshot, self._navigation))

        if request.goal_kind is PlanGoalKind.TARGET:
            return self._plan_target(request, snapshot, pose, start, graph, base)
        if request.goal_kind is PlanGoalKind.CHECK_POI:
            return self._plan_observation(request, snapshot, pose, start, graph, base, request.target_id)
        if request.goal_kind in (PlanGoalKind.EXPLORE_STOPS, PlanGoalKind.EXPLORE_PARKING):
            return self._plan_frontier(request, snapshot, pose, start, graph, base)
        return CorePlanResult(PlanStatus.INVALID_REQUEST, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="unsupported goal kind")

    def _plan_target(self, request, snapshot, pose, start, graph, base):
        if not request.target_id or request.target_id in request.exclude_target_ids:
            return CorePlanResult(PlanStatus.INVALID_REQUEST, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="target missing or excluded")
        transition = _target_transition(self._field, snapshot, self._navigation, request.target_id)
        if transition is None:
            return CorePlanResult(PlanStatus.NO_ROUTE, **base, route_id="", selected_target_id=request.target_id, reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="target unknown, conflicted, or lacks service geometry")
        graph = _add_transition(graph, transition)
        goal_anchor = transition.to_state.anchor_id
        if start.anchor_id == goal_anchor:
            return CorePlanResult(PlanStatus.ALREADY_AT_TERMINAL, **base, route_id="", selected_target_id=request.target_id, reaches_requested_target=True, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="already at target terminal")

        route = _search(graph, start, goal_anchor, provisional=True)
        if route is None:
            return CorePlanResult(PlanStatus.NO_ROUTE, **base, route_id="", selected_target_id=request.target_id, reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="no legal or observable provisional route")
        return self._materialize_route(request, snapshot, pose, route, request.target_id, base)

    def _plan_observation(self, request, snapshot, pose, start, graph, base, observation_id):
        if not observation_id or observation_id not in self._field.observation_ids:
            return CorePlanResult(PlanStatus.INVALID_REQUEST, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="unknown observation id")
        region = self._field.observation_region(observation_id)
        if start.anchor_id == region.anchor_id:
            return CorePlanResult(PlanStatus.ALREADY_AT_TERMINAL, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), terminal_observation_id=observation_id, reason="already at observation anchor")
        # A requested POI may sit beyond a currently unresolved traffic gate.
        # Search the provisional graph so we can identify that gate, but only
        # materialize the executable prefix leading to its observation hold.
        # _materialize_route() never emits the unresolved transition itself.
        route = _search(graph, start, region.anchor_id, provisional=True)
        if route is None:
            return CorePlanResult(PlanStatus.NO_ROUTE, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), terminal_observation_id=observation_id, reason="observation anchor not legally reachable")
        result = self._materialize_route(request, snapshot, pose, route, "", base)
        if result.status is PlanStatus.NEED_OBSERVATION:
            # Preserve the observation required by the first unresolved gate;
            # the originally requested POI becomes reachable only after it.
            return result
        return replace(result, terminal_observation_id=observation_id)

    def _plan_frontier(self, request, snapshot, pose, start, graph, base):
        _, _, coverage, _ = _index(snapshot)
        candidates = []
        for oid in self._field.observation_ids:
            region = self._field.observation_region(oid)
            cov = coverage.get(oid)
            if cov is not None and cov.state is CoverageResolution.COMPLETE:
                continue
            reachable = _reachable_cost(graph, start, region.anchor_id)
            if reachable is None:
                continue
            cost, path = reachable
            failed = 0 if cov is None else cov.failed_attempts
            candidates.append((cost, failed, oid, path))
        # Also consider observation points immediately before unresolved gates.
        for transition in graph.transitions:
            if transition.provisional and not transition.executable and transition.observation_id:
                reachable = _reachable_cost(graph, start, transition.from_state.anchor_id)
                if reachable is None:
                    continue
                cost, path = reachable
                cov = coverage.get(transition.observation_id)
                failed = 0 if cov is None else cov.failed_attempts
                candidates.append((cost, failed, transition.observation_id, path))
        if not candidates:
            return CorePlanResult(PlanStatus.NO_REACHABLE_FRONTIER, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), reason="no useful frontier reachable through executable motion")
        cost, _, oid, route = min(candidates, key=lambda x: (x[0], x[1], x[2]))
        if not route:
            return CorePlanResult(PlanStatus.ALREADY_AT_TERMINAL, **base, route_id="", selected_target_id="", reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=False, executable_prefix=(), terminal_observation_id=oid, reason="already at selected frontier")
        result = self._materialize_route(request, snapshot, pose, route, "", base)
        return replace(result, terminal_observation_id=oid, estimated_total_cost_s=cost)

    def _materialize_route(self, request, snapshot, pose, route, selected_target_id, base):
        executable = []
        unknown_observation = ""
        uses_unknown = False
        current_pose = pose
        for transition in route:
            if not transition.executable:
                uses_unknown = True
                unknown_observation = transition.observation_id
                break
            built = self._builder.build_segment(transition, snapshot, current_pose)
            if built.segment is None:
                return CorePlanResult(PlanStatus.NO_ROUTE, **base, route_id="", selected_target_id=selected_target_id, reaches_requested_target=False, estimated_total_cost_s=0.0, cost_uses_unknown_gates=uses_unknown, executable_prefix=tuple(executable), terminal_observation_id=unknown_observation, reason=f"segment build failed: {built.reason}")
            executable.append(built.segment)
            current_pose = PoseSample(
                current_pose.timestamp_s,
                self._field.field_to_map_pose(built.segment.path.poses[-1]),
            )

        total_cost = sum(t.estimated_cost_s for t in route)
        route_id = _route_id(request, snapshot, route)
        if uses_unknown:
            return CorePlanResult(
                PlanStatus.NEED_OBSERVATION,
                **base,
                route_id=route_id,
                selected_target_id=selected_target_id,
                reaches_requested_target=False,
                estimated_total_cost_s=total_cost,
                cost_uses_unknown_gates=True,
                executable_prefix=tuple(executable),
                terminal_observation_id=unknown_observation,
                reason="provisional route stops before unresolved gate",
            )
        return CorePlanResult(
            PlanStatus.ROUTE_READY,
            **base,
            route_id=route_id,
            selected_target_id=selected_target_id,
            reaches_requested_target=bool(selected_target_id),
            estimated_total_cost_s=total_cost,
            cost_uses_unknown_gates=False,
            executable_prefix=tuple(executable),
            reason="",
        )
