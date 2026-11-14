"""Directed state graph construction for Frob City WP08.

The graph is derived from static field primitives plus an immutable semantic
snapshot.  Unknown traffic gates may be represented provisionally, but they are
never marked executable.
"""

from __future__ import annotations

from collections import defaultdict
import math

from .field_model import FieldModel
from .geometry import path_length, project_point_to_path
from .models import (
    ApproachResolution,
    CoverageResolution,
    EdgeBlockState,
    GraphState,
    GraphTransition,
    MapSnapshot,
    Point2D,
    MovementMask,
    NavigationConfig,
    RouteDependency,
    SegmentKind,
    TerminalKind,
)


def _index_snapshot(snapshot: MapSnapshot):
    approaches = {x.approach_id: x for x in snapshot.approaches}
    edges = {x.edge_id: x for x in snapshot.edges}
    coverage = {x.coverage_id: x for x in snapshot.coverage}
    targets = {x.target_id: x for x in snapshot.targets}
    return approaches, edges, coverage, targets


def _observation_id_for_approach(field: FieldModel, anchor_id: str) -> str:
    regions = field.observation_regions_at_anchor(anchor_id)
    return regions[0].observation_id if regions else ""


def build_transitions(
    field: FieldModel,
    snapshot: MapSnapshot,
    navigation: NavigationConfig,
) -> tuple[GraphTransition, ...]:
    """Build deterministic static transitions annotated with semantic state."""
    approaches, edge_records, coverage_records, _ = _index_snapshot(snapshot)
    out: list[GraphTransition] = []

    for edge_id in field.movement_ids:
        primitive = field.movement_primitive(edge_id)
        edge_state = edge_records.get(edge_id)
        edge_version = edge_state.version if edge_state is not None else 0
        edge_open = edge_state is None or edge_state.state is EdgeBlockState.OPEN

        executable = edge_open
        provisional = edge_open
        observation_id = ""
        deps = [RouteDependency(f"edge:{edge_id}", edge_version)]

        if primitive.approach_id:
            approach = approaches.get(primitive.approach_id)
            if approach is None:
                executable = False
                provisional = edge_open
                observation_id = _observation_id_for_approach(field, primitive.from_anchor_id)
                deps.append(RouteDependency(f"approach:{primitive.approach_id}", 0))
            else:
                deps.append(RouteDependency(f"approach:{primitive.approach_id}", approach.version))
                resolved = approach.state in (
                    ApproachResolution.RESOLVED_CLEAR,
                    ApproachResolution.RESOLVED_SIGNED,
                )
                legal = bool(approach.allowed_movements & primitive.movement)
                held = approach.precautionary_hold
                if resolved and legal and not held and edge_open:
                    executable = True
                    provisional = True
                elif approach.state in (
                    ApproachResolution.UNKNOWN,
                    ApproachResolution.OBSERVING,
                ) or held:
                    executable = False
                    provisional = edge_open
                    observation_id = _observation_id_for_approach(field, primitive.from_anchor_id)
                else:
                    # Conflict, blocked, or resolved-illegal transitions do not
                    # belong even to the provisional graph.
                    executable = False
                    provisional = False


        # A long corridor may not skip an uncompleted roadside viewing
        # obligation whose anchor lies strictly inside this primitive.
        primitive_length = path_length(primitive.path)
        for oid in field.observation_ids:
            region = field.observation_region(oid)
            if region.anchor_id in (primitive.from_anchor_id, primitive.to_anchor_id):
                continue
            anchor_pose = field.anchor(region.anchor_id).pose
            projection = project_point_to_path(Point2D(anchor_pose.x_m, anchor_pose.y_m), primitive.path)
            if projection.distance_m > 0.08:
                continue
            if not (0.05 < projection.station_m < primitive_length - 0.05):
                continue
            coverage = coverage_records.get(oid)
            version = 0 if coverage is None else coverage.version
            deps.append(RouteDependency(f"coverage:{oid}", version))
            if coverage is None or coverage.state is not CoverageResolution.COMPLETE:
                executable = False
                provisional = provisional and edge_open
                if not observation_id:
                    observation_id = oid

        cost = path_length(primitive.path) / max(navigation.max_speed_mps, 1e-6)
        if provisional and not executable:
            cost += navigation.timeouts.observation_action_s

        kind = SegmentKind.JUNCTION if primitive.approach_id else SegmentKind.CORRIDOR
        out.append(
            GraphTransition(
                transition_id=edge_id,
                from_state=GraphState(primitive.from_anchor_id, ""),
                to_state=GraphState(primitive.to_anchor_id, edge_id),
                edge_id=edge_id,
                movement=primitive.movement,
                path=primitive.path,
                permitted_regions=primitive.permitted_regions,
                approach_id=primitive.approach_id,
                executable=executable,
                provisional=provisional,
                dependencies=tuple(deps),
                observation_id=observation_id,
                segment_kind=kind,
                terminal_kind=TerminalKind.HOLD,
                terminal_limit=primitive.path.poses[-1],
                commitment_gate=(
                    primitive.commitment_gate.pose if primitive.commitment_gate is not None else None
                ),
                estimated_cost_s=cost,
            )
        )
    return tuple(sorted(out, key=lambda t: t.transition_id))


class DirectedGraph:
    def __init__(self, transitions: tuple[GraphTransition, ...]):
        self.transitions = transitions
        adjacency: dict[str, list[GraphTransition]] = defaultdict(list)
        for transition in transitions:
            adjacency[transition.from_state.anchor_id].append(transition)
        self._adjacency = {
            key: tuple(sorted(value, key=lambda t: t.transition_id))
            for key, value in adjacency.items()
        }
        self._by_edge_id = {item.edge_id: item for item in transitions if item.edge_id}

    @staticmethod
    def _heading_continuous(
        state: GraphState,
        previous: GraphTransition,
        candidate: GraphTransition,
    ) -> bool:
        previous_yaw = previous.path.poses[-1].yaw_rad
        candidate_yaw = candidate.path.poses[0].yaw_rad
        error = abs(math.atan2(
            math.sin(candidate_yaw - previous_yaw),
            math.cos(candidate_yaw - previous_yaw),
        ))
        # Surveyed maneuvers start tangent to the incoming lane.  A generous
        # threshold tolerates local edits while rejecting the opposite branch.
        # If pose projection selected an edge that starts at this same anchor,
        # it is a first/partial edge and therefore a strict heading constraint.
        # After a completed incoming edge, ordinary 90-degree legacy turns are
        # still allowed; only the reverse branch is rejected.
        threshold = 60.0 if previous.from_state.anchor_id == state.anchor_id else 135.0
        return error <= math.radians(threshold)

    def outgoing(self, state: GraphState, *, provisional: bool) -> tuple[GraphTransition, ...]:
        items = self._adjacency.get(state.anchor_id, ())
        previous = self._by_edge_id.get(state.incoming_edge_id)
        if previous is not None:
            items = tuple(item for item in items if self._heading_continuous(state, previous, item))
        if provisional:
            return tuple(t for t in items if t.provisional)
        return tuple(t for t in items if t.executable)
