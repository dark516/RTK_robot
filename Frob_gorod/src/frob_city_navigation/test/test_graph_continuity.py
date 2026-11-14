import math

from frob_city_navigation.graph import DirectedGraph
from frob_city_navigation.models import (
    GraphState,
    GraphTransition,
    MetricPath,
    MovementMask,
    Pose2D,
    SegmentKind,
    TerminalKind,
)


def transition(edge_id, from_anchor, to_anchor, start_yaw, end_yaw):
    return GraphTransition(
        transition_id=edge_id,
        from_state=GraphState(from_anchor, ''),
        to_state=GraphState(to_anchor, edge_id),
        edge_id=edge_id,
        movement=MovementMask.STRAIGHT,
        path=MetricPath((
            Pose2D(0.0, 0.0, start_yaw),
            Pose2D(1.0, 0.0, end_yaw),
        )),
        permitted_regions=(),
        approach_id='',
        executable=True,
        provisional=True,
        dependencies=(),
        observation_id='',
        segment_kind=SegmentKind.CORRIDOR,
        terminal_kind=TerminalKind.HOLD,
        terminal_limit=Pose2D(1.0, 0.0, end_yaw),
        commitment_gate=None,
        estimated_cost_s=1.0,
    )


def test_outgoing_rejects_discontinuous_reverse_branch_after_previous_edge():
    incoming = transition('IN', 'A', 'MID', 0.0, 0.0)
    forward = transition('FORWARD', 'MID', 'B', 0.0, 0.0)
    reverse = transition('REVERSE', 'MID', 'A', math.pi, math.pi)
    graph = DirectedGraph((incoming, forward, reverse))

    outgoing = graph.outgoing(GraphState('MID', 'IN'), provisional=False)

    assert [item.edge_id for item in outgoing] == ['FORWARD']


def test_projected_first_edge_strictly_constrains_start_heading():
    north = transition('NORTH', 'START', 'N', math.pi / 2.0, math.pi / 2.0)
    east = transition('EAST', 'START', 'E', 0.0, 0.0)
    graph = DirectedGraph((north, east))

    outgoing = graph.outgoing(GraphState('START', 'NORTH'), provisional=False)

    assert [item.edge_id for item in outgoing] == ['NORTH']
