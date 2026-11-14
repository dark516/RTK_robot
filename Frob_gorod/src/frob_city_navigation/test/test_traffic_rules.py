from math import pi

import pytest

from frob_city_navigation.models import (
    ALL_MOVEMENTS,
    ApproachResolution,
    MovementMask,
    RuleDecisionCode,
    SignClass,
    TrafficPolicy,
)
from frob_city_navigation.traffic_rules import (
    allowed_movements,
    relative_movement_from_headings,
)


DEFAULT = TrafficPolicy()


@pytest.mark.parametrize(
    ("sign", "expected"),
    [
        (SignClass.STRAIGHT_ONLY, MovementMask.STRAIGHT),
        (SignClass.LEFT_ONLY, MovementMask.LEFT),
        (SignClass.RIGHT_ONLY, MovementMask.RIGHT),
        (SignClass.NO_LEFT, MovementMask.STRAIGHT | MovementMask.RIGHT),
        (SignClass.NO_RIGHT, MovementMask.STRAIGHT | MovementMask.LEFT),
    ],
)
def test_exact_directional_rule_table(sign, expected):
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({sign}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.code == RuleDecisionCode.ALLOWED
    assert result.allowed == expected


def test_qualified_clear_allows_all_available_policy_movements():
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset(),
        ApproachResolution.RESOLVED_CLEAR,
        DEFAULT,
    )
    assert result.allowed == ALL_MOVEMENTS


@pytest.mark.parametrize(
    "state",
    [
        ApproachResolution.UNKNOWN,
        ApproachResolution.OBSERVING,
        ApproachResolution.CONFLICT,
        ApproachResolution.BLOCKED,
    ],
)
def test_unresolved_conflict_or_blocked_never_authorizes(state):
    result = allowed_movements(ALL_MOVEMENTS, frozenset(), state, DEFAULT)
    assert result.allowed == MovementMask.NONE


def test_no_left_removes_only_left():
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({SignClass.NO_LEFT}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.STRAIGHT | MovementMask.RIGHT


def test_no_right_removes_only_right():
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({SignClass.NO_RIGHT}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.STRAIGHT | MovementMask.LEFT


def test_t_junction_no_left_leaves_only_right_when_straight_missing():
    available = MovementMask.LEFT | MovementMask.RIGHT
    result = allowed_movements(
        available,
        frozenset({SignClass.NO_LEFT}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.RIGHT


def test_t_junction_no_right_does_not_invent_left_when_straight_exists():
    available = MovementMask.STRAIGHT | MovementMask.LEFT | MovementMask.RIGHT
    result = allowed_movements(
        available,
        frozenset({SignClass.NO_RIGHT}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.STRAIGHT | MovementMask.LEFT


def test_mandatory_turn_intersects_physical_exits_and_can_become_empty():
    available = MovementMask.STRAIGHT | MovementMask.RIGHT
    result = allowed_movements(
        available,
        frozenset({SignClass.LEFT_ONLY}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.NONE
    assert result.code == RuleDecisionCode.RULE_CONFLICT_OR_NO_LEGAL_EXIT


def test_corner_topology_reduces_clear_result():
    available = MovementMask.STRAIGHT | MovementMask.RIGHT
    result = allowed_movements(
        available,
        frozenset(),
        ApproachResolution.RESOLVED_CLEAR,
        DEFAULT,
    )
    assert result.allowed == available


def test_incompatible_mandatory_signs_produce_no_legal_fallback():
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({SignClass.LEFT_ONLY, SignClass.RIGHT_ONLY}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.NONE
    assert result.code == RuleDecisionCode.RULE_CONFLICT_OR_NO_LEGAL_EXIT


def test_compatible_signs_intersect_normally():
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({SignClass.STRAIGHT_ONLY, SignClass.NO_LEFT}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.STRAIGHT


@pytest.mark.parametrize("nondirectional", [SignClass.BUS_STOP, SignClass.PARKING])
def test_bus_and_parking_do_not_change_directional_result(nondirectional):
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({SignClass.NO_LEFT, nondirectional}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.STRAIGHT | MovementMask.RIGHT
    assert nondirectional in result.nondirectional_signs


def test_bus_and_parking_alone_do_not_grant_nonexistent_turn():
    available = MovementMask.STRAIGHT
    result = allowed_movements(
        available,
        frozenset({SignClass.BUS_STOP, SignClass.PARKING}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.STRAIGHT


def test_hazard_marker_has_no_generic_turn_semantics():
    available = ALL_MOVEMENTS
    result = allowed_movements(
        available,
        frozenset({SignClass.HAZARD_MARKER}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
        blocked=MovementMask.RIGHT,
    )
    assert result.allowed == MovementMask.STRAIGHT | MovementMask.LEFT


def test_blocked_movements_are_removed_after_sign_constraints():
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({SignClass.NO_LEFT}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
        blocked=MovementMask.RIGHT,
    )
    assert result.allowed == MovementMask.STRAIGHT


def test_event_policy_intersects_before_topology_and_signs():
    policy = TrafficPolicy(permitted=MovementMask.STRAIGHT | MovementMask.RIGHT)
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({SignClass.NO_LEFT}),
        ApproachResolution.RESOLVED_SIGNED,
        policy,
    )
    assert result.allowed == MovementMask.STRAIGHT | MovementMask.RIGHT


def test_unknown_confirmed_sign_never_authorizes_motion():
    result = allowed_movements(
        ALL_MOVEMENTS,
        frozenset({SignClass.UNKNOWN}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert result.allowed == MovementMask.NONE
    assert result.code == RuleDecisionCode.UNKNOWN_CONFIRMED_SIGN


@pytest.mark.parametrize(
    ("incoming", "straight", "left", "right"),
    [
        (0.0, 0.0, pi / 2.0, -pi / 2.0),       # FROM_WEST, facing east
        (pi / 2.0, pi / 2.0, pi, 0.0),         # FROM_SOUTH, facing north
        (pi, pi, -pi / 2.0, pi / 2.0),         # FROM_EAST, facing west
        (-pi / 2.0, -pi / 2.0, 0.0, pi),       # FROM_NORTH, facing south
    ],
)
def test_relative_turn_classification_for_all_four_incoming_headings(
    incoming, straight, left, right
):
    assert relative_movement_from_headings(incoming, straight) == MovementMask.STRAIGHT
    assert relative_movement_from_headings(incoming, left) == MovementMask.LEFT
    assert relative_movement_from_headings(incoming, right) == MovementMask.RIGHT


def test_uturn_is_not_supported():
    assert relative_movement_from_headings(0.0, pi) == MovementMask.NONE


def test_non_cardinal_ambiguous_turn_is_not_silently_classified():
    assert relative_movement_from_headings(0.0, 0.55, tolerance_rad=0.20) == MovementMask.NONE


@pytest.mark.parametrize(
    "available",
    [
        MovementMask.STRAIGHT,
        MovementMask.LEFT,
        MovementMask.RIGHT,
        MovementMask.STRAIGHT | MovementMask.LEFT,
        MovementMask.STRAIGHT | MovementMask.RIGHT,
        MovementMask.LEFT | MovementMask.RIGHT,
        ALL_MOVEMENTS,
    ],
)
@pytest.mark.parametrize(
    "sign",
    [
        SignClass.STRAIGHT_ONLY,
        SignClass.LEFT_ONLY,
        SignClass.RIGHT_ONLY,
        SignClass.NO_LEFT,
        SignClass.NO_RIGHT,
    ],
)
def test_exhaustive_single_sign_result_is_subset_of_physical_topology(available, sign):
    result = allowed_movements(
        available,
        frozenset({sign}),
        ApproachResolution.RESOLVED_SIGNED,
        DEFAULT,
    )
    assert int(result.allowed & ~available) == 0
