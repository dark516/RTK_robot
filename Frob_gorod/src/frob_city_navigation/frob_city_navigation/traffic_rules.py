"""Pure approach-relative traffic-rule engine for Frob City.

This module intentionally knows nothing about ROS, maps, mission state, YOLO,
or Nav2.  It answers only one question: given the movements that physically
exist at the current approach, the confirmed applicable sign set, the
approach's observation resolution and any known blocked movements, which
relative movements are legal?
"""

from __future__ import annotations

from math import pi
from typing import Iterable

from .models import (
    ALL_MOVEMENTS,
    ApproachResolution,
    MovementMask,
    RuleDecision,
    RuleDecisionCode,
    SignClass,
    TrafficPolicy,
)


_DIRECTIONAL_CONSTRAINTS: dict[SignClass, MovementMask] = {
    SignClass.STRAIGHT_ONLY: MovementMask.STRAIGHT,
    SignClass.LEFT_ONLY: MovementMask.LEFT,
    SignClass.RIGHT_ONLY: MovementMask.RIGHT,
    SignClass.NO_LEFT: MovementMask.STRAIGHT | MovementMask.RIGHT,
    SignClass.NO_RIGHT: MovementMask.STRAIGHT | MovementMask.LEFT,
}

_NON_DIRECTIONAL = frozenset(
    {
        SignClass.BUS_STOP,
        SignClass.PARKING,
        SignClass.HAZARD_MARKER,
    }
)


def _mask_only_known_bits(mask: MovementMask) -> MovementMask:
    """Reject/strip impossible bits defensively while preserving IntFlag API."""
    return MovementMask(int(mask) & int(ALL_MOVEMENTS))


def allowed_movements(
    available: MovementMask,
    confirmed_signs: frozenset[SignClass],
    approach_state: ApproachResolution,
    policy: TrafficPolicy,
    blocked: MovementMask = MovementMask.NONE,
) -> RuleDecision:
    """Return the legal *relative* movements for one incoming approach.

    Order of operations follows architecture section 8.4:
      1. unresolved/conflicted/blocked approach => no executable movement;
      2. start from event policy;
      3. intersect every directional sign constraint;
      4. intersect physical/topological exits;
      5. remove known blocked movements;
      6. never invent a fallback if the result is empty.

    BUS_STOP and PARKING never change a turn mask. HAZARD_MARKER also has no
    generic direction meaning; its associated blocked edge/area is represented
    through `blocked` by the semantic/geometry layer.
    """

    available = _mask_only_known_bits(available)
    blocked = _mask_only_known_bits(blocked)
    permitted = _mask_only_known_bits(policy.permitted)

    if approach_state in (ApproachResolution.UNKNOWN, ApproachResolution.OBSERVING):
        return RuleDecision(
            allowed=MovementMask.NONE,
            code=RuleDecisionCode.UNRESOLVED_APPROACH,
            reason="approach is not resolved",
        )

    if approach_state == ApproachResolution.CONFLICT:
        return RuleDecision(
            allowed=MovementMask.NONE,
            code=RuleDecisionCode.APPROACH_CONFLICT,
            reason="approach evidence is conflicted",
        )

    if approach_state == ApproachResolution.BLOCKED:
        return RuleDecision(
            allowed=MovementMask.NONE,
            code=RuleDecisionCode.APPROACH_BLOCKED,
            reason="approach is blocked",
        )

    if SignClass.UNKNOWN in confirmed_signs:
        return RuleDecision(
            allowed=MovementMask.NONE,
            code=RuleDecisionCode.UNKNOWN_CONFIRMED_SIGN,
            reason="UNKNOWN cannot be a confirmed authorizing sign",
        )

    directional = tuple(
        sorted(
            (s for s in confirmed_signs if s in _DIRECTIONAL_CONSTRAINTS),
            key=int,
        )
    )
    nondirectional = tuple(
        sorted((s for s in confirmed_signs if s in _NON_DIRECTIONAL), key=int)
    )

    legal = permitted

    for sign in directional:
        legal &= _DIRECTIONAL_CONSTRAINTS[sign]

    legal &= available
    legal &= ~blocked
    legal = _mask_only_known_bits(legal)

    if legal == MovementMask.NONE:
        return RuleDecision(
            allowed=MovementMask.NONE,
            code=RuleDecisionCode.RULE_CONFLICT_OR_NO_LEGAL_EXIT,
            reason="sign constraints, policy, topology, or blockage leave no legal exit",
            applied_directional_signs=directional,
            nondirectional_signs=nondirectional,
        )

    return RuleDecision(
        allowed=legal,
        code=RuleDecisionCode.ALLOWED,
        reason="legal movement set resolved",
        applied_directional_signs=directional,
        nondirectional_signs=nondirectional,
    )


def normalize_angle(angle_rad: float) -> float:
    """Normalize to [-pi, pi)."""
    value = (angle_rad + pi) % (2.0 * pi) - pi
    # Avoid returning -0.0 in diagnostics/tests.
    return 0.0 if abs(value) < 1e-15 else value


def relative_movement_from_headings(
    incoming_forward_yaw_rad: float,
    outgoing_forward_yaw_rad: float,
    *,
    tolerance_rad: float = 0.20,
) -> MovementMask:
    """Classify cardinal-like outgoing motion relative to incoming travel.

    This helper exists to keep compass headings out of the traffic rule table.
    A movement is classified by signed yaw change:
      ~0      -> STRAIGHT
      +pi/2   -> LEFT
      -pi/2   -> RIGHT

    U-turn (~pi) is intentionally unsupported in version 1 and returns NONE.
    Non-cardinal/ambiguous changes outside `tolerance_rad` also return NONE.
    """

    delta = normalize_angle(outgoing_forward_yaw_rad - incoming_forward_yaw_rad)
    candidates = (
        (0.0, MovementMask.STRAIGHT),
        (pi / 2.0, MovementMask.LEFT),
        (-pi / 2.0, MovementMask.RIGHT),
    )
    best_angle, best_movement = min(candidates, key=lambda item: abs(normalize_angle(delta - item[0])))
    if abs(normalize_angle(delta - best_angle)) <= tolerance_rad:
        return best_movement
    return MovementMask.NONE
