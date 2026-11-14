from dataclasses import FrozenInstanceError

import pytest

from frob_city_navigation.models import (
    ApproachResolution,
    CoverageResolution,
    EdgeBlockState,
    ExecutionOutcome,
    ExecutionState,
    MissionCommand,
    MissionPhase,
    MovementMask,
    ObserveOutcome,
    ParkingPriorState,
    PlanGoalKind,
    PlanStatus,
    Point2D,
    Pose2D,
    RangeConvention,
    SegmentKind,
    SegmentValidationStatus,
    SignClass,
    TargetResolution,
    TerminalKind,
)


def test_wp01_enum_values_are_frozen():
    assert [int(v) for v in SignClass] == list(range(9))
    assert int(RangeConvention.RANGE_UNKNOWN) == 0
    assert int(RangeConvention.OPTICAL_Z) == 1
    assert int(RangeConvention.RAY_RANGE) == 2
    assert int(MovementMask.STRAIGHT) == 1
    assert int(MovementMask.LEFT) == 2
    assert int(MovementMask.RIGHT) == 4
    assert [int(v) for v in ApproachResolution] == list(range(6))
    assert [int(v) for v in TargetResolution] == list(range(3))
    assert [int(v) for v in EdgeBlockState] == list(range(3))
    assert [int(v) for v in CoverageResolution] == list(range(4))
    assert [int(v) for v in SegmentKind] == list(range(4))
    assert [int(v) for v in TerminalKind] == list(range(4))
    assert [int(v) for v in MissionPhase] == list(range(8))
    assert [int(v) for v in ExecutionState] == list(range(9))
    assert [int(v) for v in ParkingPriorState] == list(range(6))
    assert [int(v) for v in PlanGoalKind] == list(range(4))
    assert [int(v) for v in PlanStatus] == list(range(8))
    assert [int(v) for v in SegmentValidationStatus] == list(range(5))
    assert [int(v) for v in ExecutionOutcome] == list(range(3))
    assert [int(v) for v in ObserveOutcome] == list(range(7))
    assert [int(v) for v in MissionCommand] == list(range(5))


def test_core_records_are_immutable():
    point = Point2D(1.0, 2.0)
    pose = Pose2D(1.0, 2.0, 0.3)
    with pytest.raises(FrozenInstanceError):
        point.x_m = 9.0
    with pytest.raises(FrozenInstanceError):
        pose.yaw_rad = 1.0


def test_models_do_not_import_ros():
    import frob_city_navigation.models as models

    names = set(models.__dict__)
    forbidden = {name for name in names if name.startswith("rclpy") or name.startswith("geometry_msgs") or name.startswith("nav_msgs") or name.startswith("tf2")}
    assert forbidden == set()
