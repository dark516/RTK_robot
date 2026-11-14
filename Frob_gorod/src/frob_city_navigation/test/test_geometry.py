from __future__ import annotations

import math
import random
from dataclasses import replace
from pathlib import Path

import pytest

from frob_city_navigation.config_loader import load_project_config
from frob_city_navigation.field_model import FieldModel
from frob_city_navigation.geometry import (
    field_to_map_pose,
    map_to_field_pose,
    normalize_angle,
    path_length,
    project_point_to_path,
    validate_swept_footprint,
)
from frob_city_navigation.models import (
    ConfigPaths,
    DirectedAnchorConfig,
    DirectedEdgeConfig,
    FieldTransform,
    GeometryMargins,
    GeometryStatus,
    MetricPath,
    MovementMask,
    NamedPolygon,
    Point2D,
    Polygon2D,
    Pose2D,
    PoseSample,
    ProjectionStatus,
    ServiceGeometryStatus,
)
from frob_city_navigation.pose_projection import construct_service_geometry, project_pose


FIXTURE = Path(__file__).parent / "fixtures" / "valid"


def _load_field():
    project = load_project_config(
        ConfigPaths(
            field=FIXTURE / "field.yaml",
            event=FIXTURE / "event.yaml",
            perception=FIXTURE / "perception.yaml",
            navigation=FIXTURE / "navigation.yaml",
        )
    )
    return project.field


def _rect(x0, y0, x1, y1):
    return Polygon2D(
        (
            Point2D(x0, y0),
            Point2D(x1, y0),
            Point2D(x1, y1),
            Point2D(x0, y1),
        )
    )


def test_field_map_transform_round_trip_randomized():
    transform = FieldTransform(1.17, -0.44, 0.63, True)
    rng = random.Random(20260912)
    for _ in range(250):
        original = Pose2D(
            rng.uniform(-1.0, 5.0),
            rng.uniform(-1.0, 5.0),
            rng.uniform(-8.0 * math.pi, 8.0 * math.pi),
        )
        mapped = field_to_map_pose(original, transform)
        recovered = map_to_field_pose(mapped, transform)
        assert recovered.x_m == pytest.approx(original.x_m, abs=1e-9)
        assert recovered.y_m == pytest.approx(original.y_m, abs=1e-9)
        assert normalize_angle(recovered.yaw_rad) == pytest.approx(
            normalize_angle(original.yaw_rad), abs=1e-9
        )


def test_yaw_wrap_is_canonical():
    assert normalize_angle(0.0) == 0.0
    assert normalize_angle(2.0 * math.pi) == pytest.approx(0.0)
    assert normalize_angle(3.0 * math.pi) == pytest.approx(-math.pi)
    assert -math.pi <= normalize_angle(123.456) < math.pi


def test_field_model_required_queries():
    field = FieldModel.from_config(_load_field())
    assert field.anchor("A_START").cell_id == "C0_0"
    assert field.movement_primitive("C0_0->C1_0").to_anchor_id == "A_EAST"
    assert field.observation_region("OBS_EAST").anchor_id == "A_START"
    assert field.candidate_sign_sites("C1_0/FROM_WEST")[0].site_id == "SITE_0"
    assert field.geometry_calibrated is False


def test_direction_aware_projection_matches_forward_edge():
    field = FieldModel.from_config(_load_field())
    sample = PoseSample(1.0, Pose2D(0.72, 0.42, 0.02))
    result = project_pose(field, sample, None)
    assert result.status == ProjectionStatus.MATCHED
    assert result.projection is not None
    assert result.projection.movement_id == "C0_0->C1_0"
    assert result.projection.station_m == pytest.approx(0.32, abs=0.03)
    assert result.projection.cross_track_error_m == pytest.approx(0.02, abs=1e-6)


def test_wrong_way_heading_does_not_match_forward_approach():
    field = FieldModel.from_config(_load_field())
    sample = PoseSample(1.0, Pose2D(0.72, 0.40, math.pi))
    result = project_pose(field, sample, None)
    assert result.status == ProjectionStatus.NO_MATCH
    assert result.projection is None


def _parallel_ambiguous_field():
    base = _load_field()
    anchors = (
        DirectedAnchorConfig("A0", "C0_0", Pose2D(0.2, 0.40, 0.0)),
        DirectedAnchorConfig("A1", "C1_0", Pose2D(1.4, 0.40, 0.0)),
        DirectedAnchorConfig("B0", "C0_0", Pose2D(0.2, 0.44, 0.0)),
        DirectedAnchorConfig("B1", "C1_0", Pose2D(1.4, 0.44, 0.0)),
    )
    edges = (
        DirectedEdgeConfig(
            "EDGE_A", "A0", "A1", MovementMask.STRAIGHT,
            MetricPath((Pose2D(0.2, 0.40, 0.0), Pose2D(1.4, 0.40, 0.0))),
            ("ROAD",),
        ),
        DirectedEdgeConfig(
            "EDGE_B", "B0", "B1", MovementMask.STRAIGHT,
            MetricPath((Pose2D(0.2, 0.44, 0.0), Pose2D(1.4, 0.44, 0.0))),
            ("ROAD",),
        ),
    )
    return replace(
        base,
        drivable_regions=(NamedPolygon("ROAD", _rect(0.0, 0.0, 1.6, 0.8)),),
        anchors=anchors,
        edges=edges,
        observation_regions=(),
        sign_sites=(),
        gates=(),
    )


def test_ambiguous_projection_does_not_silently_snap():
    field = FieldModel.from_config(_parallel_ambiguous_field())
    sample = PoseSample(1.0, Pose2D(0.8, 0.42, 0.0))
    result = project_pose(field, sample, None)
    assert result.status == ProjectionStatus.AMBIGUOUS
    assert result.projection is None
    assert set(result.candidate_movement_ids) == {"EDGE_A", "EDGE_B"}


def test_previous_projection_provides_intersection_hysteresis():
    field = FieldModel.from_config(_parallel_ambiguous_field())
    first = project_pose(field, PoseSample(0.9, Pose2D(0.7, 0.401, 0.0)), None)
    assert first.status == ProjectionStatus.MATCHED
    assert first.projection is not None
    assert first.projection.movement_id == "EDGE_A"

    ambiguous_position = PoseSample(1.0, Pose2D(0.8, 0.42, 0.0))
    held = project_pose(field, ambiguous_position, first.projection)
    assert held.status == ProjectionStatus.MATCHED
    assert held.projection is not None
    assert held.projection.movement_id == "EDGE_A"
    assert "hysteresis" in held.reason


def test_out_of_field_pose_returns_explicit_error():
    field = FieldModel.from_config(_load_field())
    result = project_pose(field, PoseSample(1.0, Pose2D(8.0, 8.0, 0.0)), None)
    assert result.status == ProjectionStatus.OUT_OF_FIELD
    assert result.projection is None


def test_projection_station_is_monotonic_along_path_property():
    path = MetricPath(
        (
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(1.0, 0.0, 0.0),
            Pose2D(1.0, 1.0, math.pi / 2),
        )
    )
    points = [Point2D(x / 20.0, 0.02) for x in range(0, 21)]
    stations = [project_point_to_path(p, path).station_m for p in points]
    assert stations == sorted(stations)
    assert stations[0] == pytest.approx(0.0)
    assert stations[-1] == pytest.approx(1.02)
    assert path_length(path) == pytest.approx(2.0)


def test_whole_footprint_excludes_full_800mm_platform_even_with_clear_centerline():
    # The platform begins at y=0.80 and is the full required 0.8 x 0.8 m.
    # Robot centerline y=0.75 is geometrically clear, but a 0.30 m-wide robot
    # extends to y=0.90 and therefore overlaps the platform/forbidden area.
    platform = _rect(0.8, 0.8, 1.6, 1.6)
    assert (1.6 - 0.8) == pytest.approx(0.8)
    assert platform.points[0].y_m == pytest.approx(0.8)

    allowed_road_below_platform = _rect(0.0, 0.0, 2.0, 0.8)
    footprint = _rect(-0.15, -0.15, 0.15, 0.15)
    path = MetricPath((Pose2D(0.2, 0.75, 0.0), Pose2D(1.8, 0.75, 0.0)))
    result = validate_swept_footprint(
        path,
        (allowed_road_below_platform,),
        footprint,
        GeometryMargins(0.0, 0.0, 0.0),
    )
    assert result.status == GeometryStatus.OUTSIDE_PERMITTED
    assert result.valid is False


def test_rounded_corner_swept_footprint_is_checked_and_valid_in_large_region():
    footprint = _rect(-0.12, -0.10, 0.12, 0.10)
    # Curved pose sequence: yaw changes continuously through the corner.
    path = MetricPath(
        (
            Pose2D(0.20, 0.30, 0.0),
            Pose2D(0.48, 0.33, math.radians(20)),
            Pose2D(0.67, 0.48, math.radians(55)),
            Pose2D(0.72, 0.75, math.pi / 2),
        )
    )
    result = validate_swept_footprint(
        path,
        (_rect(0.0, 0.0, 1.0, 1.0),),
        footprint,
        GeometryMargins(0.01, 0.01, 0.01),
    )
    assert result.status == GeometryStatus.VALID
    assert result.valid is True
    assert result.swept_area_m2 > 0.024  # larger than the static footprint area


def test_margin_can_turn_nominally_valid_path_into_invalid_path():
    footprint = _rect(-0.10, -0.10, 0.10, 0.10)
    path = MetricPath((Pose2D(0.2, 0.2, 0.0), Pose2D(0.8, 0.2, 0.0)))
    allowed = (_rect(0.0, 0.0, 1.0, 0.31),)
    no_margin = validate_swept_footprint(
        path, allowed, footprint, GeometryMargins(0.0, 0.0, 0.0)
    )
    with_margin = validate_swept_footprint(
        path, allowed, footprint, GeometryMargins(0.02, 0.01, 0.0)
    )
    assert no_margin.valid is True
    assert with_margin.valid is False


def test_service_pose_is_before_sign_terminal_plane():
    field = FieldModel.from_config(_load_field())
    site = field.sign_site("SITE_0")
    result = construct_service_geometry(field, site, "C0_0->C1_0")
    assert result.status == ServiceGeometryStatus.VALID
    assert result.valid is True
    assert result.service_pose is not None
    assert result.terminal_limit is not None
    # Sign x=0.9 projects onto y=0.4 road. Footprint front extent is 0.15 m.
    assert result.terminal_limit.x_m == pytest.approx(0.9, abs=1e-9)
    assert result.service_pose.x_m == pytest.approx(0.75, abs=1e-9)
    assert result.service_pose.x_m + result.front_extent_m == pytest.approx(
        result.terminal_limit.x_m, abs=1e-9
    )
    assert result.calibration_required is True


def test_unknown_service_road_fails_explicitly():
    field = FieldModel.from_config(_load_field())
    result = construct_service_geometry(field, field.sign_site("SITE_0"), "NOPE")
    assert result.status == ServiceGeometryStatus.UNKNOWN_ROAD
    assert result.valid is False
