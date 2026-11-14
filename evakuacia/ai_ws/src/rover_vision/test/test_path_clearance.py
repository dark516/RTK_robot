from pathlib import Path

import cv2
import numpy as np

from rover_vision.path_clearance import RoadAppearanceModel, TemporalDecision


ROI = [0.30, 0.15, 0.70, 0.15, 0.85, 0.95, 0.15, 0.95]


def road_frame(
    with_obstacle: bool = False,
    with_marking: bool = True,
    obstacle_color: tuple[int, int, int] = (20, 30, 190),
    road_color: tuple[int, int, int] = (125, 125, 125),
) -> np.ndarray:
    frame = np.full((360, 480, 3), road_color, dtype=np.uint8)
    if with_marking:
        cv2.rectangle(frame, (225, 30), (255, 330), (235, 235, 235), -1)
    if with_obstacle:
        cv2.rectangle(frame, (150, 150), (330, 280), obstacle_color, -1)
        cv2.rectangle(frame, (165, 170), (315, 195), (20, 20, 20), -1)
    return frame


def test_clear_calibration_frame_is_clear():
    model = RoadAppearanceModel(ROI)
    model.fit([road_frame() for _ in range(4)])

    metrics, _warped, _mask = model.analyze(road_frame())

    assert not metrics.blocked
    assert metrics.anomaly_fraction < 0.05


def test_large_object_crossing_corridor_is_blocked():
    model = RoadAppearanceModel(ROI)
    model.fit([road_frame() for _ in range(4)])

    metrics, _warped, _mask = model.analyze(road_frame(with_obstacle=True))

    assert metrics.blocked
    assert metrics.largest_component_fraction > 0.025


def test_road_marking_is_not_required():
    model = RoadAppearanceModel(ROI)
    model.fit([road_frame() for _ in range(4)])

    metrics, _warped, _mask = model.analyze(road_frame(with_marking=False))

    assert not metrics.blocked


def test_low_contrast_object_is_blocked_by_its_contour():
    model = RoadAppearanceModel(ROI)
    model.fit([road_frame() for _ in range(4)])

    metrics, _warped, _mask = model.analyze(
        road_frame(with_obstacle=True, obstacle_color=(105, 105, 105))
    )

    assert metrics.blocked


def test_thin_floor_seam_across_corridor_is_clear():
    model = RoadAppearanceModel(ROI)
    model.fit([road_frame(with_marking=False) for _ in range(4)])
    frame = road_frame(with_marking=False)
    cv2.line(frame, (70, 230), (410, 230), (70, 70, 70), 3)

    metrics, _warped, _mask = model.analyze(frame)

    assert not metrics.blocked


def test_large_white_direction_arrow_is_traversable():
    model = RoadAppearanceModel(ROI)
    model.fit([road_frame(with_marking=False) for _ in range(4)])
    frame = road_frame(with_marking=False)
    cv2.arrowedLine(
        frame,
        (130, 245),
        (360, 245),
        (235, 235, 235),
        28,
        tipLength=0.35,
    )

    metrics, _warped, _mask = model.analyze(frame)

    assert not metrics.blocked


def test_orange_vehicle_is_not_suppressed_as_yellow_paint():
    model = RoadAppearanceModel(ROI)
    model.fit([road_frame() for _ in range(4)])

    metrics, _warped, _mask = model.analyze(
        road_frame(with_obstacle=True, obstacle_color=(20, 105, 230))
    )

    assert metrics.blocked


def test_model_round_trip(tmp_path: Path):
    model = RoadAppearanceModel(ROI)
    model.fit([road_frame() for _ in range(4)])
    path = tmp_path / 'road_model.npz'
    model.save(path)

    loaded = RoadAppearanceModel.load(path)
    metrics, _warped, _mask = loaded.analyze(road_frame())

    assert not metrics.blocked
    assert loaded.clear_cell_count == 1


def test_multiple_clear_cells_expand_the_road_palette(tmp_path: Path):
    model = RoadAppearanceModel(ROI)
    light = road_frame(with_marking=False, road_color=(145, 145, 145))
    dark = road_frame(with_marking=False, road_color=(70, 75, 80))
    model.fit([light for _ in range(4)])

    before, _warped, _mask = model.analyze(dark)
    model.fit([dark for _ in range(4)], append=True)
    after_light, _warped, _mask = model.analyze(light)
    after_dark, _warped, _mask = model.analyze(dark)
    path = tmp_path / 'multi_cell_model.npz'
    model.save(path)
    loaded = RoadAppearanceModel.load(path)

    assert before.blocked
    assert not after_light.blocked
    assert not after_dark.blocked
    assert model.clear_cell_count == 2
    assert loaded.clear_cell_count == 2


def test_temporal_decision_requires_multiple_votes():
    decision = TemporalDecision(window_size=5, blocked_votes=3, clear_votes=4)

    assert decision.update(True) == 'unknown'
    assert decision.update(False) == 'unknown'
    assert decision.update(True) == 'unknown'
    assert decision.update(True) == 'blocked'
