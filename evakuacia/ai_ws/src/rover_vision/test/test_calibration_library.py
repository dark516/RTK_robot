import cv2
import numpy as np

from rover_vision.calibration_library import CalibrationLibrary, cardinal_direction


ROI = [.4, .68, .6, .68, .64, .9, .36, .9]


def test_frames_can_be_added_loaded_and_removed(tmp_path):
    library = CalibrationLibrary(tmp_path, ROI)
    frame = np.full((60, 80, 3), 123, dtype=np.uint8)

    saved = library.add_frame('normal', frame)

    assert len(library.list_frames('normal')) == 1
    assert library.load_frames('normal')[0].shape == frame.shape
    assert library.remove_frame('normal', saved['id'])
    assert library.list_frames('normal') == []


def test_each_profile_has_an_independent_roi(tmp_path):
    library = CalibrationLibrary(tmp_path, ROI)
    bridge = [.35, .55, .65, .55, .7, .82, .3, .82]

    library.set_roi('bridge', bridge)

    assert library.roi('normal') == ROI
    assert library.roi('bridge') == bridge


def test_bridge_pose_requires_cell_and_direction_match(tmp_path):
    library = CalibrationLibrary(tmp_path, ROI)
    library.add_bridge_pose(2, 9, 'south')

    assert library.profile_for_pose(2, 9, 'south') == 'bridge'
    assert library.profile_for_pose(2, 9, 'north') == 'normal'
    assert library.profile_for_pose(2, 8, 'south') == 'normal'


def test_cardinal_direction_uses_dominant_view_axis():
    assert cardinal_direction(.1, -.99) == 'south'
    assert cardinal_direction(-.99, -.1) == 'west'
