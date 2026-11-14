import json
import math
from types import SimpleNamespace as NS

from frob_city_web.serializers import (
    detections_json,
    field_layout_json,
    mission_json,
    motion_permit_json,
    path_json,
    pose_json,
    stamp_seconds,
    twist_json,
)


def stamp(value):
    return NS(sec=int(value), nanosec=int((value - int(value)) * 1_000_000_000))


def header(value=1.5, frame="map"):
    return NS(stamp=stamp(value), frame_id=frame)


def pose(x=1.0, y=2.0, yaw=math.pi / 2):
    return NS(
        header=header(),
        pose=NS(
            position=NS(x=x, y=y, z=0.0),
            orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
        ),
    )


class Mission:
    PHASE_IDLE = 0
    PHASE_DONE = 5
    EXEC_HOLDING = 0
    EXEC_MOVING = 4
    PARKING_PRIOR_NOT_CHECKED = 0


def test_pose_twist_stamp_and_path_are_json_safe():
    assert stamp_seconds(stamp(3.25)) == 3.25
    assert pose_json(pose())["yaw_rad"] == pytest_approx(math.pi / 2)
    assert twist_json(NS(linear=NS(x=0.2, y=0.0), angular=NS(z=-0.1))) == {
        "linear_x_mps": 0.2,
        "linear_y_mps": 0.0,
        "angular_z_radps": -0.1,
    }
    result = path_json(NS(header=header(), poses=[pose(1.0, 2.0), pose(2.0, 3.0)]))
    assert [(p["x_m"], p["y_m"]) for p in result["poses"]] == [(1.0, 2.0), (2.0, 3.0)]
    json.dumps(result, allow_nan=False)


def test_mission_labels_come_from_message_constants():
    msg = Mission()
    msg.header = header()
    msg.run_id = "run"
    msg.navigator_instance_id = "nav"
    msg.phase = 5
    msg.execution_state = 4
    msg.objective_id = "target"
    msg.route_id = "route"
    msg.active_segment_id = "segment"
    msg.goal_generation = 8
    msg.required_unique_bus_stops = 2
    msg.serviced_stop_ids = ["a"]
    msg.current_dwell_s = 1.25
    msg.parking_prior_state = 0
    msg.attempt_tainted = False
    msg.reason = "moving"

    result = mission_json(msg)
    assert result["phase"] == {"value": 5, "label": "DONE"}
    assert result["execution_state"] == {"value": 4, "label": "MOVING"}
    assert result["parking_prior_state"]["label"] == "NOT_CHECKED"


def test_detection_payload_includes_boxes_range_and_invalid_frame():
    detection = NS(
        sign_class=6,
        confidence=0.75,
        x_min_px=10,
        y_min_px=20,
        width_px=30,
        height_px=40,
        range_valid=True,
        range_convention=1,
        range_m=0.9,
        range_stddev_m=0.1,
        detector_track_id="track",
    )
    msg = NS(
        header=header(), detector_instance_id="det", frame_sequence=9,
        image_width_px=640, image_height_px=480, calibration_id="cal",
        frame_valid=False, invalid_reason="blur", detections=[detection],
    )
    msg.STRAIGHT_ONLY = 1
    msg.BUS_STOP = 6
    msg.RANGE_OPTICAL_Z = 1

    result = detections_json(msg)
    assert result["frame_valid"] is False
    assert result["detections"][0]["class"] == {"value": 6, "label": "BUS_STOP"}
    assert result["detections"][0]["bbox"] == {"x": 10, "y": 20, "width": 30, "height": 40}


def test_field_layout_uses_canonical_config_records():
    point = lambda x, y: NS(x_m=x, y_m=y)
    polygon = NS(points=(point(0, 0), point(1, 0), point(1, 1)))
    config = NS(field=NS(
        frame_id="map", width_m=4.0, height_m=4.0,
        geometry_calibrated=True,
        field_to_map=NS(x_m=0.1, y_m=0.2, yaw_rad=0.3, valid=True),
        footprint=polygon,
        drivable_regions=(NS(polygon_id="road", polygon=polygon),),
        platforms=(),
        anchors=(NS(anchor_id="a", cell_id="c", pose=NS(x_m=1, y_m=2, yaw_rad=0)),),
        gates=(), observation_regions=(),
        edges=(NS(edge_id="e", from_anchor_id="a", to_anchor_id="b", movement=1,
                  path=NS(poses=(NS(x_m=1, y_m=2, yaw_rad=0),)),
                  permitted_region_ids=("road",), approach_id="ap", commitment_gate_id=None),),
        sign_sites=(NS(site_id="s", position=point(3, 2),
                       faces=(NS(face_id="f", yaw_rad=1.0, controlled_approach_id="ap"),)),),
    ))

    result = field_layout_json(config)
    assert result["size_m"] == [4.0, 4.0]
    assert result["sign_sites"][0]["position"] == [3.0, 2.0]
    assert result["edges"][0]["path"][0]["x_m"] == 1.0


def test_motion_permit_contains_path_corridor_and_terminal_limit():
    polygon = NS(header=header(), polygon=NS(points=[NS(x=0.0, y=0.0), NS(x=1.0, y=0.0)]))
    segment = NS(
        segment_id="seg", kind=0, from_anchor_id="a", to_anchor_id="b",
        approach_id="ap", selected_movement=1,
        path=NS(header=header(), poses=[pose(0.0, 0.0), pose(1.0, 0.0)]),
        allowed_regions=[polygon], dependency_ids=["edge"], dependency_versions=[2],
        traversed_edge_ids=["edge"], terminal_kind=0, terminal_id="",
        terminal_limit=pose(1.0, 0.0), commitment_gate_valid=False,
        commitment_gate=pose(), max_speed_mps=0.2, max_yaw_rate_radps=0.5,
        xy_tolerance_m=0.05, yaw_tolerance_rad=0.1, estimated_duration_s=5.0,
    )
    msg = NS(
        header=header(), run_id="run", map_instance_id="map", navigator_instance_id="nav",
        field_id="field", goal_generation=3, sequence=8, enabled=True,
        valid_for_s=0.2, segment=segment,
    )
    result = motion_permit_json(msg)
    assert result["enabled"] is True
    assert result["segment"]["allowed_regions"][0][1] == [1.0, 0.0]
    assert result["segment"]["terminal_limit"]["x_m"] == 1.0


def pytest_approx(value):
    import pytest
    return pytest.approx(value)
