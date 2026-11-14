import math

from frob_city_interfaces.msg import SignDetection
from traffic_sign_interfaces.msg import TrafficSignDetection, TrafficSignDetectionArray

from frob_city_navigation.detection_adapter_node import adapt_legacy_frame


def frame(*detections):
    msg = TrafficSignDetectionArray()
    msg.header.frame_id = "camera"
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 34
    msg.detections = list(detections)
    return msg


def detection(name="bus_stop", distance_cm=125.0):
    msg = TrafficSignDetection()
    msg.class_name = name
    msg.class_id = 6
    msg.confidence = 0.8
    msg.has_distance = True
    msg.distance_cm = distance_cm
    msg.x_min = 10
    msg.y_min = 20
    msg.x_max = 50
    msg.y_max = 80
    return msg


def adapt(msg, **changes):
    values = dict(
        detector_instance_id="adapter-1",
        frame_sequence=7,
        image_width_px=1280,
        image_height_px=720,
        calibration_id="UNCOMMISSIONED",
    )
    values.update(changes)
    return adapt_legacy_frame(msg, **values)


def test_positive_class_and_capture_metadata_are_preserved_but_range_is_untrusted():
    out = adapt(frame(detection()))
    assert out.header.frame_id == "camera"
    assert out.header.stamp.sec == 12
    assert out.detector_instance_id == "adapter-1"
    assert out.frame_sequence == 7
    assert out.frame_valid
    assert len(out.detections) == 1
    item = out.detections[0]
    assert item.sign_class == SignDetection.BUS_STOP
    assert (item.x_min_px, item.y_min_px, item.width_px, item.height_px) == (10, 20, 40, 60)
    assert not item.range_valid
    assert item.range_convention == SignDetection.RANGE_UNKNOWN


def test_range_is_converted_from_centimeters_exactly_once_only_when_commissioned():
    out = adapt(
        frame(detection(distance_cm=125.0)),
        trust_legacy_range=True,
        range_stddev_m=0.12,
    )
    item = out.detections[0]
    assert item.range_valid
    assert item.range_convention == SignDetection.OPTICAL_Z
    assert math.isclose(item.range_m, 1.25)
    assert math.isclose(item.range_stddev_m, 0.12)


def test_empty_legacy_frame_is_not_qualified_absence_by_default():
    out = adapt(frame())
    assert not out.frame_valid
    assert out.invalid_reason == "legacy_empty_frame_not_qualified"

    commissioned = adapt(frame(), allow_qualified_empty_frames=True)
    assert commissioned.frame_valid
    assert commissioned.invalid_reason == ""


def test_unknown_label_or_invalid_box_cannot_make_a_healthy_frame():
    unknown = detection(name="mystery")
    bad_box = detection(name="left")
    bad_box.x_max = bad_box.x_min
    out = adapt(frame(unknown, bad_box))
    assert not out.frame_valid
    assert out.detections == []
    assert "2_rejected" in out.invalid_reason
