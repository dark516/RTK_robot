import math

from rover_lidar_filter.geometry import (
    PlanarTransform,
    filter_ranges_inside_rectangle,
    point_inside_rectangle,
    yaw_from_quaternion,
)


def test_planar_transform_handles_forward_offset_and_pi_yaw():
    transform = PlanarTransform(x=0.0662, y=0.0, yaw=math.pi)
    x, y = transform.apply(0.1762, 0.0)
    assert math.isclose(x, -0.11, abs_tol=1e-6)
    assert math.isclose(y, 0.0, abs_tol=1e-6)


def test_rectangle_padding():
    assert point_inside_rectangle(
        0.12,
        0.0,
        min_x=-0.10,
        max_x=0.10,
        min_y=-0.0995,
        max_y=0.0995,
        padding=0.025,
    )
    assert not point_inside_rectangle(
        0.13,
        0.0,
        min_x=-0.10,
        max_x=0.10,
        min_y=-0.0995,
        max_y=0.0995,
        padding=0.025,
    )


def test_yaw_from_quaternion():
    yaw = yaw_from_quaternion(0.0, 0.0, 1.0, 0.0)
    assert math.isclose(abs(yaw), math.pi, abs_tol=1e-9)


def test_filter_removes_only_endpoints_inside_padded_chassis():
    transform = PlanarTransform(x=0.0662, y=0.0, yaw=math.pi)
    filtered, removed = filter_ranges_inside_rectangle(
        [0.1762, 0.30],
        angle_min=0.0,
        angle_increment=0.0,
        transform=transform,
        min_x=-0.0878,
        max_x=0.1128,
        min_y=-0.0995,
        max_y=0.0995,
        padding=0.025,
    )
    assert math.isinf(filtered[0])
    assert math.isclose(filtered[1], 0.30)
    assert removed == [0]
