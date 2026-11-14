import pytest
from builtin_interfaces.msg import Time
from frob_city_interfaces.msg import RouteSegment
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import Path

from frob_city_navigation.ros_execution_conversions import (
    build_motion_permit_msg,
    route_segment_to_nav_path,
)


def segment():
    s = RouteSegment()
    s.segment_id = 'SEG_1'
    s.path.header.frame_id = 'map'
    for x in (0.0, 1.0):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = x
        p.pose.orientation.w = 1.0
        s.path.poses.append(p)
    return s


def test_route_segment_to_nav_path_is_exact_detached_copy():
    s = segment()
    path = route_segment_to_nav_path(s, map_frame='map')
    assert isinstance(path, Path)
    assert path is not s.path
    assert [p.pose.position.x for p in path.poses] == [0.0, 1.0]
    path.poses[-1].pose.position.x = 99.0
    assert s.path.poses[-1].pose.position.x == 1.0


def test_route_segment_to_nav_path_rejects_empty_or_wrong_frame():
    s = segment()
    s.path.header.frame_id = 'odom'
    with pytest.raises(ValueError):
        route_segment_to_nav_path(s, map_frame='map')
    s = segment()
    s.path.poses.clear()
    with pytest.raises(ValueError):
        route_segment_to_nav_path(s, map_frame='map')


def test_enabled_motion_permit_requires_segment_and_generation():
    t = Time(sec=1)
    with pytest.raises(ValueError):
        build_motion_permit_msg(
            now_msg=t, run_id='r', map_instance_id='m', navigator_instance_id='n',
            field_id='f', goal_generation=1, sequence=1, enabled=True,
            valid_for_s=0.2, segment_msg=None,
        )
    msg = build_motion_permit_msg(
        now_msg=t, run_id='r', map_instance_id='m', navigator_instance_id='n',
        field_id='f', goal_generation=2, sequence=7, enabled=True,
        valid_for_s=0.2, segment_msg=segment(),
    )
    assert msg.enabled
    assert msg.goal_generation == 2
    assert msg.sequence == 7
    assert msg.segment.segment_id == 'SEG_1'


def test_disabled_motion_permit_does_not_require_segment():
    msg = build_motion_permit_msg(
        now_msg=Time(sec=1), run_id='r', map_instance_id='m', navigator_instance_id='n',
        field_id='f', goal_generation=0, sequence=1, enabled=False,
        valid_for_s=0.2, segment_msg=None,
    )
    assert not msg.enabled
