from __future__ import annotations

from dataclasses import dataclass

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

from ros2_arduino_bridge.arduino_bridge import Arduino_bridge


@dataclass
class _ArduinoData:
    left_delta: int = 0
    right_delta: int = 0
    left_speed: float = 0.0
    right_speed: float = 0.0


class FakeConnection:
    def __init__(self):
        self.speed_calls: list[tuple[float, float]] = []
        self.forward_calls = []
        self.turn_calls = []
        self.closed = False

    def setSpeeds(self, linear, angular):
        self.speed_calls.append((float(linear), float(angular)))

    def get_data(self):
        return _ArduinoData()

    def go_dist(self, dist, speed):
        self.forward_calls.append((dist, speed))
        return True

    def turn_robot(self, angle, speed):
        self.turn_calls.append((angle, speed))
        return True

    def close(self):
        self.closed = True


@pytest.fixture(scope="module", autouse=True)
def _ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def make_node(timeout=0.30):
    connection = FakeConnection()
    node = Arduino_bridge(
        connection,
        parameter_overrides=[
            Parameter(
                "cmd_vel_timeout_s",
                Parameter.Type.DOUBLE,
                timeout,
            ),
            Parameter(
                "cmd_vel_watchdog_period_s",
                Parameter.Type.DOUBLE,
                min(0.05, timeout / 2.0),
            ),
        ],
    )
    return node, connection


def nonzero_twist():
    msg = Twist()
    msg.linear.x = 0.10
    return msg


def test_default_timeout_is_300_ms_and_legacy_services_disabled():
    node, _connection = make_node()
    try:
        assert node.cmd_vel_timeout_s == pytest.approx(0.30)
        assert node.allow_legacy_motion_services is False
    finally:
        node.destroy_node()


def test_nonzero_command_is_forwarded_then_stale_stream_forces_zero_once():
    node, connection = make_node()
    try:
        node.cmd_vel_callback(nonzero_twist())
        assert connection.speed_calls[-1] == pytest.approx((0.10, 0.0))
        assert node._last_cmd_vel_was_nonzero is True

        # Avoid sleeps: age the receive timestamp directly and invoke watchdog.
        node._last_cmd_vel_monotonic -= 1.0
        node._cmd_vel_watchdog()

        assert connection.speed_calls[-1] == pytest.approx((0.0, 0.0))
        assert node._last_cmd_vel_was_nonzero is False
        call_count = len(connection.speed_calls)

        # Stale state must not spam repeated stop commands.
        node._cmd_vel_watchdog()
        assert len(connection.speed_calls) == call_count
    finally:
        node.destroy_node()


def test_fresh_nonzero_command_does_not_trip_watchdog():
    node, connection = make_node(timeout=0.30)
    try:
        node.cmd_vel_callback(nonzero_twist())
        call_count = len(connection.speed_calls)
        node._cmd_vel_watchdog()
        assert len(connection.speed_calls) == call_count
        assert connection.speed_calls[-1] == pytest.approx((0.10, 0.0))
    finally:
        node.destroy_node()


def test_explicit_zero_disarms_timeout():
    node, connection = make_node()
    try:
        node.cmd_vel_callback(nonzero_twist())
        zero = Twist()
        node.cmd_vel_callback(zero)
        assert connection.speed_calls[-1] == pytest.approx((0.0, 0.0))
        assert node._last_cmd_vel_was_nonzero is False

        node._last_cmd_vel_monotonic -= 1.0
        call_count = len(connection.speed_calls)
        node._cmd_vel_watchdog()
        assert len(connection.speed_calls) == call_count
    finally:
        node.destroy_node()


def test_legacy_direct_motion_services_are_blocked_by_default():
    node, connection = make_node()
    try:
        class Req:
            dist = 10.0
            speed = 20.0
            angle = 30.0

        class Resp:
            success = True

        response = node.handle_forward_request(Req(), Resp())
        assert response.success is False
        assert connection.forward_calls == []

        response = node.handle_rotate_request(Req(), Resp())
        assert response.success is False
        assert connection.turn_calls == []
    finally:
        node.destroy_node()


def test_shutdown_forces_zero_before_serial_close():
    node, connection = make_node()
    node.cmd_vel_callback(nonzero_twist())
    node.shutdown()
    assert connection.speed_calls[-1] == pytest.approx((0.0, 0.0))
    assert connection.closed is True
