from __future__ import annotations

import math
from typing import Optional, Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from rover_interfaces.msg import WheelEncoders


def diagonal36(values: Sequence[float]) -> list[float]:
    if len(values) != 6:
        raise ValueError('covariance diagonal must have six values')
    result = [0.0] * 36
    for i, value in enumerate(values):
        result[i * 6 + i] = float(value)
    return result


def counter_delta(current: int, previous: int) -> int:
    delta = current - previous
    if delta > (1 << 31):
        delta -= 1 << 32
    elif delta < -(1 << 31):
        delta += 1 << 32
    return delta


def forward_mecanum(wheels: Sequence[float], wheelbase: float, track: float):
    fl, fr, rl, rr = (float(v) for v in wheels)
    k = (wheelbase + track) / 2.0
    return (
        (fl + fr + rl + rr) / 4.0,
        (-fl + fr + rl - rr) / 4.0,
        (-fl + fr - rl + rr) / (4.0 * k),
    )


class WheelOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odometry_node')
        self.declare_parameter('encoder_topic', '/wheel/encoders')
        self.declare_parameter('odometry_topic', '/wheel/odometry')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('wheel_radius_m', 0.03)
        self.declare_parameter('wheelbase_m', 0.135)
        self.declare_parameter('track_width_m', 0.195)
        self.declare_parameter('encoder_lines', 11.0)
        self.declare_parameter('reduction_ratio', 45.0)
        self.declare_parameter('quadrature_factor', 4.0)
        self.declare_parameter('x_multiplier', 1.0)
        self.declare_parameter('y_multiplier', 1.0)
        self.declare_parameter('yaw_multiplier', 1.0)
        self.declare_parameter('max_sample_gap_sec', 0.5)
        self.declare_parameter('max_plausible_wheel_speed_mps', 1.5)
        self.declare_parameter(
            'pose_covariance_diagonal',
            [0.03, 0.08, 999.0, 999.0, 999.0, 0.15],
        )
        self.declare_parameter(
            'twist_covariance_diagonal',
            [0.02, 0.06, 999.0, 999.0, 999.0, 0.12],
        )

        radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.track = float(self.get_parameter('track_width_m').value)
        counts_per_rev = (
            float(self.get_parameter('encoder_lines').value)
            * float(self.get_parameter('reduction_ratio').value)
            * float(self.get_parameter('quadrature_factor').value)
        )
        self.metres_per_count = 2.0 * math.pi * radius / counts_per_rev
        self.multipliers = (
            float(self.get_parameter('x_multiplier').value),
            float(self.get_parameter('y_multiplier').value),
            float(self.get_parameter('yaw_multiplier').value),
        )
        self.max_gap = float(self.get_parameter('max_sample_gap_sec').value)
        self.max_speed = float(
            self.get_parameter('max_plausible_wheel_speed_mps').value
        )
        self.odom_frame = str(self.get_parameter('odom_frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame_id').value)
        self.pose_cov = diagonal36(
            list(self.get_parameter('pose_covariance_diagonal').value)
        )
        self.twist_cov = diagonal36(
            list(self.get_parameter('twist_covariance_diagonal').value)
        )

        self.previous_counts: Optional[tuple[int, int, int, int]] = None
        self.previous_stamp_ns: Optional[int] = None
        self.previous_sequence: Optional[int] = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter('odometry_topic').value), 20
        )
        self.create_subscription(
            WheelEncoders,
            str(self.get_parameter('encoder_topic').value),
            self._encoder,
            20,
        )
        self.get_logger().info(
            f'Wheel odometry uses accumulated counts; {self.metres_per_count:.9f} m/count'
        )

    def _encoder(self, message: WheelEncoders) -> None:
        if not message.valid:
            return
        counts = tuple(int(v) for v in message.total_counts)
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if self.previous_counts is None or self.previous_stamp_ns is None:
            self.previous_counts = counts  # type: ignore[assignment]
            self.previous_stamp_ns = stamp_ns
            self.previous_sequence = int(message.sequence)
            return

        dt = (stamp_ns - self.previous_stamp_ns) / 1e9
        old_counts = self.previous_counts
        self.previous_counts = counts  # type: ignore[assignment]
        self.previous_stamp_ns = stamp_ns
        self.previous_sequence = int(message.sequence)
        if dt <= 0.0 or dt > self.max_gap:
            self.get_logger().warning(f'Reset encoder baseline after {dt:.3f}s sample gap')
            return

        deltas = tuple(counter_delta(c, p) for c, p in zip(counts, old_counts))
        wheel_delta = tuple(delta * self.metres_per_count for delta in deltas)
        if any(abs(distance / dt) > self.max_speed for distance in wheel_delta):
            self.get_logger().warning(f'Rejected implausible encoder jump: {deltas}')
            return

        dx, dy, dyaw = forward_mecanum(wheel_delta, self.wheelbase, self.track)
        dx *= self.multipliers[0]
        dy *= self.multipliers[1]
        dyaw *= self.multipliers[2]
        mid_yaw = self.yaw + dyaw / 2.0
        self.x += dx * math.cos(mid_yaw) - dy * math.sin(mid_yaw)
        self.y += dx * math.sin(mid_yaw) + dy * math.cos(mid_yaw)
        self.yaw = math.atan2(
            math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw)
        )

        odom = Odometry()
        odom.header = message.header
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.pose.covariance = self.pose_cov
        odom.twist.twist.linear.x = dx / dt
        odom.twist.twist.linear.y = dy / dt
        odom.twist.twist.angular.z = dyaw / dt
        odom.twist.covariance = self.twist_cov
        self.publisher.publish(odom)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
