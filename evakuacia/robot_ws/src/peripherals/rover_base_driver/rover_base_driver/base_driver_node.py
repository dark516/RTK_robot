from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray

from rover_interfaces.msg import WheelCommand, WheelEncoders

from .kinematics import inverse_mecanum, scale_wheels
from .quad_md_protocol import QuadMdProtocol


def move_towards(current: float, target: float, step: float) -> float:
    error = target - current
    if abs(error) <= step:
        return target
    return current + math.copysign(step, error)


class BaseDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('base_driver_node')

        self.declare_parameter('serial_device', '/tmp/rover_devices/motor_controller')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheelbase_m', 0.135)
        self.declare_parameter('track_width_m', 0.195)
        self.declare_parameter('wheel_radius_m', 0.03)
        self.declare_parameter('encoder_lines', 11.0)
        self.declare_parameter('reduction_ratio', 45.0)
        self.declare_parameter('quadrature_factor', 4.0)
        self.declare_parameter('motor_command_order', [0, 1, 2, 3])
        self.declare_parameter('motor_command_signs', [1, -1, -1, 1])
        self.declare_parameter('encoder_feedback_order', [0, 1, 2, 3])
        self.declare_parameter('encoder_feedback_signs', [1, -1, -1, 1])
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.50)
        self.declare_parameter('feedback_timeout_sec', 0.35)
        self.declare_parameter('max_wheel_speed_mps', 0.30)
        self.declare_parameter('hold_position_on_zero_cmd', True)
        self.declare_parameter('stale_brake_hold_sec', 0.35)
        self.declare_parameter('max_accel_x_mps2', 0.25)
        self.declare_parameter('max_decel_x_mps2', 0.50)
        self.declare_parameter('max_accel_y_mps2', 0.18)
        self.declare_parameter('max_decel_y_mps2', 0.35)
        self.declare_parameter('max_accel_z_radps2', 0.80)
        self.declare_parameter('max_decel_z_radps2', 1.50)
        self.declare_parameter('max_jerk_x_mps3', 0.70)
        self.declare_parameter('max_jerk_y_mps3', 0.45)
        self.declare_parameter('max_jerk_z_radps3', 2.00)

        self.device = str(self.get_parameter('serial_device').value)
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.track_width = float(self.get_parameter('track_width_m').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.counts_per_rev = (
            float(self.get_parameter('encoder_lines').value)
            * float(self.get_parameter('reduction_ratio').value)
            * float(self.get_parameter('quadrature_factor').value)
        )
        self.rate = float(self.get_parameter('control_rate_hz').value)
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)
        self.feedback_timeout = float(self.get_parameter('feedback_timeout_sec').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed_mps').value)
        self.hold_on_zero = bool(self.get_parameter('hold_position_on_zero_cmd').value)
        self.stale_brake_hold = float(
            self.get_parameter('stale_brake_hold_sec').value
        )
        if self.stale_brake_hold < 0.0:
            raise ValueError('stale_brake_hold_sec must be non-negative')
        self.accel = [
            float(self.get_parameter('max_accel_x_mps2').value),
            float(self.get_parameter('max_accel_y_mps2').value),
            float(self.get_parameter('max_accel_z_radps2').value),
        ]
        self.decel = [
            float(self.get_parameter('max_decel_x_mps2').value),
            float(self.get_parameter('max_decel_y_mps2').value),
            float(self.get_parameter('max_decel_z_radps2').value),
        ]
        self.jerk = [
            float(self.get_parameter('max_jerk_x_mps3').value),
            float(self.get_parameter('max_jerk_y_mps3').value),
            float(self.get_parameter('max_jerk_z_radps3').value),
        ]

        self.protocol = QuadMdProtocol(
            self.device,
            int(self.get_parameter('baudrate').value),
            list(self.get_parameter('motor_command_order').value),
            list(self.get_parameter('motor_command_signs').value),
            list(self.get_parameter('encoder_feedback_order').value),
            list(self.get_parameter('encoder_feedback_signs').value),
        )

        self.target = [0.0, 0.0, 0.0]
        self.current = [0.0, 0.0, 0.0]
        self.current_accel = [0.0, 0.0, 0.0]
        now = time.monotonic()
        self.last_cmd = now
        self.last_loop = now
        self.last_sequence = 0
        self.last_feedback_time = now
        self.last_battery_request = 0.0
        self.released = True
        self.stale_since = None

        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self._cmd,
            10,
        )
        self.command_pub = self.create_publisher(
            WheelCommand, '/drive/wheel_commands', 10
        )
        self.encoder_pub = self.create_publisher(
            WheelEncoders, '/wheel/encoders', 20
        )
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 20)
        self.speed_pub = self.create_publisher(
            Float64MultiArray, '/drive/wheel_speeds/measured', 20
        )
        self.battery_pub = self.create_publisher(Float32, '/battery_voltage', 10)

        self.timer = self.create_timer(1.0 / self.rate, self._loop)
        self.get_logger().info(
            f'Base driver connected to {self.device}; built-in board speed PID is active'
        )

    def _cmd(self, message: Twist) -> None:
        values = [message.linear.x, message.linear.y, message.angular.z]
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error('Ignored non-finite cmd_vel')
            return
        self.target = [float(v) for v in values]
        self.last_cmd = time.monotonic()

    def _profile_axis(self, i: int, target: float, dt: float) -> None:
        current = self.current[i]
        same_direction = current == 0.0 or target == 0.0 or current * target > 0.0
        speeding_up = same_direction and abs(target) > abs(current)
        limit = self.accel[i] if speeding_up else self.decel[i]
        desired_accel = max(-limit, min(limit, (target - current) / dt))
        accel = move_towards(
            self.current_accel[i], desired_accel, self.jerk[i] * dt
        )
        new_value = current + accel * dt
        if (target - current) * (target - new_value) <= 0.0:
            new_value = target
            accel = 0.0
        self.current[i] = new_value
        self.current_accel[i] = accel

    def _loop(self) -> None:
        now = time.monotonic()
        dt = max(0.001, min(0.1, now - self.last_loop))
        self.last_loop = now

        stale = now - self.last_cmd > self.command_timeout
        if stale:
            # A lost cmd_vel stream is a safety event. First command an active
            # zero-speed stop so the onboard PID brakes the rover, then release
            # the motors after a short hold interval.
            self.target = [0.0, 0.0, 0.0]
            self.current = [0.0, 0.0, 0.0]
            self.current_accel = [0.0, 0.0, 0.0]
            target_wheels = (0.0, 0.0, 0.0, 0.0)
            board_command = (0, 0, 0, 0)

            if self.stale_since is None:
                self.stale_since = now
                self.protocol.hold_stop()
                self.released = False
            elif (
                not self.released
                and now - self.stale_since >= self.stale_brake_hold
            ):
                self.protocol.release()
                self.released = True
        else:
            self.stale_since = None
            for index in range(3):
                self._profile_axis(index, self.target[index], dt)
            target_wheels = scale_wheels(
                inverse_mecanum(
                    self.current[0], self.current[1], self.current[2],
                    self.wheelbase, self.track_width,
                ),
                self.max_wheel_speed,
            )
            if max(abs(v) for v in target_wheels) < 1e-6:
                if self.hold_on_zero:
                    self.protocol.hold_stop()
                    self.released = False
                else:
                    self.protocol.release()
                    self.released = True
                board_command = (0, 0, 0, 0)
            else:
                board_command = self.protocol.command_speed(target_wheels)
                self.released = False

        stamp = self.get_clock().now().to_msg()
        command_msg = WheelCommand()
        command_msg.header.stamp = stamp
        command_msg.header.frame_id = 'base_link'
        command_msg.target_mps = list(target_wheels)
        command_msg.board_command_mm_s = list(board_command)
        self.command_pub.publish(command_msg)

        sample = self.protocol.sample()
        if sample is not None and sample.sequence != self.last_sequence:
            self.last_sequence = sample.sequence
            self.last_feedback_time = now

            encoder = WheelEncoders()
            encoder.header.stamp = stamp
            encoder.header.frame_id = 'base_link'
            encoder.total_counts = list(sample.counts)
            encoder.measured_mps = list(sample.measured_mps)
            encoder.sequence = sample.sequence
            encoder.valid = True
            self.encoder_pub.publish(encoder)

            speed = Float64MultiArray()
            speed.data = list(sample.measured_mps)
            self.speed_pub.publish(speed)

            joint = JointState()
            joint.header.stamp = stamp
            joint.name = [
                'front_left_wheel_joint', 'front_right_wheel_joint',
                'rear_left_wheel_joint', 'rear_right_wheel_joint',
            ]
            joint.position = [
                count / self.counts_per_rev * 2.0 * math.pi
                for count in sample.counts
            ]
            joint.velocity = [v / self.wheel_radius for v in sample.measured_mps]
            self.joint_pub.publish(joint)

        if not self.released and now - self.last_feedback_time > self.feedback_timeout:
            self.get_logger().error('Encoder feedback timeout; releasing motors')
            self.protocol.release()
            self.released = True

        if now - self.last_battery_request > 5.0:
            self.protocol.request_battery()
            self.last_battery_request = now
        battery = self.protocol.battery()
        if battery is not None:
            message = Float32()
            message.data = float(battery)
            self.battery_pub.publish(message)

    def close(self) -> None:
        self.protocol.close()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[BaseDriverNode] = None
    try:
        node = BaseDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
