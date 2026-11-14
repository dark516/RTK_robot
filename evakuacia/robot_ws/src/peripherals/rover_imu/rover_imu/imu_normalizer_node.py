from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def covariance(x: float, y: float, z: float) -> list[float]:
    return [x*x, 0.0, 0.0, 0.0, y*y, 0.0, 0.0, 0.0, z*z]


def zero_covariance(values) -> bool:
    return all(abs(float(v)) < 1e-15 for v in values)


class ImuNormalizerNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_normalizer_node')
        self.declare_parameter('input_topic', '/imu/raw')
        self.declare_parameter('output_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('override_frame_id', True)
        self.declare_parameter('gyro_stddev_radps', 0.035)
        self.declare_parameter('accel_stddev_mps2', 0.50)
        self.declare_parameter('orientation_roll_pitch_stddev_rad', 0.10)
        self.declare_parameter('orientation_yaw_stddev_rad', 0.25)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.override_frame = bool(self.get_parameter('override_frame_id').value)
        gyro = float(self.get_parameter('gyro_stddev_radps').value)
        accel = float(self.get_parameter('accel_stddev_mps2').value)
        rp = float(self.get_parameter('orientation_roll_pitch_stddev_rad').value)
        yaw = float(self.get_parameter('orientation_yaw_stddev_rad').value)
        for value in (gyro, accel, rp, yaw):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('IMU standard deviations must be positive')
        self.gyro_cov = covariance(gyro, gyro, gyro)
        self.accel_cov = covariance(accel, accel, accel)
        self.orientation_cov = covariance(rp, rp, yaw)

        self.publisher = self.create_publisher(
            Imu, str(self.get_parameter('output_topic').value), qos_profile_sensor_data
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('input_topic').value),
            self._imu,
            qos_profile_sensor_data,
        )

    def _imu(self, source: Imu) -> None:
        message = Imu()
        message.header = source.header
        if self.override_frame or not message.header.frame_id:
            message.header.frame_id = self.frame_id
        message.orientation = source.orientation
        message.angular_velocity = source.angular_velocity
        message.linear_acceleration = source.linear_acceleration
        message.orientation_covariance = list(source.orientation_covariance)
        message.angular_velocity_covariance = list(source.angular_velocity_covariance)
        message.linear_acceleration_covariance = list(source.linear_acceleration_covariance)
        if zero_covariance(message.orientation_covariance):
            message.orientation_covariance = list(self.orientation_cov)
        if zero_covariance(message.angular_velocity_covariance):
            message.angular_velocity_covariance = list(self.gyro_cov)
        if zero_covariance(message.linear_acceleration_covariance):
            message.linear_acceleration_covariance = list(self.accel_cov)
        self.publisher.publish(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ImuNormalizerNode()
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
