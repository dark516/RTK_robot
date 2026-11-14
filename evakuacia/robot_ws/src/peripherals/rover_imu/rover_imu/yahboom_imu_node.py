"""Native ROS 2 driver for the Yahboom/YB-MRA02 IMU USB protocol."""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import UInt64

from .yahboom_imu_protocol import (
    TYPE_ACCELERATION,
    TYPE_EULER,
    TYPE_GYROSCOPE,
    TYPE_MAGNETIC,
    TYPE_YB_MRA02_EULER,
    TYPE_YB_MRA02_QUATERNION,
    TYPE_YB_MRA02_RAW,
    YahboomFrameParser,
    decode_acceleration,
    decode_euler,
    decode_gyroscope,
    decode_magnetic_raw,
    decode_yb_mra02_euler,
    decode_yb_mra02_quaternion,
    decode_yb_mra02_raw,
    quaternion_from_euler,
    remap_vector,
)


def diagonal_covariance(x: float, y: float, z: float) -> list[float]:
    return [
        x*x, 0.0, 0.0,
        0.0, y*y, 0.0,
        0.0, 0.0, z*z,
    ]


class YahboomImuNode(Node):
    def __init__(self) -> None:
        super().__init__("yahboom_imu_node")

        self.declare_parameter("serial_device", "/tmp/rover_devices/imu")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("mag_topic", "/imu/mag")
        self.declare_parameter("euler_topic", "/imu/euler")
        self.declare_parameter("frame_count_topic", "/imu/valid_frame_count")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("data_timeout_sec", 0.50)

        # Keep orientation disabled in EKF until its world convention and
        # magnetic calibration are verified. Gyro Z is safe to use first.
        self.declare_parameter("publish_sensor_orientation", False)

        # Sensor vectors are expressed in imu_link. Normally leave these at
        # identity and describe physical mounting using base_link->imu_link TF.
        self.declare_parameter("axis_map", [0, 1, 2])
        self.declare_parameter("axis_signs", [1, 1, 1])

        self.declare_parameter("gyro_stddev_radps", 0.035)
        self.declare_parameter("accel_stddev_mps2", 0.50)
        self.declare_parameter("orientation_roll_pitch_stddev_rad", 0.10)
        self.declare_parameter("orientation_yaw_stddev_rad", 0.35)

        # The official basic example exposes raw magnetic channels but does
        # not specify a universal calibrated scale. Default assumes 1 mG/LSB.
        # The magnetometer is diagnostic only and is not fused by EKF.
        self.declare_parameter("magnetic_scale_tesla_per_lsb", 1.0e-7)
        self.declare_parameter("magnetic_stddev_tesla", 8.0e-6)

        self.serial_device = str(self.get_parameter("serial_device").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.data_timeout = float(self.get_parameter("data_timeout_sec").value)
        self.publish_orientation = bool(
            self.get_parameter("publish_sensor_orientation").value
        )
        self.axis_map = tuple(self.get_parameter("axis_map").value)
        self.axis_signs = tuple(self.get_parameter("axis_signs").value)
        self.mag_scale = float(
            self.get_parameter("magnetic_scale_tesla_per_lsb").value
        )

        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if self.data_timeout <= 0.0:
            raise ValueError("data_timeout_sec must be positive")

        # Validate axis parameters immediately.
        remap_vector((1.0, 2.0, 3.0), self.axis_map, self.axis_signs)

        gyro_std = float(self.get_parameter("gyro_stddev_radps").value)
        accel_std = float(self.get_parameter("accel_stddev_mps2").value)
        rp_std = float(
            self.get_parameter("orientation_roll_pitch_stddev_rad").value
        )
        yaw_std = float(
            self.get_parameter("orientation_yaw_stddev_rad").value
        )
        mag_std = float(
            self.get_parameter("magnetic_stddev_tesla").value
        )
        for name, value in (
            ("gyro_stddev_radps", gyro_std),
            ("accel_stddev_mps2", accel_std),
            ("orientation_roll_pitch_stddev_rad", rp_std),
            ("orientation_yaw_stddev_rad", yaw_std),
            ("magnetic_stddev_tesla", mag_std),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        self.gyro_covariance = diagonal_covariance(
            gyro_std, gyro_std, gyro_std
        )
        self.accel_covariance = diagonal_covariance(
            accel_std, accel_std, accel_std
        )
        self.orientation_covariance = diagonal_covariance(
            rp_std, rp_std, yaw_std
        )
        self.magnetic_covariance = diagonal_covariance(
            mag_std, mag_std, mag_std
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            str(self.get_parameter("imu_topic").value),
            qos_profile_sensor_data,
        )
        self.mag_publisher = self.create_publisher(
            MagneticField,
            str(self.get_parameter("mag_topic").value),
            qos_profile_sensor_data,
        )
        self.euler_publisher = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("euler_topic").value),
            qos_profile_sensor_data,
        )
        self.frame_count_publisher = self.create_publisher(
            UInt64,
            str(self.get_parameter("frame_count_topic").value),
            10,
        )

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._parser = YahboomFrameParser()
        self._acceleration: Optional[tuple[float, float, float]] = None
        self._gyro: Optional[tuple[float, float, float]] = None
        self._euler: Optional[tuple[float, float, float]] = None
        self._orientation_quaternion: Optional[
            tuple[float, float, float, float]
        ] = None
        self._magnetic: Optional[tuple[float, float, float]] = None
        self._last_accel_time = 0.0
        self._last_gyro_time = 0.0
        self._last_euler_time = 0.0
        self._last_quaternion_time = 0.0
        self._last_mag_time = 0.0
        self._last_warning_time = 0.0
        self._last_mag_publish_time = 0.0
        self._serial_fault: Optional[str] = None

        self._serial = serial.Serial(
            port=self.serial_device,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.5,
        )
        time.sleep(0.15)
        self._serial.reset_input_buffer()

        self._reader = threading.Thread(
            target=self._reader_loop,
            name="yahboom_imu_serial_reader",
            daemon=True,
        )
        self._reader.start()

        self.create_timer(1.0 / self.publish_rate, self._publish_latest)
        self.create_timer(1.0, self._publish_frame_count)

        self.get_logger().info(
            f"Yahboom/YB-MRA02 IMU connected: {self.serial_device} "
            f"@ {self.baudrate}; output /imu/data"
        )
        self.get_logger().info(
            "EKF-safe mode: sensor orientation is "
            + ("enabled" if self.publish_orientation else "disabled; gyro Z only")
        )

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                data = self._serial.read(512)
            except serial.SerialException as exc:
                with self._lock:
                    self._serial_fault = str(exc)
                return

            if not data:
                continue

            now = time.monotonic()
            frames = self._parser.feed(data)
            if not frames:
                continue

            with self._lock:
                for frame in frames:
                    if frame.protocol == 'yb_mra02_v1':
                        if frame.frame_type == TYPE_YB_MRA02_RAW:
                            acceleration, gyro, magnetic = decode_yb_mra02_raw(
                                frame.payload
                            )
                            self._acceleration = acceleration
                            self._gyro = gyro
                            self._magnetic = tuple(
                                value / self.mag_scale for value in magnetic
                            )
                            self._last_accel_time = now
                            self._last_gyro_time = now
                            self._last_mag_time = now
                        elif frame.frame_type == TYPE_YB_MRA02_EULER:
                            self._euler = decode_yb_mra02_euler(frame.payload)
                            self._last_euler_time = now
                        elif frame.frame_type == TYPE_YB_MRA02_QUATERNION:
                            self._orientation_quaternion = (
                                decode_yb_mra02_quaternion(frame.payload)
                            )
                            self._last_quaternion_time = now
                        continue

                    if frame.frame_type == TYPE_ACCELERATION:
                        self._acceleration = decode_acceleration(frame.payload)
                        self._last_accel_time = now
                    elif frame.frame_type == TYPE_GYROSCOPE:
                        self._gyro = decode_gyroscope(frame.payload)
                        self._last_gyro_time = now
                    elif frame.frame_type == TYPE_EULER:
                        self._euler = decode_euler(frame.payload)
                        self._last_euler_time = now
                    elif frame.frame_type == TYPE_MAGNETIC:
                        self._magnetic = decode_magnetic_raw(frame.payload)
                        self._last_mag_time = now

    def _publish_latest(self) -> None:
        now_mono = time.monotonic()
        with self._lock:
            acceleration = self._acceleration
            gyro = self._gyro
            euler = self._euler
            magnetic = self._magnetic
            orientation_quaternion = self._orientation_quaternion
            accel_time = self._last_accel_time
            gyro_time = self._last_gyro_time
            euler_time = self._last_euler_time
            quaternion_time = self._last_quaternion_time
            mag_time = self._last_mag_time
            serial_fault = self._serial_fault

        if serial_fault:
            if now_mono - self._last_warning_time > 2.0:
                self.get_logger().error(f"IMU serial fault: {serial_fault}")
                self._last_warning_time = now_mono
            return

        if acceleration is None or gyro is None:
            if now_mono - self._last_warning_time > 2.0:
                self.get_logger().warning(
                    "Waiting for valid IMU acceleration and gyro frames"
                )
                self._last_warning_time = now_mono
            return

        newest_required = min(accel_time, gyro_time)
        if now_mono - newest_required > self.data_timeout:
            if now_mono - self._last_warning_time > 2.0:
                self.get_logger().warning(
                    "Yahboom IMU data is stale; suppressing /imu/data"
                )
                self._last_warning_time = now_mono
            return

        acceleration = remap_vector(
            acceleration, self.axis_map, self.axis_signs
        )
        gyro = remap_vector(gyro, self.axis_map, self.axis_signs)

        stamp = self.get_clock().now().to_msg()
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id

        message.linear_acceleration.x = acceleration[0]
        message.linear_acceleration.y = acceleration[1]
        message.linear_acceleration.z = acceleration[2]
        message.linear_acceleration_covariance = list(self.accel_covariance)

        message.angular_velocity.x = gyro[0]
        message.angular_velocity.y = gyro[1]
        message.angular_velocity.z = gyro[2]
        message.angular_velocity_covariance = list(self.gyro_covariance)

        if (
            self.publish_orientation
            and orientation_quaternion is not None
            and now_mono - quaternion_time <= self.data_timeout
        ):
            (
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ) = orientation_quaternion
            message.orientation_covariance = list(
                self.orientation_covariance
            )
        elif (
            self.publish_orientation
            and euler is not None
            and now_mono - euler_time <= self.data_timeout
        ):
            # This assumes the sensor's Euler world convention has been
            # validated for the installation. It is disabled by default.
            roll, pitch, yaw = euler
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
            message.orientation.x = qx
            message.orientation.y = qy
            message.orientation.z = qz
            message.orientation.w = qw
            message.orientation_covariance = list(
                self.orientation_covariance
            )
        else:
            message.orientation.w = 1.0
            message.orientation_covariance[0] = -1.0

        self.imu_publisher.publish(message)

        if euler is not None and now_mono - euler_time <= self.data_timeout:
            euler_message = Vector3Stamped()
            euler_message.header = message.header
            euler_message.vector.x = euler[0]
            euler_message.vector.y = euler[1]
            euler_message.vector.z = euler[2]
            self.euler_publisher.publish(euler_message)

        if (
            magnetic is not None
            and now_mono - mag_time <= self.data_timeout
            and mag_time != self._last_mag_publish_time
        ):
            mag_vector = remap_vector(
                tuple(float(value) for value in magnetic),
                self.axis_map,
                self.axis_signs,
            )
            mag_message = MagneticField()
            mag_message.header = message.header
            mag_message.magnetic_field.x = mag_vector[0] * self.mag_scale
            mag_message.magnetic_field.y = mag_vector[1] * self.mag_scale
            mag_message.magnetic_field.z = mag_vector[2] * self.mag_scale
            mag_message.magnetic_field_covariance = list(
                self.magnetic_covariance
            )
            self.mag_publisher.publish(mag_message)
            self._last_mag_publish_time = mag_time

    def _publish_frame_count(self) -> None:
        message = UInt64()
        message.data = self._parser.valid_frames
        self.frame_count_publisher.publish(message)

    def close(self) -> None:
        self._stop_event.set()
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if self._serial.is_open:
            self._serial.close()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[YahboomImuNode] = None
    try:
        node = YahboomImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
