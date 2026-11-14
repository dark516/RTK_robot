from __future__ import annotations

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import (
    PlanarTransform,
    filter_ranges_inside_rectangle,
    yaw_from_quaternion,
)


class LidarFootprintFilterNode(Node):
    """Remove LaserScan returns that fall inside the rover chassis footprint."""

    def __init__(self) -> None:
        super().__init__('lidar_footprint_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('footprint_min_x_m', -0.0878)
        self.declare_parameter('footprint_max_x_m', 0.1128)
        self.declare_parameter('footprint_min_y_m', -0.0995)
        self.declare_parameter('footprint_max_y_m', 0.0995)
        self.declare_parameter('padding_m', 0.025)
        self.declare_parameter('use_tf', True)
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('fallback_sensor_x_m', 0.0662)
        self.declare_parameter('fallback_sensor_y_m', 0.0)
        self.declare_parameter('fallback_sensor_yaw_rad', math.pi)
        self.declare_parameter('warn_interval_sec', 5.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.min_x = float(self.get_parameter('footprint_min_x_m').value)
        self.max_x = float(self.get_parameter('footprint_max_x_m').value)
        self.min_y = float(self.get_parameter('footprint_min_y_m').value)
        self.max_y = float(self.get_parameter('footprint_max_y_m').value)
        self.padding = max(0.0, float(self.get_parameter('padding_m').value))
        self.use_tf = bool(self.get_parameter('use_tf').value)
        self.tf_timeout = max(
            0.0,
            float(self.get_parameter('tf_timeout_sec').value),
        )
        self.warn_interval = max(
            0.1,
            float(self.get_parameter('warn_interval_sec').value),
        )
        self.fallback_transform = PlanarTransform(
            x=float(self.get_parameter('fallback_sensor_x_m').value),
            y=float(self.get_parameter('fallback_sensor_y_m').value),
            yaw=float(self.get_parameter('fallback_sensor_yaw_rad').value),
        )

        if self.min_x >= self.max_x or self.min_y >= self.max_y:
            raise ValueError(
                'Invalid footprint bounds: min values must be below max values'
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self._last_tf_warning_monotonic = 0.0
        self._scan_count = 0
        self._filtered_count = 0

        self.get_logger().info(
            'Filtering %s -> %s; footprint x=[%.3f, %.3f], y=[%.3f, %.3f], '
            'padding=%.3f m, base_frame=%s'
            % (
                self.input_topic,
                self.output_topic,
                self.min_x,
                self.max_x,
                self.min_y,
                self.max_y,
                self.padding,
                self.base_frame_id,
            )
        )

    def _lookup_scan_to_base(self, scan: LaserScan) -> PlanarTransform:
        source_frame = scan.header.frame_id.strip()
        if not self.use_tf or not source_frame or source_frame == self.base_frame_id:
            if source_frame == self.base_frame_id:
                return PlanarTransform(0.0, 0.0, 0.0)
            return self.fallback_transform

        try:
            stamp = Time.from_msg(scan.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame_id,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return PlanarTransform(
                x=float(translation.x),
                y=float(translation.y),
                yaw=yaw_from_quaternion(
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self._last_tf_warning_monotonic >= self.warn_interval:
                self.get_logger().warning(
                    'Cannot transform %s -> %s (%s); using configured static fallback '
                    'x=%.4f y=%.4f yaw=%.4f'
                    % (
                        source_frame or '<empty>',
                        self.base_frame_id,
                        exc,
                        self.fallback_transform.x,
                        self.fallback_transform.y,
                        self.fallback_transform.yaw,
                    )
                )
                self._last_tf_warning_monotonic = now
            return self.fallback_transform

    def _on_scan(self, scan: LaserScan) -> None:
        transform = self._lookup_scan_to_base(scan)
        filtered_scan = LaserScan()
        filtered_scan.header = scan.header
        filtered_scan.angle_min = scan.angle_min
        filtered_scan.angle_max = scan.angle_max
        filtered_scan.angle_increment = scan.angle_increment
        filtered_scan.time_increment = scan.time_increment
        filtered_scan.scan_time = scan.scan_time
        filtered_scan.range_min = scan.range_min
        filtered_scan.range_max = scan.range_max
        filtered_scan.ranges, removed_indices = filter_ranges_inside_rectangle(
            scan.ranges,
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            transform=transform,
            min_x=self.min_x,
            max_x=self.max_x,
            min_y=self.min_y,
            max_y=self.max_y,
            padding=self.padding,
        )
        filtered_scan.intensities = list(scan.intensities)
        for index in removed_indices:
            if index < len(filtered_scan.intensities):
                filtered_scan.intensities[index] = 0.0

        self.publisher.publish(filtered_scan)
        self._scan_count += 1
        self._filtered_count += len(removed_indices)
        if self._scan_count % 200 == 0:
            self.get_logger().info(
                'Processed %d scans; removed %d chassis returns in total'
                % (self._scan_count, self._filtered_count)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFootprintFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
