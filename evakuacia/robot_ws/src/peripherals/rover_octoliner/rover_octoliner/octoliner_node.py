from __future__ import annotations

import math
from typing import Iterable

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter as RosParameter
from rover_octoliner.octoliner_driver import OctolinerDriver
from rover_interfaces.msg import OctolinerReading
from rover_interfaces.srv import SetSensitivity
from std_msgs.msg import Bool, Float32, Float32MultiArray, UInt8
from std_srvs.srv import Trigger


def _coerce_pattern_mask(raw_pattern: object) -> int:
    if isinstance(raw_pattern, (list, tuple)):
        mask = 0
        for value in raw_pattern[:8]:
            mask = (mask << 1) | int(bool(value))
        return mask
    if isinstance(raw_pattern, int):
        return raw_pattern & 0xFF
    raise TypeError(f'Unsupported pattern type: {type(raw_pattern)!r}')


class OctolinerNode(Node):
    def __init__(self) -> None:
        super().__init__('octoliner_node')

        self.declare_parameter('i2c_address', 42)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'octoliner_link')
        self.declare_parameter('reading_topic', '/octoliner/reading')
        self.declare_parameter('analog_topic', '/octoliner/analog')
        self.declare_parameter('pattern_topic', '/octoliner/pattern')
        self.declare_parameter('line_position_topic', '/octoliner/line_position')
        self.declare_parameter(
            'tracked_line_topic', '/octoliner/tracked_line_position'
        )
        self.declare_parameter('line_visible_topic', '/octoliner/line_visible')
        self.declare_parameter('sensitivity_topic', '/octoliner/sensitivity')
        self.declare_parameter(
            'set_sensitivity_service', '/octoliner/set_sensitivity'
        )
        self.declare_parameter(
            'optimize_on_black_service', '/octoliner/optimize_on_black'
        )
        self.declare_parameter('sensitivity', 0.8)
        self.declare_parameter('auto_optimize_on_start', False)

        self._driver = self._create_driver()
        self._sensitivity = float(self.get_parameter('sensitivity').value)
        self._last_reading: OctolinerReading | None = None

        self._reading_publisher = self.create_publisher(
            OctolinerReading, self.get_parameter('reading_topic').value, 10
        )
        self._analog_publisher = self.create_publisher(
            Float32MultiArray, self.get_parameter('analog_topic').value, 10
        )
        self._pattern_publisher = self.create_publisher(
            UInt8, self.get_parameter('pattern_topic').value, 10
        )
        self._line_position_publisher = self.create_publisher(
            Float32, self.get_parameter('line_position_topic').value, 10
        )
        self._tracked_line_publisher = self.create_publisher(
            Float32, self.get_parameter('tracked_line_topic').value, 10
        )
        self._line_visible_publisher = self.create_publisher(
            Bool, self.get_parameter('line_visible_topic').value, 10
        )
        self._sensitivity_publisher = self.create_publisher(
            Float32, self.get_parameter('sensitivity_topic').value, 10
        )

        self.create_service(
            SetSensitivity,
            self.get_parameter('set_sensitivity_service').value,
            self._handle_set_sensitivity,
        )
        self.create_service(
            Trigger,
            self.get_parameter('optimize_on_black_service').value,
            self._handle_optimize_on_black,
        )

        self.add_on_set_parameters_callback(self._on_parameter_set)

        self._apply_sensitivity(self._sensitivity)
        if bool(self.get_parameter('auto_optimize_on_start').value):
            self._optimize_on_black()

        poll_rate_hz = max(1.0, float(self.get_parameter('poll_rate_hz').value))
        self._timer = self.create_timer(1.0 / poll_rate_hz, self._publish_reading)
        self.get_logger().info(
            'Octoliner ready on I2C address %d at %.1f Hz'
            % (int(self.get_parameter('i2c_address').value), poll_rate_hz)
        )

    def _create_driver(self):
        i2c_address = int(self.get_parameter('i2c_address').value)
        i2c_bus = int(self.get_parameter('i2c_bus').value)
        return OctolinerDriver(i2c_address=i2c_address, i2c_bus=i2c_bus)

    def _on_parameter_set(self, parameters):
        for parameter in parameters:
            if parameter.name == 'sensitivity':
                try:
                    self._apply_sensitivity(float(parameter.value))
                except Exception as exc:
                    return SetParametersResult(
                        successful=False,
                        reason=f'Failed to apply sensitivity: {exc}',
                    )
            elif parameter.name == 'poll_rate_hz':
                poll_rate_hz = max(1.0, float(parameter.value))
                self._timer.cancel()
                self._timer = self.create_timer(
                    1.0 / poll_rate_hz, self._publish_reading
                )
        return SetParametersResult(successful=True)

    def _apply_sensitivity(self, sensitivity: float) -> float:
        applied = max(0.0, min(1.0, float(sensitivity)))
        self._driver.set_sensitivity(applied)
        self._sensitivity = applied
        return applied

    def _optimize_on_black(self) -> float:
        success = bool(self._driver.optimize_sensitivity_on_black())
        if not success:
            raise RuntimeError(
                'Calibration failed. Place all 8 sensors fully over the black line '
                'or a black surface and try again.'
            )
        self._sensitivity = float(self._driver.get_sensitivity())
        return self._sensitivity

    def _build_reading(self) -> OctolinerReading:
        analog_values = [float(value) for value in self._driver.analog_read_all()]
        analog_values = (analog_values + [0.0] * 8)[:8]
        pattern = _coerce_pattern_mask(
            self._driver.map_analog_to_pattern(analog_values)
        )
        dark_sensor_count = int(pattern.bit_count())
        line_visible = pattern != 0
        line_position = float(self._driver.map_pattern_to_line(pattern))
        tracked_line_position = float(self._driver.track_line(pattern))

        msg = OctolinerReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.analog_values = analog_values
        msg.pattern = pattern
        msg.dark_sensor_count = dark_sensor_count
        msg.line_visible = line_visible
        msg.line_position = line_position
        msg.tracked_line_position = tracked_line_position
        msg.sensitivity = float(self._sensitivity)
        return msg

    def _publish_reading(self) -> None:
        try:
            reading = self._build_reading()
        except Exception as exc:
            self.get_logger().warning(
                f'Octoliner read failed: {exc}', throttle_duration_sec=2.0
            )
            return

        self._last_reading = reading
        self._reading_publisher.publish(reading)

        analog = Float32MultiArray()
        analog.data = list(reading.analog_values)
        self._analog_publisher.publish(analog)

        pattern = UInt8()
        pattern.data = int(reading.pattern)
        self._pattern_publisher.publish(pattern)

        line_position = Float32()
        line_position.data = float(reading.line_position)
        self._line_position_publisher.publish(line_position)

        tracked = Float32()
        tracked.data = float(reading.tracked_line_position)
        self._tracked_line_publisher.publish(tracked)

        visible = Bool()
        visible.data = bool(reading.line_visible)
        self._line_visible_publisher.publish(visible)

        sensitivity = Float32()
        sensitivity.data = float(reading.sensitivity)
        self._sensitivity_publisher.publish(sensitivity)

    def _handle_set_sensitivity(self, request, response):
        try:
            result = self.set_parameters([
                RosParameter('sensitivity', value=float(request.sensitivity))
            ])
            if not result or not result[0].successful:
                raise RuntimeError(
                    result[0].reason if result else 'parameter update rejected'
                )
            applied = float(self._sensitivity)
        except Exception as exc:
            response.success = False
            response.message = f'Failed to set sensitivity: {exc}'
            response.applied_sensitivity = float(self._sensitivity)
            return response

        response.success = True
        response.message = 'Sensitivity updated'
        response.applied_sensitivity = float(applied)
        return response

    def _handle_optimize_on_black(self, request, response):
        del request
        try:
            optimized = self._optimize_on_black()
            result = self.set_parameters([
                RosParameter('sensitivity', value=float(optimized))
            ])
            if not result or not result[0].successful:
                raise RuntimeError(
                    result[0].reason if result else 'parameter update rejected'
                )
        except Exception as exc:
            response.success = False
            response.message = f'Failed to optimize sensitivity: {exc}'
            return response

        response.success = True
        response.message = (
            f'Sensitivity optimized on black surface: {optimized:.1f}'
        )
        return response


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node: OctolinerNode | None = None
    try:
        node = OctolinerNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
