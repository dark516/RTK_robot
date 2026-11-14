from __future__ import annotations

import glob
import math
import sys
import uuid
from time import monotonic, sleep
from typing import Iterable

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from serial import Serial, SerialException
from std_msgs.msg import Float32, Int32

from frob_interfaces.srv import Forward, Turn
from ros2_arduino_bridge.connection import ArduinoConnection


class Arduino_bridge(Node):
    """ROS-to-Arduino motor bridge with an independent command watchdog.

    The bridge intentionally owns a second watchdog downstream of the city
    ``motion_guard``.  If the guard (or any upstream command publisher) dies
    after issuing a non-zero command, this node sends one explicit zero-speed
    command after ``cmd_vel_timeout_s`` using ``time.monotonic()``.

    The timeout is independent of ROS time, so a paused/frozen simulation clock
    cannot keep physical motion alive.
    """

    MAX_LIN_SPEED = 0.4863  # [m/s]
    MAX_ANG_SPEED = 5.4  # [rad/s]

    DEFAULT_CMD_VEL_TIMEOUT_S = 0.30
    DEFAULT_WATCHDOG_PERIOD_S = 0.05
    SPEED_EPSILON = 1e-9

    def __init__(
        self,
        connection: ArduinoConnection,
        *,
        parameter_overrides: Iterable | None = None,
    ):
        unique_id = str(uuid.uuid4())[:8]
        super().__init__(
            f"arduino_bridge_{unique_id}",
            parameter_overrides=list(parameter_overrides or ()),
        )

        self._connect = connection
        self._connected = True

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_timeout_s", self.DEFAULT_CMD_VEL_TIMEOUT_S)
        self.declare_parameter(
            "cmd_vel_watchdog_period_s",
            self.DEFAULT_WATCHDOG_PERIOD_S,
        )
        self.declare_parameter("allow_legacy_motion_services", False)

        self.cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )
        self.cmd_vel_timeout_s = float(
            self.get_parameter("cmd_vel_timeout_s").value
        )
        self.cmd_vel_watchdog_period_s = float(
            self.get_parameter("cmd_vel_watchdog_period_s").value
        )
        self.allow_legacy_motion_services = bool(
            self.get_parameter("allow_legacy_motion_services").value
        )

        if not math.isfinite(self.cmd_vel_timeout_s) or self.cmd_vel_timeout_s <= 0.0:
            raise ValueError("cmd_vel_timeout_s must be finite and > 0")
        if self.cmd_vel_timeout_s > 1.0:
            raise ValueError(
                "cmd_vel_timeout_s must be <= 1.0 s for the motor safety watchdog"
            )
        if (
            not math.isfinite(self.cmd_vel_watchdog_period_s)
            or self.cmd_vel_watchdog_period_s <= 0.0
        ):
            raise ValueError("cmd_vel_watchdog_period_s must be finite and > 0")
        if self.cmd_vel_watchdog_period_s >= self.cmd_vel_timeout_s:
            raise ValueError(
                "cmd_vel_watchdog_period_s must be smaller than cmd_vel_timeout_s"
            )

        # Subscribers and publishers.
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.left_delta_pub = self.create_publisher(
            Int32,
            "left_motor/encoder/delta",
            10,
        )
        self.right_delta_pub = self.create_publisher(
            Int32,
            "right_motor/encoder/delta",
            10,
        )
        self.left_speed_pub = self.create_publisher(
            Float32,
            "left_motor/speed",
            10,
        )
        self.right_speed_pub = self.create_publisher(
            Float32,
            "right_motor/speed",
            10,
        )

        # Keep legacy service names for compatibility, but block direct motion
        # by default.  Competition/autonomy must go through motion_guard.
        self.rotate_srv = self.create_service(
            Turn,
            "rotate_robot",
            self.handle_rotate_request,
        )
        self.forward_srv = self.create_service(
            Forward,
            "forward_robot",
            self.handle_forward_request,
        )

        # Last command actually sent to the Arduino.
        self.last_linear: float | None = None
        self.last_angular: float | None = None

        # Independent downstream command watchdog state.
        self._last_cmd_vel_monotonic: float | None = None
        self._last_cmd_vel_was_nonzero = False
        self._timeout_stop_sent = False

        # Timers.
        self.data_request_timer = self.create_timer(0.1, self.data)
        self.connection_check_timer = self.create_timer(
            1.0,
            self.check_connection,
        )
        self.cmd_vel_watchdog_timer = self.create_timer(
            self.cmd_vel_watchdog_period_s,
            self._cmd_vel_watchdog,
        )

        self.get_logger().info(
            "Motor command watchdog enabled: "
            f"topic={self.cmd_vel_topic}, "
            f"timeout={self.cmd_vel_timeout_s:.3f}s, "
            f"period={self.cmd_vel_watchdog_period_s:.3f}s, "
            "legacy motion services="
            f"{'ENABLED' if self.allow_legacy_motion_services else 'DISABLED'}"
        )

    @classmethod
    def clamp_speed(cls, value, max_value):
        return max(min(value, max_value), -max_value)

    @classmethod
    def _is_nonzero_command(cls, linear: float, angular: float) -> bool:
        return (
            abs(linear) > cls.SPEED_EPSILON
            or abs(angular) > cls.SPEED_EPSILON
        )

    def _send_speeds(
        self,
        linear: float,
        angular: float,
        *,
        force: bool = False,
    ) -> None:
        """Send clamped speeds to the Arduino and update bridge state."""
        linear = float(self.clamp_speed(linear, self.MAX_LIN_SPEED))
        angular = float(self.clamp_speed(angular, self.MAX_ANG_SPEED))

        if force or linear != self.last_linear or angular != self.last_angular:
            self._connect.setSpeeds(linear, angular)
            self.last_linear = linear
            self.last_angular = angular
            self.get_logger().debug(
                f"New speeds: Lin={linear:.2f}m/s, Ang={angular:.2f}rad/s"
            )

    def _force_stop(self, reason: str, *, force: bool = True) -> bool:
        """Send an explicit zero command while the serial link is available."""
        if not self._connected:
            return False

        try:
            self._send_speeds(0.0, 0.0, force=force)
            self._last_cmd_vel_was_nonzero = False
            self._timeout_stop_sent = True
            self.get_logger().warning(f"Motor stop commanded: {reason}")
            return True
        except SerialException:
            self._handle_disconnection()
            return False

    def handle_rotate_request(self, request, response):
        if not self.allow_legacy_motion_services:
            self.get_logger().warning(
                "rotate_robot rejected: legacy direct-motion services are "
                "disabled while motion_guard owns motor commands"
            )
            response.success = False
            return response

        if not self._connected:
            self._print_error("Service call failed: No active connection!")
            response.success = False
            return response

        try:
            result = self._connect.turn_robot(request.angle, request.speed)
            response.success = bool(result)
            if result:
                self.get_logger().info(
                    f"Rotated by {request.angle}° at {request.speed}% power"
                )
        except SerialException as exc:
            self._handle_disconnection()
            response.success = False
            self._print_error(f"Rotation failed: {str(exc)}")
        return response

    def handle_forward_request(self, request, response):
        if not self.allow_legacy_motion_services:
            self.get_logger().warning(
                "forward_robot rejected: legacy direct-motion services are "
                "disabled while motion_guard owns motor commands"
            )
            response.success = False
            return response

        if not self._connected:
            self._print_error("Service call failed: No active connection!")
            response.success = False
            return response

        try:
            result = self._connect.go_dist(request.dist, request.speed)
            response.success = bool(result)
            if result:
                self.get_logger().info(
                    f"Moved {request.dist}cm at {request.speed}% power"
                )
        except SerialException as exc:
            self._handle_disconnection()
            response.success = False
            self._print_error(f"Movement failed: {str(exc)}")
        return response

    def cmd_vel_callback(self, msg: Twist):
        if not self._connected:
            return

        # Record receive time before serial I/O.  This is deliberately local
        # monotonic time, not msg time or ROS time.
        received_monotonic = monotonic()

        try:
            linear = float(
                self.clamp_speed(msg.linear.x, self.MAX_LIN_SPEED)
            )
            angular = float(
                self.clamp_speed(msg.angular.z, self.MAX_ANG_SPEED)
            )

            self._last_cmd_vel_monotonic = received_monotonic
            self._last_cmd_vel_was_nonzero = self._is_nonzero_command(
                linear,
                angular,
            )
            self._timeout_stop_sent = False

            self._send_speeds(linear, angular)

        except SerialException:
            self._handle_disconnection()

    def _cmd_vel_watchdog(self):
        """Fail closed when a previously nonzero command stream goes stale."""
        if not self._connected:
            return
        if self._last_cmd_vel_monotonic is None:
            return
        if not self._last_cmd_vel_was_nonzero:
            return
        if self._timeout_stop_sent:
            return

        age_s = monotonic() - self._last_cmd_vel_monotonic
        if age_s < self.cmd_vel_timeout_s:
            return

        self._force_stop(
            "cmd_vel watchdog expired "
            f"after {age_s:.3f}s "
            f"(limit {self.cmd_vel_timeout_s:.3f}s)"
        )

    def data(self):
        if not self._connected:
            return

        try:
            arduino_data = self._connect.get_data()
            self.left_delta_pub.publish(
                Int32(data=arduino_data.left_delta)
            )
            self.right_delta_pub.publish(
                Int32(data=arduino_data.right_delta)
            )
            self.left_speed_pub.publish(
                Float32(data=arduino_data.left_speed)
            )
            self.right_speed_pub.publish(
                Float32(data=arduino_data.right_speed)
            )
        except SerialException:
            self._handle_disconnection()

    def check_connection(self):
        if not self._connected:
            self._try_reconnect()

    def _handle_disconnection(self):
        if self._connected:
            self._print_error("Connection lost! Check Arduino connection.")
            self._connected = False
            self._last_cmd_vel_monotonic = None
            self._last_cmd_vel_was_nonzero = False
            self._timeout_stop_sent = False
            self.last_linear = None
            self.last_angular = None
            self._connect.close()

    def _try_reconnect(self):
        ports = self._find_arduino_ports()
        for port in ports:
            try:
                self.get_logger().info(
                    f"Attempting connection to {port}..."
                )
                connection = ArduinoConnection(Serial(port, 115200))
                sleep(2)

                if connection.is_arduino():
                    # A reconnect must never inherit an old nonzero actuator
                    # command.  Send zero before accepting fresh ROS commands.
                    connection.setSpeeds(0.0, 0.0)
                    self._connect = connection
                    self._connected = True
                    self.last_linear = 0.0
                    self.last_angular = 0.0
                    self._last_cmd_vel_monotonic = None
                    self._last_cmd_vel_was_nonzero = False
                    self._timeout_stop_sent = True
                    self.get_logger().info(
                        f"Successfully connected to Arduino at {port}; "
                        "explicit zero command sent"
                    )
                    return True

                connection.close()
                self.get_logger().warning(
                    f"Invalid response from {port}"
                )

            except (SerialException, OSError) as exc:
                self.get_logger().warning(
                    f"Connection failed to {port}: {str(exc)}"
                )
                continue
            except Exception as exc:
                self._print_error(
                    f"Unexpected error: {str(exc)}"
                )
                continue

        self._print_error("No valid Arduino devices found!")
        return False

    def _find_arduino_ports(self):
        if sys.platform.startswith("win"):
            return [f"COM{i + 1}" for i in range(256)]
        if sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
            return glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        if sys.platform.startswith("darwin"):
            return glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/tty.usbserial*")
        raise EnvironmentError("Unsupported OS")

    def _print_error(self, message):
        self.get_logger().error("\033[1;31m" + message + "\033[0m")

    def shutdown(self):
        if self._connected:
            try:
                # Stop before closing the serial transport.  Force the write
                # even if our cached state already says zero.
                self._force_stop("bridge shutdown", force=True)
            except Exception as exc:
                self._print_error(
                    f"Shutdown stop command failed: {str(exc)}"
                )

            try:
                self._connect.close()
            except Exception as exc:
                self._print_error(
                    f"Connection closure error: {str(exc)}"
                )

        self.destroy_node()


def find_arduino_ports():
    if sys.platform.startswith("win"):
        return [f"COM{i + 1}" for i in range(256)]
    if sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        return glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    if sys.platform.startswith("darwin"):
        return glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/tty.usbserial*")
    raise EnvironmentError("Unsupported OS")


def main(args=None):
    rclpy.init(args=args)
    arduino_bridge = None

    try:
        import argparse

        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--port",
            type=str,
            help="Serial port for Arduino",
            default=None,
        )
        parsed_args, _unknown = parser.parse_known_args()

        port = parsed_args.port
        ports = find_arduino_ports()
        if port:
            if port not in ports:
                print(f"\033[1;31mPort {port} not found!\033[0m")
                return
            ports = [port]
        else:
            print(
                "\033[36mNo port specified, searching for Arduino devices...\033[0m"
            )

        for candidate_port in ports:
            try:
                print(f"\033[36mConnecting to {candidate_port}...\033[0m")
                connection = ArduinoConnection(
                    Serial(candidate_port, 115200)
                )
                sleep(2)
                if connection.is_arduino():
                    # Initial startup is fail-closed as well.
                    connection.setSpeeds(0.0, 0.0)
                    arduino_bridge = Arduino_bridge(connection)
                    arduino_bridge.last_linear = 0.0
                    arduino_bridge.last_angular = 0.0
                    print(
                        "\033[1;32mSuccessfully connected to Arduino at "
                        f"{candidate_port}\033[0m"
                    )
                    break

                print(
                    "\033[33mDevice at "
                    f"{candidate_port} is not responding as Arduino\033[0m"
                )
                connection.close()
            except Exception as exc:
                print(
                    "\033[33mConnection to "
                    f"{candidate_port} failed: {str(exc)}\033[0m"
                )
                continue
        else:
            print("\033[1;31mNo valid Arduino devices found!\033[0m")
            return

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(arduino_bridge)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            arduino_bridge.shutdown()

    except Exception as exc:
        print(f"\033[1;31mCritical error: {str(exc)}\033[0m")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
