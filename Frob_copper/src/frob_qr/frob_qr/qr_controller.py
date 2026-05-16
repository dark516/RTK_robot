import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class QRController(Node):
    """PID controller: centers robot on QR code while moving forward at base speed."""

    def __init__(self):
        super().__init__('qr_controller')

        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('base_speed', 0.3)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('verbose', False)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.max_angular = self.get_parameter('max_angular').value
        self._verbose = self.get_parameter('verbose').value

        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = self.get_clock().now()
        self._detected = False
        self._error = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32, '/qr/error', self._error_callback, 10)
        self.create_subscription(Bool, '/qr/detected', self._detected_callback, 10)

        freq = self.get_parameter('control_frequency').value
        self.timer = self.create_timer(1.0 / freq, self._control_loop)

        self.get_logger().info(
            f'QRController started — base_speed={self.base_speed}, '
            f'Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, '
            f'max_angular={self.max_angular}'
        )

    def _log(self, msg):
        if self._verbose:
            self.get_logger().info(msg)

    def _error_callback(self, msg):
        self._error = msg.data

    def _detected_callback(self, msg):
        was = self._detected
        self._detected = msg.data
        if self._detected and not was:
            self.get_logger().info('QR acquired — starting PID control')

    def _control_loop(self):
        cmd = Twist()

        if not self._detected:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._integral = 0.0
            self._prev_error = 0.0
            self.cmd_pub.publish(cmd)
            return

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        if dt <= 0.0 or dt > 0.5:
            cmd.linear.x = self.base_speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        error = self._error

        self._integral += error * dt
        self._integral = max(min(self._integral, 1.0), -1.0)
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        angular = max(min(output, self.max_angular), -self.max_angular)

        cmd.linear.x = self.base_speed
        cmd.angular.z = angular

        self._log(
            f'PID | error={error:+.3f} | P={self.kp * error:+.3f} '
            f'I={self.ki * self._integral:+.3f} D={self.kd * derivative:+.3f} '
            f'| angular={angular:+.3f}'
        )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = QRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
