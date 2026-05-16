import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
import math


class QRNavigator(Node):
    """State-machine: drives forward using odometry + QR centering, reverses, stops."""

    def __init__(self):
        super().__init__('qr_navigator')

        self.declare_parameter('distance', 110.0)
        self.declare_parameter('base_speed', 0.07)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('wait_duration', 2.0)
        self.declare_parameter('kp_fwd', -1.0)
        self.declare_parameter('ki_fwd', 0.00)
        self.declare_parameter('kd_fwd', 0.0)
        self.declare_parameter('kp_bwd', -1.0)
        self.declare_parameter('ki_bwd', -0.0)
        self.declare_parameter('kd_bwd', -0.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('verbose', False)

        self._target_dist = float(self.get_parameter('distance').value) / 100.0
        self._base_speed = float(self.get_parameter('base_speed').value)
        self._max_angular = float(self.get_parameter('max_angular').value)
        self._wait_dur = float(self.get_parameter('wait_duration').value)
        freq = float(self.get_parameter('control_frequency').value)
        self._verbose = self.get_parameter('verbose').value

        self._kp_fwd = float(self.get_parameter('kp_fwd').value)
        self._ki_fwd = float(self.get_parameter('ki_fwd').value)
        self._kd_fwd = float(self.get_parameter('kd_fwd').value)
        self._kp_bwd = float(self.get_parameter('kp_bwd').value)
        self._ki_bwd = float(self.get_parameter('ki_bwd').value)
        self._kd_bwd = float(self.get_parameter('kd_bwd').value)

        self._state = 'FORWARD'
        self._state_since = self.get_clock().now()
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = self.get_clock().now()
        self._detected = False
        self._error = 0.0
        self._odom_pos = (0.0, 0.0)
        self._forward_start = None
        self._backward_start = None
        self._max_dist = 0.0
        self._dist = 0.0

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._dist_pub = self.create_publisher(Float32, '/dist', 10)

        self.create_subscription(Float32, '/qr/error', self._error_cb, 10)
        self.create_subscription(Bool, '/qr/detected', self._detected_cb, 10)
        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_cb, 10
        )

        self._timer = self.create_timer(1.0 / freq, self._tick)

        self.get_logger().info(
            f'QRNavigator started — go {self._target_dist*100:.0f}cm, '
            f'wait {self._wait_dur}s, base_speed={self._base_speed}'
        )

    def _log(self, msg):
        if self._verbose:
            self.get_logger().info(msg)

    @staticmethod
    def _pose(msg):
        p = msg.pose.pose.position
        return (p.x, p.y)

    @staticmethod
    def _dist_between(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _error_cb(self, msg):
        self._error = msg.data

    def _detected_cb(self, msg):
        self._detected = msg.data

    def _odom_cb(self, msg):
        self._odom_pos = self._pose(msg)

    def _set_state(self, new_state):
        self._state = new_state
        self._state_since = self.get_clock().now()
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = self.get_clock().now()
        self.get_logger().info(f'State -> {new_state}')

    def _elapsed(self):
        return (self.get_clock().now() - self._state_since).nanoseconds / 1e9

    def _compute_angular(self, error, dt, kp, ki, kd):
        self._integral += error * dt
        self._integral = max(min(self._integral, 1.0), -1.0)
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        output = kp * error + ki * self._integral + kd * derivative
        return max(min(output, self._max_angular), -self._max_angular)

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0 or dt > 0.5:
            dt = 0.05

        state = self._state
        cmd = Twist()

        if state == 'FORWARD':
            if self._forward_start is None:
                self._forward_start = self._odom_pos
                self._max_dist = 0.0

            self._dist = self._dist_between(self._odom_pos, self._forward_start)

            if self._dist >= self._target_dist:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self._cmd_pub.publish(cmd)
                self._dist_pub.publish(Float32(data=self._dist * 100.0))
                self._max_dist = self._dist
                self._set_state('WAIT_FWD')
                self.get_logger().info(f'Forward done - {self._dist*100:.1f}cm')
                return

            if not self._detected:
                cmd.linear.x = self._base_speed
                cmd.angular.z = 0.0
            else:
                angular = self._compute_angular(
                    self._error, dt, self._kp_fwd, self._ki_fwd, self._kd_fwd
                )
                cmd.linear.x = self._base_speed
                cmd.angular.z = angular
                self._log(f'FWD | dist={self._dist*100:.1f}cm error={self._error:+.3f} ang={angular:+.3f}')

        elif state == 'WAIT_FWD':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self._elapsed() >= self._wait_dur:
                self._forward_start = None
                self._backward_start = self._odom_pos
                self._set_state('BACKWARD')

        elif state == 'BACKWARD':
            if self._backward_start is None:
                self._backward_start = self._odom_pos
            back_dist = self._dist_between(self._odom_pos, self._backward_start)
            self._dist = max(0.0, self._max_dist - back_dist)

            if back_dist >= self._target_dist:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self._cmd_pub.publish(cmd)
                self._dist_pub.publish(Float32(data=self._dist * 100.0))
                self._set_state('WAIT_BWD')
                self.get_logger().info(f'Backward done - {back_dist*100:.1f}cm')
                return

            if not self._detected:
                cmd.linear.x = -self._base_speed
                cmd.angular.z = 0.0
            else:
                angular = self._compute_angular(
                    self._error, dt, self._kp_bwd, self._ki_bwd, self._kd_bwd
                )
                cmd.linear.x = -self._base_speed
                cmd.angular.z = angular
                self._log(f'BWD | dist={self._dist*100:.1f}cm error={self._error:+.3f} ang={angular:+.3f}')

        elif state == 'WAIT_BWD':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self._elapsed() >= self._wait_dur:
                self._set_state('STOP')
                self.get_logger().info('Mission complete')

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self._cmd_pub.publish(cmd)
        self._dist_pub.publish(Float32(data=self._dist * 100.0))


def main(args=None):
    rclpy.init(args=args)
    node = QRNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
