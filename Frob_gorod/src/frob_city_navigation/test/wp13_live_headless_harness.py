#!/usr/bin/env python3
"""Live WP13 headless/replay gate.

Runs production city_replay.launch.py with real motion_guard + readiness, while
all neighboring systems are fake and the final motor topic is physically
unreachable.  The fake environment toggles Nav2/perception health to prove
readiness is fail-closed.
"""
from __future__ import annotations

import math
import os
import signal
from pathlib import Path
import subprocess
import tempfile
import sys
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from frob_city_interfaces.msg import MapState, SignDetectionArray


class FakeEnvironment(Node):
    def __init__(self) -> None:
        super().__init__('wp13_fake_environment')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(MapState, '/city/map_state', transient)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.det_pub = self.create_publisher(SignDetectionArray, '/traffic_sign/detections', 10)
        self.raw_pub = self.create_publisher(Twist, '/city/cmd_vel_raw', 10)
        self.create_subscription(Twist, '/city/replay/cmd_vel_sink', self._sink, 20)
        self.create_subscription(Bool, '/city/system_ready', self._ready, transient)
        self.create_subscription(DiagnosticArray, '/city/diagnostics', self._diag, 10)
        self.tf = TransformBroadcaster(self)
        self.ready_history: list[bool] = []
        self.last_diag = ''
        self.nonzero_sink_count = 0
        self.sink_count = 0
        self.publish_perception = True
        self.nav2_server = self._make_nav2_server()
        self.create_timer(0.05, self._tick)

    def _make_nav2_server(self):
        return ActionServer(self, FollowPath, '/follow_path', execute_callback=self._execute)

    async def _execute(self, goal_handle):
        goal_handle.succeed()
        return FollowPath.Result()

    def drop_nav2(self):
        if self.nav2_server is not None:
            self.nav2_server.destroy()
            self.nav2_server = None

    def restore_nav2(self):
        if self.nav2_server is None:
            self.nav2_server = self._make_nav2_server()

    def _ready(self, msg: Bool):
        self.ready_history.append(bool(msg.data))

    def _diag(self, msg: DiagnosticArray):
        for status in msg.status:
            if status.name.endswith('/system'):
                self.last_diag = status.message

    def _sink(self, msg: Twist):
        self.sink_count += 1
        if (
            abs(float(msg.linear.x)) > 1e-9
            or abs(float(msg.linear.y)) > 1e-9
            or abs(float(msg.angular.z)) > 1e-9
        ):
            self.nonzero_sink_count += 1

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        m = MapState()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.run_id = 'WP13_RUN'
        m.map_instance_id = 'WP13_MAP'
        m.field_id = 'WP13_TEST_FIELD'
        m.revision = 1
        m.ready = True
        m.perception_healthy = True
        m.localization_healthy = True
        m.reason = 'wp13_fake_healthy'
        self.map_pub.publish(m)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        if self.publish_perception:
            d = SignDetectionArray()
            d.header.stamp = stamp
            d.header.frame_id = 'camera_optical_frame'
            d.detector_instance_id = 'WP13_FAKE_DETECTOR'
            d.frame_sequence = int(time.monotonic() * 20) & 0xFFFFFFFF
            d.image_width_px = 640
            d.image_height_px = 480
            d.calibration_id = 'WP13_TEST'
            d.frame_valid = True
            d.invalid_reason = ''
            self.det_pub.publish(d)

        self.raw_pub.publish(Twist())
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)


def wait_until(predicate, timeout: float, label: str) -> None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(f'timeout waiting for {label}')


def main() -> int:
    # Run the complete WP13 live gate in a dedicated DDS domain so an
    # unrelated robot/teleop/Nav2 process in the operator's normal domain
    # cannot participate in the test graph or reach hardware.  The child
    # `ros2 launch` process inherits this value.
    test_domain = os.environ.get('WP13_ROS_DOMAIN_ID', '137')
    os.environ['ROS_DOMAIN_ID'] = test_domain
    print(f'WP13 live isolated ROS_DOMAIN_ID={test_domain}')

    root = Path(__file__).resolve().parents[1]
    guard_cfg = root / 'config/wp13_guard_test.yaml'
    readiness_cfg = root / 'config/readiness.yaml'
    field_cfg = root / 'config/wp13_test_field.yaml'
    event_cfg = root / 'config/wp13_test_event.yaml'
    perception_cfg = root / 'config/wp13_test_perception.yaml'
    navigation_cfg = root / 'config/wp13_test_navigation.yaml'
    navigator_cfg = root / 'config/navigator.yaml'

    rclpy.init()
    env = FakeEnvironment()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(env)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # Dedicated domain must start with no real-driver publisher.  This is
    # stronger than filtering by node name: any publisher would invalidate
    # hardware isolation.
    initial_real_pubs = env.get_publishers_info_by_topic('/cmd_vel')
    if initial_real_pubs:
        names = [f'{i.node_namespace}/{i.node_name}' for i in initial_real_pubs]
        raise AssertionError(f'isolated domain unexpectedly has /cmd_vel publishers before launch: {names}')

    cmd = [
        'ros2', 'launch', 'frob_city_navigation', 'city_replay.launch.py',
        'include_map_planner:=false', 'include_navigator:=false',
        'include_motion_guard:=true', 'include_readiness:=true',
        f'motion_guard_config:={guard_cfg}', f'readiness_config:={readiness_cfg}',
        f'field_config:={field_cfg}', f'event_config:={event_cfg}',
        f'perception_config:={perception_cfg}', f'navigation_config:={navigation_cfg}',
        f'navigator_config:={navigator_cfg}',
    ]
    launch_log = tempfile.NamedTemporaryFile(prefix='wp13_launch_', suffix='.log', delete=False, mode='w+', encoding='utf-8')
    proc = subprocess.Popen(
        cmd,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        wait_until(lambda: bool(env.ready_history), 8.0, 'first readiness publication')
        wait_until(lambda: env.ready_history and env.ready_history[-1] is True, 10.0, 'healthy READY')
        print('L1 replay launch reaches READY with healthy fake neighbors: PASS')

        real_pubs = env.get_publishers_info_by_topic('/cmd_vel')
        real_pub_names = [f'{i.node_namespace}/{i.node_name}' for i in real_pubs]
        assert len(real_pubs) == 0, (
            f'replay unexpectedly has /cmd_vel publishers in isolated domain: {real_pub_names}'
        )
        wait_until(lambda: env.sink_count > 3, 4.0, 'replay sink zero commands')
        assert env.nonzero_sink_count == 0
        print('L2 replay has no /cmd_vel publisher and sink remains zero: PASS')

        env.drop_nav2()
        wait_until(lambda: env.ready_history and env.ready_history[-1] is False, 4.0, 'READY false after Nav2 loss')
        assert 'nav2' in env.last_diag or 'not_ready' in env.last_diag
        print('L3 Nav2 loss -> NOT READY while output remains zero: PASS')
        assert env.nonzero_sink_count == 0

        env.restore_nav2()
        wait_until(lambda: env.ready_history and env.ready_history[-1] is True, 5.0, 'READY after Nav2 restore')
        env.publish_perception = False
        wait_until(lambda: env.ready_history and env.ready_history[-1] is False, 4.0, 'READY false after perception stale')
        print('L4 stale perception -> NOT READY: PASS')

        env.publish_perception = True
        wait_until(lambda: env.ready_history and env.ready_history[-1] is True, 5.0, 'READY after perception restore')
        assert env.nonzero_sink_count == 0
        print('L5 recovery is explicit and replay command remains zero: PASS')
        print('=== WP13 LIVE HEADLESS READINESS SCENARIO: PASS ===')
        return 0
    except Exception as exc:
        print(f'WP13 live FAIL: {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    finally:
        # Terminate the complete ros2 launch process group, not only the
        # parent process.  launch children (motion_guard/readiness/etc.) must
        # never survive a failed or interrupted WP13 live gate.
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=2.0)
        launch_log.flush()
        launch_log.seek(0)
        output = launch_log.read()
        launch_log.close()
        if proc.returncode not in (0, -15, -2, None) and output:
            print('--- city_replay launch output ---', file=sys.stderr)
            print(output[-6000:], file=sys.stderr)
        executor.shutdown(timeout_sec=2.0)
        env.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == '__main__':
    raise SystemExit(main())
