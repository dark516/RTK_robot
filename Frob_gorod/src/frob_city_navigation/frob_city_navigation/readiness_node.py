"""ROS 2 readiness/diagnostics aggregator for Frob City (WP13).

This node does not command motion.  It evaluates whether the headless autonomy
stack is safe to START and publishes a single fail-closed readiness boolean plus
structured diagnostics and reproducibility fingerprints.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from frob_city_interfaces.msg import GuardState, MapState, MissionState, SignDetectionArray

from .readiness import ReadinessConfig, ReadinessSample, evaluate_readiness


class ReadinessNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__('city_readiness', parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._started_mono = time.monotonic()
        self._odom_rx: float | None = None
        self._det_rx: float | None = None
        self._map_rx: float | None = None
        self._guard_rx: float | None = None
        self._latest_odom: Odometry | None = None
        self._latest_map: MapState | None = None
        self._latest_guard: GuardState | None = None
        self._latest_detection_healthy = False
        self._latest_mission: MissionState | None = None
        self._last_ready: bool | None = None
        self._last_reason = 'startup'
        self._replay_real_driver_publishers: tuple[str, ...] = ()

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(MapState, self._p('map_topic'), self._on_map, map_qos)
        self.create_subscription(GuardState, self._p('guard_state_topic'), self._on_guard, map_qos)
        self.create_subscription(Odometry, self._p('odom_topic'), self._on_odom, 20)
        self.create_subscription(
            SignDetectionArray, self._p('detections_topic'), self._on_detections, 10
        )
        self.create_subscription(MissionState, self._p('mission_state_topic'), self._on_mission, 10)

        self._ready_pub = self.create_publisher(Bool, self._p('system_ready_topic'), map_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, self._p('diagnostics_topic'), 10)
        self._fingerprint_pub = self.create_publisher(
            String, self._p('fingerprint_topic'), map_qos
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._nav2 = ActionClient(self, FollowPath, self._p('follow_path_action'))

        self._config_fingerprint, self._configuration_ok = self._fingerprint_files(
            list(self.get_parameter('config_files').value)
        )
        self._model_fingerprint, self._models_ok = self._fingerprint_files(
            list(self.get_parameter('model_files').value), allow_empty=True
        )
        self._publish_fingerprint()
        hz = float(self.get_parameter('check_hz').value)
        if not math.isfinite(hz) or hz <= 0.0:
            raise ValueError('check_hz must be finite and > 0')
        self.create_timer(1.0 / hz, self._evaluate)
        self.get_logger().info('WP13 readiness aggregator started fail-closed.')

    def _declare_parameters(self) -> None:
        defaults = {
            'system_ready_topic': '/city/system_ready',
            'diagnostics_topic': '/city/diagnostics',
            'fingerprint_topic': '/city/config_fingerprint',
            'map_topic': '/city/map_state',
            'guard_state_topic': '/city/guard_state',
            'mission_state_topic': '/city/mission_state',
            'odom_topic': '/odom',
            'detections_topic': '/traffic_sign/detections',
            'raw_cmd_topic': '/city/cmd_vel_raw',
            'driver_cmd_topic': '/cmd_vel',
            'real_driver_topic': '/cmd_vel',
            'follow_path_action': '/follow_path',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'expected_guard_node_name': 'motion_guard',
            'check_hz': 5.0,
            'startup_grace_s': 0.50,
            'odom_max_age_s': 0.40,
            'perception_max_age_s': 1.00,
            'map_max_age_s': 1.25,
            'guard_max_age_s': 0.50,
            'require_perception': True,
            'require_tf': True,
            'require_nav2': True,
            'require_command_chain': True,
            'require_start_pose': False,
            'require_driver_subscriber': True,
            'replay_mode': False,
            'start_x_m': 0.0,
            'start_y_m': 0.0,
            'start_yaw_rad': 0.0,
            'start_xy_tolerance_m': 0.10,
            'start_yaw_tolerance_rad': 0.20,
            'config_files': [''],
            'model_files': [''],
            'log_path': '/tmp/frob_city_readiness.jsonl',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _p(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg
        self._odom_rx = time.monotonic()

    def _on_detections(self, msg: SignDetectionArray) -> None:
        self._det_rx = time.monotonic()
        self._latest_detection_healthy = bool(msg.frame_valid)

    def _on_map(self, msg: MapState) -> None:
        self._latest_map = msg
        self._map_rx = time.monotonic()

    def _on_guard(self, msg: GuardState) -> None:
        self._latest_guard = msg
        self._guard_rx = time.monotonic()

    def _on_mission(self, msg: MissionState) -> None:
        self._latest_mission = msg

    @staticmethod
    def _age(now: float, stamp: float | None) -> float | None:
        return None if stamp is None else max(0.0, now - stamp)

    def _fingerprint_files(self, values: list[str], *, allow_empty: bool = False) -> tuple[str, bool]:
        paths = [Path(str(v)).expanduser() for v in values if str(v).strip()]
        if not paths:
            return ('none', bool(allow_empty))
        h = hashlib.sha256()
        for path in sorted(paths, key=lambda p: str(p)):
            h.update(str(path).encode())
            if not path.is_file():
                h.update(b'<missing>')
                return h.hexdigest(), False
            h.update(path.read_bytes())
        return h.hexdigest(), True

    def _topic_types_ok(self) -> bool:
        expected = {
            self._p('map_topic'): 'frob_city_interfaces/msg/MapState',
            self._p('guard_state_topic'): 'frob_city_interfaces/msg/GuardState',
            self._p('odom_topic'): 'nav_msgs/msg/Odometry',
            self._p('detections_topic'): 'frob_city_interfaces/msg/SignDetectionArray',
            self._p('raw_cmd_topic'): 'geometry_msgs/msg/Twist',
            self._p('driver_cmd_topic'): 'geometry_msgs/msg/Twist',
        }
        available = dict(self.get_topic_names_and_types())
        for topic, wanted in expected.items():
            types = available.get(topic, [])
            if wanted not in types:
                return False
        return True

    def _tf_ok(self) -> bool:
        try:
            self._tf_buffer.lookup_transform(self._p('map_frame'), self._p('base_frame'), Time())
            return True
        except Exception:
            return False

    def _command_chain_ok(self) -> bool:
        driver_topic = self._p('driver_cmd_topic')
        raw_topic = self._p('raw_cmd_topic')
        pubs = self.get_publishers_info_by_topic(driver_topic)
        raw_pubs = self.get_publishers_info_by_topic(raw_topic)
        subs = self.get_subscriptions_info_by_topic(driver_topic)
        expected = self._p('expected_guard_node_name').lstrip('/')
        if len(pubs) != 1:
            return False
        publisher_name = str(pubs[0].node_name).lstrip('/')
        if expected and publisher_name != expected:
            return False
        if not raw_pubs:
            return False
        forbidden = ('teleop', 'keyboard', 'joy')
        if any(any(x in str(info.node_name).lower() for x in forbidden) for info in pubs):
            return False
        if bool(self.get_parameter('require_driver_subscriber').value) and not subs:
            return False
        return True

    def _start_pose_ok(self) -> bool:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._p('map_frame'), self._p('base_frame'), Time()
            )
        except Exception:
            return False
        dx = float(transform.transform.translation.x) - float(self.get_parameter('start_x_m').value)
        dy = float(transform.transform.translation.y) - float(self.get_parameter('start_y_m').value)
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        dyaw = math.atan2(
            math.sin(yaw - float(self.get_parameter('start_yaw_rad').value)),
            math.cos(yaw - float(self.get_parameter('start_yaw_rad').value)),
        )
        return (
            math.hypot(dx, dy) <= float(self.get_parameter('start_xy_tolerance_m').value)
            and abs(dyaw) <= float(self.get_parameter('start_yaw_tolerance_rad').value)
        )

    @staticmethod
    def _endpoint_name(info) -> str:
        namespace = str(getattr(info, 'node_namespace', '') or '').rstrip('/')
        name = str(getattr(info, 'node_name', '') or '').lstrip('/')
        return f"{namespace}/{name}" if namespace else f"/{name}"

    def _replay_isolation_ok(self) -> bool:
        if not bool(self.get_parameter('replay_mode').value):
            self._replay_real_driver_publishers = ()
            return True
        driver_topic = self._p('driver_cmd_topic')
        real_topic = self._p('real_driver_topic')
        pubs = self.get_publishers_info_by_topic(real_topic)
        self._replay_real_driver_publishers = tuple(
            sorted(self._endpoint_name(info) for info in pubs)
        )
        return driver_topic != real_topic and not self._replay_real_driver_publishers

    def _sample(self) -> ReadinessSample:
        now = time.monotonic()
        map_msg = self._latest_map
        guard_msg = self._latest_guard
        replay_mode = bool(self.get_parameter('replay_mode').value)
        return ReadinessSample(
            configuration_ok=self._configuration_ok and self._models_ok,
            topic_types_ok=self._topic_types_ok(),
            tf_ok=self._tf_ok(),
            odom_age_s=self._age(now, self._odom_rx),
            perception_age_s=self._age(now, self._det_rx),
            map_age_s=self._age(now, self._map_rx),
            map_ready=bool(map_msg.ready) if map_msg is not None else False,
            localization_healthy=bool(map_msg.localization_healthy) if map_msg is not None else False,
            perception_healthy=(
                bool(map_msg.perception_healthy) and self._latest_detection_healthy
                if map_msg is not None
                else False
            ),
            guard_age_s=self._age(now, self._guard_rx),
            guard_fault_latched=bool(guard_msg.fault_latched) if guard_msg is not None else False,
            guard_stopped=bool(guard_msg.stopped) if guard_msg is not None else False,
            nav2_ready=self._nav2.server_is_ready(),
            command_chain_ok=self._command_chain_ok(),
            start_pose_ok=self._start_pose_ok(),
            replay_isolated=self._replay_isolation_ok() if replay_mode else True,
        )

    def _config(self) -> ReadinessConfig:
        return ReadinessConfig(
            odom_max_age_s=float(self.get_parameter('odom_max_age_s').value),
            perception_max_age_s=float(self.get_parameter('perception_max_age_s').value),
            map_max_age_s=float(self.get_parameter('map_max_age_s').value),
            guard_max_age_s=float(self.get_parameter('guard_max_age_s').value),
            require_perception=bool(self.get_parameter('require_perception').value),
            require_tf=bool(self.get_parameter('require_tf').value),
            require_nav2=bool(self.get_parameter('require_nav2').value),
            require_command_chain=bool(self.get_parameter('require_command_chain').value),
            require_start_pose=bool(self.get_parameter('require_start_pose').value),
            require_driver_subscriber=bool(self.get_parameter('require_driver_subscriber').value),
            replay_mode=bool(self.get_parameter('replay_mode').value),
        )

    def _evaluate(self) -> None:
        decision = evaluate_readiness(self._config(), self._sample())
        ready = bool(decision.ready)
        # Keep fail-closed during the short graph-discovery grace period too.
        if time.monotonic() - self._started_mono < float(self.get_parameter('startup_grace_s').value):
            ready = False
        self._ready_pub.publish(Bool(data=ready))

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, check in decision.checks.items():
            status = DiagnosticStatus()
            status.name = f'frob_city/readiness/{name}'
            status.hardware_id = 'frob-city'
            status.level = DiagnosticStatus.OK if check.ok else DiagnosticStatus.ERROR
            status.message = check.reason
            msg.status.append(status)
        overall = DiagnosticStatus()
        overall.name = 'frob_city/readiness/system'
        overall.hardware_id = 'frob-city'
        overall.level = DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
        failed = [name for name, c in decision.checks.items() if not c.ok]
        overall.message = 'ready' if ready else 'not_ready:' + ','.join(failed)
        overall.values = [
            KeyValue(key='config_sha256', value=self._config_fingerprint),
            KeyValue(key='model_sha256', value=self._model_fingerprint),
            KeyValue(key='run_id', value=str(self._latest_map.run_id) if self._latest_map else ''),
            KeyValue(key='map_instance_id', value=str(self._latest_map.map_instance_id) if self._latest_map else ''),
            KeyValue(
                key='navigator_instance_id',
                value=str(self._latest_guard.navigator_instance_id) if self._latest_guard else '',
            ),
            KeyValue(
                key='real_driver_publishers',
                value=','.join(self._replay_real_driver_publishers),
            ),
        ]
        msg.status.append(overall)
        self._diag_pub.publish(msg)
        reason = overall.message
        if ready != self._last_ready or reason != self._last_reason:
            self._write_log(ready, reason, decision)
            self._last_ready = ready
            self._last_reason = reason

    def _publish_fingerprint(self) -> None:
        payload = {
            'config_sha256': self._config_fingerprint,
            'model_sha256': self._model_fingerprint,
        }
        self._fingerprint_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _write_log(self, ready: bool, reason: str, decision) -> None:
        path = Path(self._p('log_path')).expanduser()
        if not str(path):
            return
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            record = {
                'time_ros_s': self.get_clock().now().nanoseconds * 1e-9,
                'time_monotonic_s': time.monotonic(),
                'ready': ready,
                'reason': reason,
                'checks': {k: {'ok': v.ok, 'reason': v.reason} for k, v in decision.checks.items()},
                'config_sha256': self._config_fingerprint,
                'model_sha256': self._model_fingerprint,
                'run_id': str(self._latest_map.run_id) if self._latest_map else '',
                'map_instance_id': str(self._latest_map.map_instance_id) if self._latest_map else '',
                'navigator_instance_id': (
                    str(self._latest_guard.navigator_instance_id) if self._latest_guard else ''
                ),
            }
            with path.open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(record, sort_keys=True) + '\n')
        except Exception as exc:
            self.get_logger().error(f'readiness log write failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReadinessNode()
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
