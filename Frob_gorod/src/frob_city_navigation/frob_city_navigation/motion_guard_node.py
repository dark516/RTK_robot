"""Independent ROS 2 wrapper for :mod:`motion_guard_core`.

The node is intentionally fail closed.  It publishes zero on startup and on
all missing/stale/invalid inputs.  The existing robot uses ``Twist`` on the
motor-driver boundary, therefore this wrapper consumes ``/city/cmd_vel_raw``
and is the only autonomy publisher intended to drive ``/cmd_vel``.
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from frob_city_interfaces.msg import GuardState, MapState, MotionPermit

from .models import Point2D, Polygon2D, Pose2D, PoseSample
from .motion_guard_core import GuardConfig, GuardInputs, MotionGuardCore, OdomSample, RawCommand
from .ros_execution_conversions import (
    map_state_from_msg,
    motion_permit_from_msg,
    odometry_to_guard_samples,
)
from .ros_map_conversions import stamp_to_seconds, transform_to_pose2d


class MotionGuardNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("motion_guard", parameter_overrides=parameter_overrides)

        self.declare_parameter("commissioned", False)
        self.declare_parameter("field_id", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("raw_cmd_topic", "/city/cmd_vel_raw")
        self.declare_parameter("driver_cmd_topic", "/cmd_vel")
        self.declare_parameter("map_topic", "/city/map_state")
        self.declare_parameter("permit_topic", "/city/motion_permit")
        self.declare_parameter("guard_state_topic", "/city/guard_state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("emergency_stop_topic", "/city/emergency_stop")
        self.declare_parameter("enable_emergency_stop_topic", False)
        self.declare_parameter("allow_map_frame_odom_pose", False)
        self.declare_parameter("evaluation_hz", 50.0)

        # Physical parameters are intentionally invalid by default.  Setting
        # commissioned=true requires measured values to be supplied explicitly.
        self.declare_parameter("footprint_xy", [0.0])
        self.declare_parameter("braking_deceleration_mps2", -1.0)
        self.declare_parameter("braking_latency_s", -1.0)
        self.declare_parameter("hard_max_linear_speed_mps", -1.0)
        self.declare_parameter("hard_max_yaw_rate_radps", -1.0)
        self.declare_parameter("path_lateral_tolerance_m", -1.0)
        self.declare_parameter("permit_max_age_s", 0.20)
        self.declare_parameter("raw_command_max_age_s", 0.20)
        self.declare_parameter("map_max_age_s", 1.00)
        self.declare_parameter("pose_max_age_s", 0.20)
        self.declare_parameter("odom_max_age_s", 0.20)
        self.declare_parameter("max_permit_lifetime_s", 0.20)
        self.declare_parameter("stationary_linear_max_mps", 0.01)
        self.declare_parameter("stationary_angular_max_radps", 0.03)
        self.declare_parameter("footprint_edge_sample_m", 0.02)
        self.declare_parameter("rollout_step_m", 0.02)
        self.declare_parameter("geometry_epsilon_m", 1e-6)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._allow_map_frame_odom_pose = bool(
            self.get_parameter("allow_map_frame_odom_pose").value
        )
        self._core: MotionGuardCore | None = None
        self._commissioning_reason = "guard_not_commissioned"
        self._build_core_if_commissioned()

        self._permit = None
        self._map_snapshot = None
        self._map_stamp_s = 0.0
        self._map_receipt_monotonic_s = -math.inf
        self._pose: PoseSample | None = None
        self._pose_receipt_monotonic_s = -math.inf
        self._odom: OdomSample | None = None
        self._raw_command: RawCommand | None = None
        self._emergency_stop_active = False
        self._navigator_instance_id = ""
        self._boundary_reason = "startup_no_inputs"

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_volatile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._driver_pub = self.create_publisher(
            Twist, str(self.get_parameter("driver_cmd_topic").value), reliable_volatile
        )
        self._state_pub = self.create_publisher(
            GuardState, str(self.get_parameter("guard_state_topic").value), state_qos
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("raw_cmd_topic").value),
            self._on_raw_command,
            reliable_volatile,
        )
        self.create_subscription(
            MotionPermit,
            str(self.get_parameter("permit_topic").value),
            self._on_permit,
            reliable_volatile,
        )
        self.create_subscription(
            MapState,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            state_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            sensor_qos,
        )
        if bool(self.get_parameter("enable_emergency_stop_topic").value):
            self.create_subscription(
                Bool,
                str(self.get_parameter("emergency_stop_topic").value),
                self._on_emergency_stop,
                reliable_volatile,
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        hz = float(self.get_parameter("evaluation_hz").value)
        if not math.isfinite(hz) or hz <= 0.0:
            raise ValueError("evaluation_hz must be finite and > 0")
        self._timer = self.create_timer(1.0 / hz, self._evaluate)

        # A guard restart must never leave an old nonzero command downstream.
        self._publish_zero()
        self._publish_state(reason=self._commissioning_reason if self._core is None else "startup")

    def _build_core_if_commissioned(self) -> None:
        if not bool(self.get_parameter("commissioned").value):
            self._commissioning_reason = "guard_not_commissioned"
            return
        try:
            field_id = str(self.get_parameter("field_id").value)
            raw = [float(v) for v in self.get_parameter("footprint_xy").value]
            if len(raw) < 6 or len(raw) % 2:
                raise ValueError("footprint_xy must contain x,y pairs for >=3 vertices")
            footprint = Polygon2D(
                tuple(Point2D(raw[i], raw[i + 1]) for i in range(0, len(raw), 2))
            )
            cfg = GuardConfig(
                field_id=field_id,
                footprint=footprint,
                braking_deceleration_mps2=float(self.get_parameter("braking_deceleration_mps2").value),
                braking_latency_s=float(self.get_parameter("braking_latency_s").value),
                hard_max_linear_speed_mps=float(self.get_parameter("hard_max_linear_speed_mps").value),
                hard_max_yaw_rate_radps=float(self.get_parameter("hard_max_yaw_rate_radps").value),
                path_lateral_tolerance_m=float(self.get_parameter("path_lateral_tolerance_m").value),
                permit_max_age_s=float(self.get_parameter("permit_max_age_s").value),
                raw_command_max_age_s=float(self.get_parameter("raw_command_max_age_s").value),
                map_max_age_s=float(self.get_parameter("map_max_age_s").value),
                pose_max_age_s=float(self.get_parameter("pose_max_age_s").value),
                odom_max_age_s=float(self.get_parameter("odom_max_age_s").value),
                max_permit_lifetime_s=float(self.get_parameter("max_permit_lifetime_s").value),
                stationary_linear_max_mps=float(self.get_parameter("stationary_linear_max_mps").value),
                stationary_angular_max_radps=float(self.get_parameter("stationary_angular_max_radps").value),
                footprint_edge_sample_m=float(self.get_parameter("footprint_edge_sample_m").value),
                rollout_step_m=float(self.get_parameter("rollout_step_m").value),
                geometry_epsilon_m=float(self.get_parameter("geometry_epsilon_m").value),
            )
            self._core = MotionGuardCore(cfg)
            self._commissioning_reason = "commissioned"
        except Exception as exc:
            self._core = None
            self._commissioning_reason = f"commissioning_invalid:{exc}"
            self.get_logger().error(self._commissioning_reason)

    def _now_ros_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_raw_command(self, msg: Twist) -> None:
        now_mono = time.monotonic()
        now_ros = self._now_ros_s()
        vals = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(float(v)) for v in vals):
            self._raw_command = None
            self._boundary_reason = "raw_command_nonfinite"
            return
        self._raw_command = RawCommand(
            stamp_s=now_ros,
            receipt_monotonic_s=now_mono,
            linear_x_mps=float(msg.linear.x),
            linear_y_mps=float(msg.linear.y),
            angular_z_radps=float(msg.angular.z),
        )

    def _on_permit(self, msg: MotionPermit) -> None:
        receipt = time.monotonic()
        try:
            permit = motion_permit_from_msg(
                msg,
                receipt_monotonic_s=receipt,
                map_frame=self._map_frame,
            )
        except Exception as exc:
            self._permit = None
            self._boundary_reason = f"permit_conversion_failed:{exc}"
            return
        if not self._navigator_instance_id:
            self._navigator_instance_id = permit.navigator_instance_id
        self._permit = permit
        self._boundary_reason = "permit_received"

    def _on_map(self, msg: MapState) -> None:
        receipt = time.monotonic()
        try:
            snapshot = map_state_from_msg(msg, map_frame=self._map_frame)
            stamp_s = stamp_to_seconds(msg.header.stamp)
        except Exception as exc:
            self._map_snapshot = None
            self._boundary_reason = f"map_conversion_failed:{exc}"
            return
        self._map_snapshot = snapshot
        self._map_stamp_s = stamp_s
        self._map_receipt_monotonic_s = receipt

    def _on_odom(self, msg: Odometry) -> None:
        receipt = time.monotonic()
        try:
            odom, pose = odometry_to_guard_samples(
                msg,
                receipt_monotonic_s=receipt,
                map_frame=self._map_frame,
                allow_map_frame_pose=self._allow_map_frame_odom_pose,
            )
        except Exception as exc:
            self._odom = None
            self._boundary_reason = f"odom_conversion_failed:{exc}"
            return
        self._odom = odom
        if pose is not None:
            self._pose = pose
            self._pose_receipt_monotonic_s = receipt

    def _on_emergency_stop(self, msg: Bool) -> None:
        self._emergency_stop_active = bool(msg.data)

    def _refresh_pose_from_tf(self) -> None:
        if self._allow_map_frame_odom_pose:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
            )
        except TransformException as exc:
            self._pose = None
            self._boundary_reason = f"tf_unavailable:{exc}"
            return
        try:
            stamp_s = stamp_to_seconds(transform.header.stamp)
            pose = transform_to_pose2d(transform)
        except Exception as exc:
            self._pose = None
            self._boundary_reason = f"tf_invalid:{exc}"
            return
        self._pose = PoseSample(timestamp_s=stamp_s, pose=pose)
        self._pose_receipt_monotonic_s = time.monotonic()

    def _evaluate(self) -> None:
        if self._core is None:
            self._publish_zero()
            self._publish_state(reason=self._commissioning_reason)
            return

        self._refresh_pose_from_tf()
        if self._map_snapshot is None:
            self._publish_zero()
            self._publish_state(reason=self._boundary_reason or "map_missing")
            return

        now_mono = time.monotonic()
        inputs = GuardInputs(
            now_ros_s=self._now_ros_s(),
            run_id=self._map_snapshot.run_id,
            map_instance_id=self._map_snapshot.map_instance_id,
            navigator_instance_id=self._navigator_instance_id,
            field_id=self._map_snapshot.field_id,
            permit=self._permit,
            raw_command=self._raw_command,
            map_snapshot=self._map_snapshot,
            map_stamp_s=self._map_stamp_s,
            map_receipt_monotonic_s=self._map_receipt_monotonic_s,
            pose=self._pose,
            pose_receipt_monotonic_s=self._pose_receipt_monotonic_s,
            odom=self._odom,
            emergency_stop_active=self._emergency_stop_active,
        )
        decision = self._core.evaluate(inputs, now_mono)

        out = Twist()
        out.linear.x = float(decision.linear_x_mps)
        out.linear.y = float(decision.linear_y_mps)
        out.angular.z = float(decision.angular_z_radps)
        self._driver_pub.publish(out)
        self._publish_state(
            reason=decision.reason,
            motion_allowed=decision.motion_allowed,
            stopped=decision.stopped,
            fault_latched=decision.fault_latched,
            active_segment_id=decision.active_segment_id,
            last_permission_sequence=max(0, int(decision.last_permission_sequence)),
        )

    def _publish_zero(self) -> None:
        self._driver_pub.publish(Twist())

    def _publish_state(
        self,
        *,
        reason: str,
        motion_allowed: bool = False,
        stopped: bool = False,
        fault_latched: bool = False,
        active_segment_id: str = "",
        last_permission_sequence: int = 0,
    ) -> None:
        msg = GuardState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        if self._map_snapshot is not None:
            msg.run_id = self._map_snapshot.run_id
            msg.map_instance_id = self._map_snapshot.map_instance_id
        msg.navigator_instance_id = self._navigator_instance_id
        msg.goal_generation = int(self._permit.goal_generation) if self._permit is not None else 0
        msg.last_permission_sequence = int(last_permission_sequence)
        msg.active_segment_id = active_segment_id
        msg.motion_allowed = bool(motion_allowed)
        msg.stopped = bool(stopped)
        msg.fault_latched = bool(fault_latched)
        msg.reason = str(reason)
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
