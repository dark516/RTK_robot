from __future__ import annotations

import math
import os
import time
import traceback
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rover_interfaces.msg import LedStripState
from rover_interfaces.srv import SetLedStripState

from rover_agent_mcp.tool_schemas import mcp_tools
from rover_agent_mcp.utils import (
    clamp,
    clamp_int,
    color_to_hex,
    json_dumps,
    normalize_effect,
    parse_bool,
    parse_color,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


class RoverRosBridge(Node):
    """ROS 2 implementation behind the MCP tools.

    This node intentionally exposes only high-level rover capabilities. The LLM
    never receives raw ROS access: tools call service/action/topic APIs here.
    """

    def __init__(self) -> None:
        super().__init__('rover_mcp_ros_bridge')

        self.declare_parameter('led_set_state_service', os.getenv('ROVER_LED_SERVICE', '/led_strip/set_state'))
        self.declare_parameter('led_state_topic', os.getenv('ROVER_LED_STATE_TOPIC', '/led_strip/state'))
        self.declare_parameter('cmd_vel_topic', os.getenv('ROVER_CMD_VEL_TOPIC', '/cmd_vel_test'))
        self.declare_parameter('nav2_action_name', os.getenv('ROVER_NAV_ACTION', '/navigate_to_pose'))
        self.declare_parameter('odom_topic', os.getenv('ROVER_ODOM_TOPIC', '/odom'))
        self.declare_parameter('amcl_pose_topic', os.getenv('ROVER_AMCL_POSE_TOPIC', '/amcl_pose'))
        self.declare_parameter('scan_topic', os.getenv('ROVER_SCAN_TOPIC', '/scan_filtered'))
        self.declare_parameter('default_forward_distance_m', 0.30)
        self.declare_parameter('default_forward_speed_mps', 0.12)
        self.declare_parameter('default_lateral_speed_mps', 0.10)
        self.declare_parameter('default_angular_speed_degps', 45.0)
        self.declare_parameter('max_relative_distance_m', 3.0)
        self.declare_parameter('max_relative_turn_deg', 720.0)
        self.declare_parameter('max_drive_duration_s', 40.0)
        self.declare_parameter('motion_position_tolerance_m', 0.025)
        self.declare_parameter('motion_yaw_tolerance_deg', 3.0)
        self.declare_parameter('motion_command_rate_hz', 20.0)
        # If the physical lidar is mounted backwards, set scan_front_angle_deg:=180.0.
        self.declare_parameter('scan_front_angle_deg', 0.0)

        self.led_set_state_service = str(self.get_parameter('led_set_state_service').value)
        self.led_state_topic = str(self.get_parameter('led_state_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.nav2_action_name = str(self.get_parameter('nav2_action_name').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.amcl_pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)

        self._led_client = self.create_client(SetLedStripState, self.led_set_state_service)
        self._cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        self._led_state: LedStripState | None = None
        self._last_led_state_time = 0.0
        self._odom: Odometry | None = None
        self._last_odom_time = 0.0
        self._amcl_pose: PoseWithCovarianceStamped | None = None
        self._last_amcl_time = 0.0
        self._scan: LaserScan | None = None
        self._last_scan_time = 0.0

        self.create_subscription(LedStripState, self.led_state_topic, self._on_led_state, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_amcl_pose, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)

        self._current_goal_handle = None
        self._last_nav_goal: dict[str, Any] | None = None
        self._nav_status: dict[str, Any] = {
            'active': False,
            'status': 'idle',
            'message': 'No navigation goal has been sent.',
        }
        self._last_nav_feedback: dict[str, Any] | None = None

    def _on_led_state(self, msg: LedStripState) -> None:
        self._led_state = msg
        self._last_led_state_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._last_odom_time = time.monotonic()

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl_pose = msg
        self._last_amcl_time = time.monotonic()

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._last_scan_time = time.monotonic()

    def call_tool(self, name: str, arguments: dict[str, Any] | None = None) -> dict[str, Any]:
        args = dict(arguments or {})
        tool_map = {
            'get_available_tools': self.get_available_tools,
            'wait': self.wait,
            'set_led_strip': self.set_led_strip,
            'set_led_preset': self.set_led_preset,
            'blink_led_strip': self.blink_led_strip,
            'drive_relative': self.drive_relative,
            'drive_forward': self.drive_forward,  # compatibility alias
            'turn_relative': self.turn_relative,
            'run_motion_sequence': self.run_motion_sequence,
            'run_relative_sequence': self.run_motion_sequence,  # compatibility alias
            'stop_motion': self.stop_motion,
            'navigate_to_pose': self.navigate_to_pose,
            'cancel_navigation': self.cancel_navigation,
            'get_navigation_status': self.get_navigation_status,
            'is_navigation_ready': self.is_navigation_ready,
            'get_robot_pose': self.get_robot_pose,
            'get_laser_summary': self.get_laser_summary,
            'get_led_strip_state': self.get_led_strip_state,
            'get_system_status': self.get_system_status,
        }
        if name not in tool_map:
            return {'success': False, 'error': f'Unknown tool: {name}'}
        try:
            return tool_map[name](**args)
        except Exception as exc:  # pragma: no cover - runtime guard for ROS demos
            self.get_logger().error(f'Tool {name} failed: {exc}\n{traceback.format_exc()}')
            return {'success': False, 'error': str(exc), 'tool': name}

    def _wait_future(self, future: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return bool(future.done())

    @staticmethod
    def _duration_to_sec(duration_msg: Any) -> float | None:
        if duration_msg is None:
            return None
        sec = getattr(duration_msg, 'sec', None)
        nanosec = getattr(duration_msg, 'nanosec', None)
        if sec is None or nanosec is None:
            return None
        return float(sec) + float(nanosec) * 1e-9

    @staticmethod
    def _goal_status_name(status_code: int | None) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(int(status_code), f'UNKNOWN_{status_code}') if status_code is not None else 'UNKNOWN'

    def _on_nav_feedback(self, feedback_msg: Any) -> None:
        feedback = getattr(feedback_msg, 'feedback', None)
        if feedback is None:
            return
        current_pose = getattr(feedback, 'current_pose', None)
        pose_payload: dict[str, Any] | None = None
        if current_pose is not None:
            pose = getattr(current_pose, 'pose', None)
            if pose is not None:
                yaw = quaternion_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
                pose_payload = {
                    'frame_id': getattr(getattr(current_pose, 'header', None), 'frame_id', ''),
                    'x': float(pose.position.x),
                    'y': float(pose.position.y),
                    'yaw_deg': math.degrees(yaw),
                }
        self._last_nav_feedback = {
            'distance_remaining_m': float(getattr(feedback, 'distance_remaining', 0.0)),
            'navigation_time_s': self._duration_to_sec(getattr(feedback, 'navigation_time', None)),
            'estimated_time_remaining_s': self._duration_to_sec(getattr(feedback, 'estimated_time_remaining', None)),
            'number_of_recoveries': int(getattr(feedback, 'number_of_recoveries', 0)),
            'current_pose': pose_payload,
            'updated_at_monotonic': time.monotonic(),
        }
        if self._nav_status.get('active'):
            self._nav_status['status'] = 'executing'
            self._nav_status['message'] = 'Nav2 goal is executing.'
            self._nav_status['feedback'] = self._last_nav_feedback

    def _make_twist(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0) -> Twist:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        return msg

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(self._make_twist(0.0, 0.0, 0.0))

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def _angle_delta(current_rad: float, start_rad: float) -> float:
        return math.atan2(math.sin(current_rad - start_rad), math.cos(current_rad - start_rad))

    def _wait_for_odom(self, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while rclpy.ok() and self._odom is None and time.monotonic() < deadline:
            time.sleep(0.02)
        return self._odom is not None

    def _odom_pose_xy_yaw(self) -> tuple[float, float, float] | None:
        if self._odom is None:
            return None
        pose = self._odom.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return float(pose.position.x), float(pose.position.y), float(yaw)

    def get_available_tools(self) -> dict[str, Any]:
        tools = mcp_tools()
        categories = {
            'general': ['get_available_tools', 'wait'],
            'led': ['set_led_strip', 'set_led_preset', 'blink_led_strip', 'get_led_strip_state'],
            'relative_motion': ['drive_relative', 'turn_relative', 'run_motion_sequence', 'stop_motion'],
            'nav2': ['navigate_to_pose', 'cancel_navigation', 'get_navigation_status', 'is_navigation_ready', 'get_robot_pose'],
            'diagnostics': ['get_laser_summary', 'get_system_status'],
            'compatibility_aliases': ['drive_forward', 'run_relative_sequence'],
        }
        return {
            'success': True,
            'tool_count': len(tools),
            'categories': categories,
            'tools': [{'name': tool.get('name'), 'description': tool.get('description', '')} for tool in tools],
        }

    def wait(self, duration_s: float = 1.0) -> dict[str, Any]:
        duration = float(clamp(duration_s, 0.0, 60.0))
        time.sleep(duration)
        return {'success': True, 'message': f'Waited {duration:.2f} seconds.', 'duration_s': duration}

    def set_led_strip(
        self,
        enabled: bool = True,
        effect: str = 'fill',
        brightness: float = 0.35,
        color: str = '#16B8F3',
        secondary_color: str = '#FFFFFF',
        effect_speed_hz: float = 1.0,
    ) -> dict[str, Any]:
        if not self._led_client.wait_for_service(timeout_sec=2.0):
            return {'success': False, 'error': f'LED service {self.led_set_state_service} is not available.'}

        red, green, blue = parse_color(color)
        secondary_red, secondary_green, secondary_blue = parse_color(secondary_color, '#FFFFFF')

        req = SetLedStripState.Request()
        req.enabled = parse_bool(enabled, True)
        req.brightness = float(clamp(brightness, 0.0, 1.0))
        req.effect = normalize_effect(effect)
        req.effect_speed_hz = float(clamp(effect_speed_hz, 0.05, 20.0))
        req.red = clamp_int(red, 0, 255)
        req.green = clamp_int(green, 0, 255)
        req.blue = clamp_int(blue, 0, 255)
        req.secondary_red = clamp_int(secondary_red, 0, 255)
        req.secondary_green = clamp_int(secondary_green, 0, 255)
        req.secondary_blue = clamp_int(secondary_blue, 0, 255)

        future = self._led_client.call_async(req)
        if not self._wait_future(future, 3.0):
            return {'success': False, 'error': 'Timed out waiting for LED service response.'}
        resp = future.result()
        return {
            'success': bool(resp.success),
            'message': str(resp.message),
            'enabled': bool(req.enabled),
            'effect': req.effect,
            'brightness': req.brightness,
            'color': color_to_hex(req.red, req.green, req.blue),
            'secondary_color': color_to_hex(req.secondary_red, req.secondary_green, req.secondary_blue),
        }

    def set_led_preset(self, preset: str) -> dict[str, Any]:
        name = str(preset).strip().lower()
        presets = {
            'off': dict(enabled=False, effect='fill', color='#000000', brightness=0.0),
            'idle': dict(enabled=True, effect='fill', color='#16B8F3', brightness=0.18),
            'zima_blue': dict(enabled=True, effect='fill', color='#16B8F3', brightness=0.35),
            'blue': dict(enabled=True, effect='fill', color='#0000FF', brightness=0.35),
            'cyan': dict(enabled=True, effect='fill', color='#00FFFF', brightness=0.35),
            'green': dict(enabled=True, effect='fill', color='#00FF00', brightness=0.35),
            'red': dict(enabled=True, effect='fill', color='#FF0000', brightness=0.35),
            'white': dict(enabled=True, effect='fill', color='#FFFFFF', brightness=0.35),
            'yellow': dict(enabled=True, effect='fill', color='#FFFF00', brightness=0.35),
            'purple': dict(enabled=True, effect='fill', color='#8000FF', brightness=0.35),
            'rainbow': dict(enabled=True, effect='rainbow', color='#16B8F3', brightness=0.35, effect_speed_hz=0.5),
            'thinking': dict(enabled=True, effect='fade', color='#16B8F3', brightness=0.35, effect_speed_hz=0.7),
            'navigation': dict(enabled=True, effect='wipe', color='#16B8F3', brightness=0.35, effect_speed_hz=1.5),
            'manual_control': dict(enabled=True, effect='fill', color='#FFFFFF', brightness=0.25),
            'warning': dict(enabled=True, effect='blink_fast', color='#FF8000', brightness=0.55, effect_speed_hz=4.0),
            'blink_blue': dict(enabled=True, effect='blink', color='#16B8F3', brightness=0.35, effect_speed_hz=2.0),
            'success': dict(enabled=True, effect='flash', color='#00FF00', brightness=0.45, effect_speed_hz=3.0),
            'error': dict(enabled=True, effect='blink_fast', color='#FF0000', brightness=0.5, effect_speed_hz=4.0),
        }
        if name not in presets:
            return {'success': False, 'error': f'Unknown LED preset: {preset}', 'available_presets': sorted(presets)}
        result = self.set_led_strip(**presets[name])
        result['preset'] = name
        return result

    def blink_led_strip(
        self,
        color: str = '#16B8F3',
        times: int = 3,
        interval_s: float = 0.35,
        brightness: float = 0.35,
        restore: str = 'steady',
    ) -> dict[str, Any]:
        blink_count = clamp_int(times, 1, 20)
        interval = float(clamp(interval_s, 0.05, 5.0))
        bright = float(clamp(brightness, 0.0, 1.0))
        chosen_color = color_to_hex(*parse_color(color))
        for _ in range(blink_count):
            self.set_led_strip(True, 'fill', bright, chosen_color)
            time.sleep(interval)
            self.set_led_strip(False, 'fill', 0.0, '#000000')
            time.sleep(interval)
        restore_mode = str(restore).strip().lower()
        if restore_mode in {'steady', 'previous', 'on'}:
            self.set_led_strip(True, 'fill', bright, chosen_color)
        else:
            self.set_led_strip(False, 'fill', 0.0, '#000000')
        return {
            'success': True,
            'message': f'LED strip blinked {blink_count} times.',
            'color': chosen_color,
            'times': blink_count,
            'interval_s': interval,
            'restore': restore,
        }

    def drive_relative(
        self,
        forward_m: float = 0.30,
        left_m: float = 0.0,
        speed_mps: float | None = None,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        """Drive in the robot's local frame using odometry feedback.

        forward_m is +X in base_link, left_m is +Y in base_link. This is suitable
        for mecanum/omni bases that accept Twist.linear.y.
        """
        if not self._wait_for_odom(timeout_s=2.0):
            return {'success': False, 'error': f'No odometry received on {self.odom_topic}; cannot do odom-based relative motion.'}

        max_distance = float(self.get_parameter('max_relative_distance_m').value)
        target_forward = float(clamp(float(forward_m), -max_distance, max_distance))
        target_left = float(clamp(float(left_m), -max_distance, max_distance))
        target_dist = math.hypot(target_forward, target_left)
        if target_dist < 1e-4:
            self._publish_stop()
            return {'success': True, 'message': 'Relative drive target is zero; no movement needed.', 'forward_m': 0.0, 'left_m': 0.0}

        default_speed = float(self.get_parameter('default_forward_speed_mps').value)
        speed_abs = abs(float(speed_mps if speed_mps is not None else default_speed))
        speed_abs = float(clamp(speed_abs, 0.02, 0.45))
        tolerance = float(clamp(float(self.get_parameter('motion_position_tolerance_m').value), 0.005, 0.20))
        rate_hz = float(clamp(float(self.get_parameter('motion_command_rate_hz').value), 5.0, 50.0))
        sleep_s = 1.0 / rate_hz
        computed_timeout = target_dist / max(speed_abs, 1e-3) + 3.0
        max_timeout = float(self.get_parameter('max_drive_duration_s').value)
        timeout = float(clamp(timeout_s if timeout_s is not None else computed_timeout, 0.5, max_timeout))

        start = self._odom_pose_xy_yaw()
        if start is None:
            return {'success': False, 'error': 'Odometry disappeared before motion start.'}
        start_x, start_y, start_yaw = start
        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)
        unit_forward = target_forward / target_dist
        unit_left = target_left / target_dist
        deadline = time.monotonic() + timeout
        last_forward = 0.0
        last_left = 0.0
        remaining = target_dist

        try:
            while rclpy.ok() and time.monotonic() < deadline:
                pose = self._odom_pose_xy_yaw()
                if pose is None:
                    break
                cur_x, cur_y, _ = pose
                dx = cur_x - start_x
                dy = cur_y - start_y
                # Project odom displacement into the start base_link frame.
                last_forward = cos_yaw * dx + sin_yaw * dy
                last_left = -sin_yaw * dx + cos_yaw * dy
                err_forward = target_forward - last_forward
                err_left = target_left - last_left
                remaining = math.hypot(err_forward, err_left)
                if remaining <= tolerance:
                    break

                # Slow down near the target to reduce overshoot.
                cmd_speed = min(speed_abs, max(0.035, remaining * 1.2))
                cmd_x = unit_forward * cmd_speed
                cmd_y = unit_left * cmd_speed
                self._cmd_vel_pub.publish(self._make_twist(linear_x=cmd_x, linear_y=cmd_y, angular_z=0.0))
                time.sleep(sleep_s)
        finally:
            self._publish_stop()

        success = remaining <= max(tolerance * 2.0, 0.05)
        return {
            'success': success,
            'message': 'Completed odom-based relative drive.' if success else 'Relative drive stopped before reaching target tolerance.',
            'target_forward_m': target_forward,
            'target_left_m': target_left,
            'measured_forward_m': last_forward,
            'measured_left_m': last_left,
            'remaining_m': remaining,
            'speed_mps': speed_abs,
            'tolerance_m': tolerance,
            'timeout_s': timeout,
            'cmd_vel_topic': self.cmd_vel_topic,
            'odom_topic': self.odom_topic,
        }

    def drive_forward(self, distance_m: float = 0.30, speed_mps: float | None = None, timeout_s: float | None = None) -> dict[str, Any]:
        return self.drive_relative(forward_m=distance_m, left_m=0.0, speed_mps=speed_mps, timeout_s=timeout_s)

    def turn_relative(
        self,
        angle_deg: float,
        angular_speed_degps: float | None = None,
        timeout_s: float | None = None,
    ) -> dict[str, Any]:
        if not self._wait_for_odom(timeout_s=2.0):
            return {'success': False, 'error': f'No odometry received on {self.odom_topic}; cannot do odom-based relative turn.'}

        max_turn = float(self.get_parameter('max_relative_turn_deg').value)
        target = math.radians(float(clamp(float(angle_deg), -max_turn, max_turn)))
        if abs(target) < math.radians(0.5):
            self._publish_stop()
            return {'success': True, 'message': 'Turn target is zero; no movement needed.', 'angle_deg': 0.0}

        default_speed = float(self.get_parameter('default_angular_speed_degps').value)
        speed_deg_abs = abs(float(angular_speed_degps if angular_speed_degps is not None else default_speed))
        speed_deg_abs = float(clamp(speed_deg_abs, 5.0, 180.0))
        speed_rad_abs = math.radians(speed_deg_abs)
        tolerance_rad = math.radians(float(clamp(float(self.get_parameter('motion_yaw_tolerance_deg').value), 0.5, 15.0)))
        rate_hz = float(clamp(float(self.get_parameter('motion_command_rate_hz').value), 5.0, 50.0))
        sleep_s = 1.0 / rate_hz
        computed_timeout = abs(target) / max(speed_rad_abs, 1e-3) + 3.0
        timeout = float(clamp(timeout_s if timeout_s is not None else computed_timeout, 0.5, 40.0))

        start = self._odom_pose_xy_yaw()
        if start is None:
            return {'success': False, 'error': 'Odometry disappeared before turn start.'}
        _, _, start_yaw = start
        direction = 1.0 if target >= 0.0 else -1.0
        deadline = time.monotonic() + timeout
        traveled = 0.0
        remaining = abs(target)

        try:
            while rclpy.ok() and time.monotonic() < deadline:
                pose = self._odom_pose_xy_yaw()
                if pose is None:
                    break
                _, _, cur_yaw = pose
                traveled = self._angle_delta(cur_yaw, start_yaw)
                remaining_signed = target - traveled
                remaining = abs(remaining_signed)
                if remaining <= tolerance_rad:
                    break
                cmd_speed = min(speed_rad_abs, max(math.radians(8.0), remaining * 1.5))
                self._cmd_vel_pub.publish(self._make_twist(angular_z=direction * cmd_speed))
                time.sleep(sleep_s)
        finally:
            self._publish_stop()

        success = remaining <= max(tolerance_rad * 2.0, math.radians(5.0))
        return {
            'success': success,
            'message': 'Completed odom-based relative turn.' if success else 'Relative turn stopped before reaching target tolerance.',
            'target_angle_deg': math.degrees(target),
            'measured_angle_deg': math.degrees(traveled),
            'remaining_deg': math.degrees(remaining),
            'angular_speed_degps': speed_deg_abs,
            'tolerance_deg': math.degrees(tolerance_rad),
            'timeout_s': timeout,
            'cmd_vel_topic': self.cmd_vel_topic,
            'odom_topic': self.odom_topic,
        }

    def stop_motion(self, cancel_navigation: bool = False) -> dict[str, Any]:
        for _ in range(4):
            self._publish_stop()
            time.sleep(0.03)
        nav_result = None
        if parse_bool(cancel_navigation, False):
            nav_result = self.cancel_navigation()
        return {
            'success': True,
            'message': f'Published stop Twist to {self.cmd_vel_topic}.',
            'cancel_navigation': bool(cancel_navigation),
            'navigation_cancel_result': nav_result,
        }

    def run_motion_sequence(self, steps: list[dict[str, Any]], stop_on_error: bool = True) -> dict[str, Any]:
        results: list[dict[str, Any]] = []
        if not isinstance(steps, list) or not steps:
            return {'success': False, 'error': 'steps must be a non-empty list'}
        aliases = {
            'drive': 'drive_relative',
            'move': 'drive_relative',
            'turn': 'turn_relative',
            'sleep': 'wait',
        }
        for index, raw_step in enumerate(steps):
            step = dict(raw_step or {})
            step_type = str(step.pop('type', '')).strip()
            step_type = aliases.get(step_type, step_type)
            if step_type == 'drive_relative' and 'distance_m' in step and 'forward_m' not in step:
                step['forward_m'] = step.pop('distance_m')
            if step_type == 'navigate_to_pose':
                # In a sequence, a navigation step should block until the action result
                # arrives, then the next step starts immediately. timeout_s remains only
                # a maximum guard, not a fixed wait.
                step.setdefault('wait_until_done', True)
                step.setdefault('timeout_s', 90.0)
            result = self.call_tool(step_type, step)
            result['step_index'] = index
            result['step_type'] = step_type
            results.append(result)
            if parse_bool(stop_on_error, True) and not result.get('success', False):
                self.stop_motion(cancel_navigation=False)
                return {
                    'success': False,
                    'message': f'Sequence stopped at step {index} ({step_type}).',
                    'results': results,
                }
        return {'success': True, 'message': f'Completed {len(results)} sequence steps.', 'results': results}

    def _nav_result_payload(self, nav_result: Any) -> dict[str, Any]:
        status_code = getattr(nav_result, 'status', None)
        status_int = int(status_code) if status_code is not None else None
        status_name = self._goal_status_name(status_int)
        succeeded = status_int == GoalStatus.STATUS_SUCCEEDED
        return {
            'success': bool(succeeded),
            'nav2_status_code': status_int,
            'nav2_status_name': status_name,
            'message': 'Nav2 goal succeeded.' if succeeded else f'Nav2 goal finished with status {status_name}.',
            'feedback': self._last_nav_feedback,
        }

    def _apply_nav_result(self, nav_result: Any) -> dict[str, Any]:
        payload = self._nav_result_payload(nav_result)
        succeeded = bool(payload.get('success', False))
        self._current_goal_handle = None
        self._nav_status = {
            'active': False,
            'status': str(payload.get('nav2_status_name', 'UNKNOWN')).lower(),
            'status_code': payload.get('nav2_status_code'),
            'status_name': payload.get('nav2_status_name'),
            'message': payload.get('message'),
            'last_goal': self._last_nav_goal,
            'feedback': self._last_nav_feedback,
        }
        if not succeeded:
            # Keep explicit failure in status; do not leave an already finished goal as active.
            self._nav_status['active'] = False
        return payload

    def _on_nav_result_future(self, future: Any) -> None:
        try:
            nav_result = future.result()
            self._apply_nav_result(nav_result)
        except Exception as exc:
            self._nav_status = {
                'active': False,
                'status': 'result_error',
                'message': f'Failed to read Nav2 result: {exc}',
                'last_goal': self._last_nav_goal,
                'feedback': self._last_nav_feedback,
            }

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        yaw_deg: float = 0.0,
        frame_id: str = 'map',
        wait_until_done: bool = False,
        timeout_s: float = 60.0,
    ) -> dict[str, Any]:
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            return {'success': False, 'error': f'Nav2 action server {self.nav2_action_name} is not available.'}

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(frame_id or 'map')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(math.radians(float(yaw_deg)))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        if not self._wait_future(send_future, 5.0):
            return {'success': False, 'error': 'Timed out while sending Nav2 goal.'}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._nav_status = {'active': False, 'status': 'rejected', 'message': 'Nav2 rejected the goal.'}
            return {'success': False, 'error': 'Nav2 rejected the goal.'}

        self._current_goal_handle = goal_handle
        self._last_nav_feedback = None
        self._last_nav_goal = {'x': float(x), 'y': float(y), 'yaw_deg': float(yaw_deg), 'frame_id': str(frame_id or 'map')}
        self._nav_status = {
            'active': True,
            'status': 'accepted',
            'message': 'Nav2 goal accepted.',
            'last_goal': self._last_nav_goal,
        }

        result_data = {'success': True, 'message': 'Nav2 goal accepted.', 'goal': self._last_nav_goal, 'action': self.nav2_action_name}
        result_future = goal_handle.get_result_async()

        if parse_bool(wait_until_done, False):
            if self._wait_future(result_future, float(timeout_s)):
                nav_result = result_future.result()
                final_payload = self._apply_nav_result(nav_result)
                result_data.update(final_payload)
            else:
                result_data['success'] = False
                result_data['message'] = 'Timed out waiting for Nav2 result; goal may still be active.'
                result_data['feedback'] = self._last_nav_feedback
        else:
            result_future.add_done_callback(self._on_nav_result_future)
        return result_data

    def cancel_navigation(self) -> dict[str, Any]:
        if self._current_goal_handle is None:
            self._nav_status = {'active': False, 'status': 'idle', 'message': 'No active Nav2 goal.'}
            return {'success': True, 'message': 'No active Nav2 goal to cancel.'}
        cancel_future = self._current_goal_handle.cancel_goal_async()
        if not self._wait_future(cancel_future, 5.0):
            return {'success': False, 'error': 'Timed out while canceling Nav2 goal.'}
        self._current_goal_handle = None
        self._nav_status = {'active': False, 'status': 'canceled', 'message': 'Cancel request sent to Nav2.', 'last_goal': self._last_nav_goal}
        return {'success': True, 'message': 'Cancel request sent to Nav2.'}

    def get_navigation_status(self) -> dict[str, Any]:
        status = dict(self._nav_status)
        status['nav2_action_server_ready'] = self._nav_client.server_is_ready()
        status['robot_pose'] = self.get_robot_pose()
        return status

    def is_navigation_ready(self) -> dict[str, Any]:
        pose = self.get_robot_pose()
        return {
            'success': True,
            'ready': bool(self._nav_client.server_is_ready() and pose.get('success', False)),
            'nav2_action_server': self._nav_client.server_is_ready(),
            'nav2_action_name': self.nav2_action_name,
            'pose_available': bool(pose.get('success', False)),
            'pose_source': pose.get('source'),
            'map_pose_available': bool(pose.get('success', False) and pose.get('frame') == 'map'),
            'odom_received': self._odom is not None,
            'amcl_pose_received': self._amcl_pose is not None,
            'scan_received': self._scan is not None,
            'hint': 'Set initial pose in RViz/AMCL if map pose is not available.',
        }

    def get_robot_pose(self) -> dict[str, Any]:
        now = time.monotonic()
        if self._amcl_pose is not None:
            pose = self._amcl_pose.pose.pose
            yaw = quaternion_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            return {
                'success': True,
                'source': self.amcl_pose_topic,
                'frame': self._amcl_pose.header.frame_id or 'map',
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw_deg': math.degrees(yaw),
                'age_s': now - self._last_amcl_time,
            }
        if self._odom is not None:
            pose = self._odom.pose.pose
            yaw = quaternion_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            return {
                'success': True,
                'source': self.odom_topic,
                'frame': self._odom.header.frame_id or 'odom',
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw_deg': math.degrees(yaw),
                'age_s': now - self._last_odom_time,
            }
        return {'success': False, 'error': 'No AMCL pose or odometry received yet.'}

    def get_laser_summary(self) -> dict[str, Any]:
        if self._scan is None:
            return {'success': False, 'error': 'No LaserScan received yet.'}

        scan = self._scan
        ranges = list(scan.ranges)
        now = time.monotonic()

        def sector_min(center_deg: float, width_deg: float) -> float | None:
            center = math.radians(center_deg)
            half = math.radians(width_deg) * 0.5
            values: list[float] = []
            for idx, value in enumerate(ranges):
                if not math.isfinite(value):
                    continue
                if value < scan.range_min or value > scan.range_max:
                    continue
                angle = scan.angle_min + idx * scan.angle_increment
                delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
                if abs(delta) <= half:
                    values.append(float(value))
            if not values:
                return None
            return min(values)

        front_angle = float(self.get_parameter('scan_front_angle_deg').value)
        return {
            'success': True,
            'source': self.scan_topic,
            'front_angle_deg': front_angle,
            'front_min_m': sector_min(front_angle, 40.0),
            'left_min_m': sector_min(front_angle + 90.0, 60.0),
            'right_min_m': sector_min(front_angle - 90.0, 60.0),
            'back_min_m': sector_min(front_angle + 180.0, 60.0),
            'age_s': now - self._last_scan_time,
            'range_min': scan.range_min,
            'range_max': scan.range_max,
        }

    def get_led_strip_state(self) -> dict[str, Any]:
        if self._led_state is None:
            return {'success': False, 'error': 'No LED strip state received yet.'}
        msg = self._led_state
        return {
            'success': True,
            'connected': bool(msg.connected),
            'enabled': bool(msg.enabled),
            'led_count': int(msg.led_count),
            'lit_count': int(msg.lit_count),
            'brightness': float(msg.brightness),
            'effect': str(msg.effect),
            'effect_speed_hz': float(msg.effect_speed_hz),
            'backend': str(msg.backend),
            'transport': str(msg.transport),
            'status_message': str(msg.status_message),
            'color': color_to_hex(msg.red, msg.green, msg.blue),
            'secondary_color': color_to_hex(msg.secondary_red, msg.secondary_green, msg.secondary_blue),
            'age_s': time.monotonic() - self._last_led_state_time,
        }

    def get_system_status(self) -> dict[str, Any]:
        nav_ready = self.is_navigation_ready()
        return {
            'success': True,
            'led_service': self._led_client.service_is_ready(),
            'led_service_name': self.led_set_state_service,
            'nav2': nav_ready,
            'cmd_vel_topic': self.cmd_vel_topic,
            'odom_received': self._odom is not None,
            'odom_topic': self.odom_topic,
            'amcl_pose_received': self._amcl_pose is not None,
            'amcl_pose_topic': self.amcl_pose_topic,
            'scan_received': self._scan is not None,
            'scan_topic': self.scan_topic,
            'led_state_received': self._led_state is not None,
            'led_state_topic': self.led_state_topic,
            'navigation_status': self._nav_status,
        }

    def result_to_text(self, result: dict[str, Any]) -> str:
        return json_dumps(result)
