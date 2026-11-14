"""ROS 2 boundary for the Frob City web operator dashboard."""

from __future__ import annotations

from pathlib import Path
import threading
import time

import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from tf2_ros import Buffer, TransformException, TransformListener

from frob_city_interfaces.msg import GuardState, MapState, MissionState, MotionPermit, SignDetectionArray
from frob_city_interfaces.srv import ControlMission
from frob_city_navigation.config_loader import load_project_config
from frob_city_navigation.models import ConfigPaths

from .calibration_service import CalibrationService
from .calibration_store import CalibrationStore
from .http_server import ControlResult, DashboardServer
from .map_catalog import MapCatalog
from .runtime import ControlMailbox, ImageThrottle, source_health
from .serializers import (
    detections_json,
    diagnostics_json,
    field_layout_json,
    guard_json,
    map_state_json,
    mission_json,
    motion_permit_json,
    occupancy_json,
    odom_json,
    path_json,
    stamp_seconds,
    twist_json,
)
from .state_store import StateStore


class FrobCityWebNode(Node):
    """Observe city autonomy and proxy only the canonical mission service."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("frob_city_web", parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._store = StateStore(
            delta_limit=int(self.get_parameter("delta_history_limit").value),
            event_limit=int(self.get_parameter("event_history_limit").value),
            trail_limit=int(self.get_parameter("trail_history_limit").value),
        )
        self._mailbox = ControlMailbox(limit=8)
        self._bridge = CvBridge()
        self._image_throttle = ImageThrottle(
            max_fps=float(self.get_parameter("camera_max_fps").value),
            jpeg_quality=int(self.get_parameter("camera_jpeg_quality").value),
        )
        self._source_receipts: dict[str, float] = {}
        self._last_mission_key = None
        self._last_guard_key = None
        self._control_future = None
        self._control_pending = None
        self._web_subscriptions = []

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._create_ros_endpoints()
        self._load_field_layout()
        self._store.update("meta", {
            "name": "Frob City Control",
            "version": "0.1.0",
            "started_wall_s": time.time(),
            "map_frame": self._map_frame,
            "base_frame": self._base_frame,
        })

        static_root = Path(__file__).resolve().parent / "static"
        map_roots = [Path(value).expanduser() for value in self.get_parameter("calibration_map_roots").value]
        try:
            map_roots.append(Path(get_package_share_directory("frob_navigation")) / "maps")
        except PackageNotFoundError:
            pass
        calibration = CalibrationService(
            MapCatalog(map_roots),
            CalibrationStore(Path(str(self.get_parameter("calibration_root").value)).expanduser()),
            live_map_snapshot=self._calibration_live_map,
            live_map_max_age_s=float(self.get_parameter("calibration_live_map_stale_s").value),
        )
        self._server = DashboardServer(
            self._store,
            static_root,
            self._submit_control,
            host=str(self.get_parameter("http_host").value),
            port=int(self.get_parameter("http_port").value),
            control_timeout_s=float(self.get_parameter("control_timeout_s").value),
            calibration=calibration,
        )
        self._server.start()
        self._timers = [
            self.create_timer(0.05, self._drain_control),
            self.create_timer(0.10, self._update_tf_pose),
            self.create_timer(0.50, self._update_source_health),
        ]
        self._store.append_event("info", "web", "Веб-интерфейс запущен", {"url": self._server.url})
        self.get_logger().info(f"Frob City web dashboard listening on {self._server.url}")

    def _declare_parameters(self) -> None:
        defaults = {
            "http_host": "0.0.0.0", "http_port": 8080,
            "map_frame": "map", "base_frame": "base_footprint",
            "map_topic": "/map", "odom_topic": "/odom",
            "map_state_topic": "/city/map_state",
            "mission_state_topic": "/city/mission_state",
            "planned_route_topic": "/city/planned_route",
            "guard_state_topic": "/city/guard_state",
            "motion_permit_topic": "/city/motion_permit",
            "detections_topic": "/traffic_sign/detections",
            "annotated_image_topic": "/traffic_sign/annotated_image",
            "annotated_compressed_topic": "/traffic_sign/annotated_image/compressed",
            "diagnostics_topic": "/diagnostics",
            "map_planner_diagnostics_topic": "/city/map_planner_diagnostics",
            "raw_cmd_topic": "/city/cmd_vel_raw", "final_cmd_topic": "/cmd_vel",
            "control_service": "/city/control_mission",
            "camera_max_fps": 5.0, "camera_jpeg_quality": 70,
            "control_timeout_s": 2.0, "state_stale_s": 2.0,
            "motion_stale_s": 0.5, "camera_stale_s": 2.0,
            "delta_history_limit": 256, "event_history_limit": 250,
            "trail_history_limit": 500,
            "field_config_path": "", "event_config_path": "",
            "perception_config_path": "", "navigation_config_path": "",
            "calibration_root": "~/Frob_gorod/calibration",
            "calibration_map_roots": [
                "~/Frob_gorod/src/frob_navigation/maps",
                "~/Frob_gorod/install/frob_navigation/share/frob_navigation/maps",
            ],
            "calibration_live_map_stale_s": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _create_ros_endpoints(self) -> None:
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscribe(OccupancyGrid, "map_topic", self._on_map, state_qos)
        self._subscribe(Odometry, "odom_topic", self._on_odom, sensor_qos)
        self._subscribe(MapState, "map_state_topic", self._on_map_state, state_qos)
        self._subscribe(MissionState, "mission_state_topic", self._on_mission, state_qos)
        self._subscribe(PathMsg, "planned_route_topic", self._on_route, state_qos)
        self._subscribe(GuardState, "guard_state_topic", self._on_guard, state_qos)
        self._subscribe(MotionPermit, "motion_permit_topic", self._on_permit, reliable)
        self._subscribe(SignDetectionArray, "detections_topic", self._on_detections, sensor_qos)
        self._subscribe(Image, "annotated_image_topic", self._on_image, sensor_qos)
        self._subscribe(
            CompressedImage,
            "annotated_compressed_topic",
            self._on_compressed_image,
            sensor_qos,
        )
        self._subscribe(DiagnosticArray, "diagnostics_topic", self._on_diagnostics, reliable)
        self._subscribe(
            DiagnosticArray,
            "map_planner_diagnostics_topic",
            self._on_map_planner_diagnostics,
            reliable,
        )
        self._subscribe(Twist, "raw_cmd_topic", lambda msg: self._on_twist("raw_command", msg), reliable)
        self._subscribe(Twist, "final_cmd_topic", lambda msg: self._on_twist("final_command", msg), reliable)
        self._control_client = self.create_client(
            ControlMission, str(self.get_parameter("control_service").value)
        )

    def _subscribe(self, message_type, parameter: str, callback, qos) -> None:
        self._web_subscriptions.append(self.create_subscription(
            message_type, str(self.get_parameter(parameter).value), callback, qos
        ))

    def _received(self, source: str) -> None:
        self._source_receipts[source] = time.monotonic()

    def _safe_update(self, source: str, section: str, convert, msg) -> None:
        self._received(source)
        try:
            self._store.update(section, convert(msg))
        except Exception as exc:
            self._store.append_event("error", source, "Ошибка преобразования ROS-сообщения", {"reason": str(exc)})

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._safe_update("map", "map", occupancy_json, msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._safe_update("odom", "odometry", odom_json, msg)

    def _on_map_state(self, msg: MapState) -> None:
        self._safe_update("map_state", "map_state", map_state_json, msg)

    def _on_route(self, msg: PathMsg) -> None:
        self._safe_update("route", "route", path_json, msg)

    def _on_permit(self, msg: MotionPermit) -> None:
        self._safe_update("permit", "permit", motion_permit_json, msg)

    def _on_detections(self, msg: SignDetectionArray) -> None:
        self._safe_update("detections", "detections", detections_json, msg)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        self._safe_update("diagnostics", "diagnostics", diagnostics_json, msg)

    def _on_map_planner_diagnostics(self, msg: DiagnosticArray) -> None:
        self._safe_update(
            "map_planner_diagnostics",
            "map_planner_diagnostics",
            diagnostics_json,
            msg,
        )

    def _on_twist(self, section: str, msg: Twist) -> None:
        self._received(section)
        payload = twist_json(msg)
        payload["received_wall_s"] = time.time()
        self._store.update(section, payload)

    def _on_mission(self, msg: MissionState) -> None:
        self._received("mission")
        try:
            payload = mission_json(msg)
            self._store.update("mission", payload)
            key = (payload["phase"]["value"], payload["execution_state"]["value"], payload["reason"])
            if key != self._last_mission_key:
                self._last_mission_key = key
                self._store.append_event("info", "mission", f"{payload['phase']['label']} / {payload['execution_state']['label']}", {"reason": payload["reason"]})
        except Exception as exc:
            self._store.append_event("error", "mission", "Ошибка состояния миссии", {"reason": str(exc)})

    def _on_guard(self, msg: GuardState) -> None:
        self._received("guard")
        try:
            payload = guard_json(msg)
            self._store.update("guard", payload)
            key = (payload["motion_allowed"], payload["fault_latched"], payload["reason"])
            if key != self._last_guard_key:
                self._last_guard_key = key
                level = "error" if payload["fault_latched"] else ("info" if payload["motion_allowed"] else "warning")
                self._store.append_event(level, "guard", payload["reason"] or "guard state")
        except Exception as exc:
            self._store.append_event("error", "guard", "Ошибка состояния guard", {"reason": str(exc)})

    def _on_image(self, msg: Image) -> None:
        self._received("camera")
        now = time.monotonic()
        if not self._image_throttle.should_encode(now_s=now, viewers=self._server.camera_viewers):
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg", frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._image_throttle.jpeg_quality],
            )
            if not ok:
                raise ValueError("OpenCV JPEG encoding failed")
            self._store.set_camera(encoded.tobytes(), stamp_seconds(msg.header.stamp))
        except Exception as exc:
            self._store.append_event("warning", "camera", "Не удалось подготовить веб-кадр", {"reason": str(exc)})

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        self._received("camera")
        now = time.monotonic()
        if not self._image_throttle.should_encode(
            now_s=now, viewers=self._server.camera_viewers
        ):
            return
        try:
            if not msg.data:
                raise ValueError("empty compressed image")
            # The laptop already produced a JPEG; forwarding it avoids a second
            # decode/encode pass on the Raspberry Pi.
            self._store.set_camera(bytes(msg.data), stamp_seconds(msg.header.stamp))
        except Exception as exc:
            self._store.append_event(
                "warning",
                "camera",
                "Не удалось принять сжатый веб-кадр",
                {"reason": str(exc)},
            )

    def _update_tf_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = 2.0 * __import__("math").atan2(float(rotation.z), float(rotation.w))
            stamp_s = stamp_seconds(transform.header.stamp)
            payload = {
                "stamp_s": stamp_s, "frame_id": self._map_frame,
                "x_m": float(translation.x), "y_m": float(translation.y),
                "z_m": float(translation.z), "yaw_rad": yaw,
            }
            self._received("tf")
            self._store.update("pose", payload)
            self._store.append_trail(payload["x_m"], payload["y_m"], stamp_s)
        except TransformException:
            return
        except Exception as exc:
            self._store.append_event("warning", "tf", "Некорректное преобразование позиции", {"reason": str(exc)})

    def _load_field_layout(self) -> None:
        names = ("field_config_path", "event_config_path", "perception_config_path", "navigation_config_path")
        values = [str(self.get_parameter(name).value) for name in names]
        if not any(values):
            self._store.update("field", {"available": False, "reason": "configuration_paths_not_set"})
            return
        if not all(values):
            self._store.update("field", {"available": False, "reason": "configuration_paths_incomplete"})
            return
        try:
            config = load_project_config(ConfigPaths(*(Path(value) for value in values)))
            payload = field_layout_json(config)
            payload["available"] = True
            self._store.update("field", payload)
        except Exception as exc:
            self._store.update("field", {"available": False, "reason": str(exc)})

    def _calibration_live_map(self) -> dict | None:
        receipt = self._source_receipts.get("map")
        map_data = self._store.snapshot()["sections"].get("map")
        if receipt is None or map_data is None:
            return None
        return {"map": map_data, "age_s": max(0.0, time.monotonic() - receipt)}

    def _update_source_health(self) -> None:
        now = time.monotonic()
        state_max = float(self.get_parameter("state_stale_s").value)
        motion_max = float(self.get_parameter("motion_stale_s").value)
        camera_max = float(self.get_parameter("camera_stale_s").value)
        motion_sources = {"odom", "tf", "raw_command", "final_command"}
        health = {}
        for source in ("map", "odom", "tf", "map_state", "mission", "route", "guard", "permit", "detections", "camera", "diagnostics", "map_planner_diagnostics", "raw_command", "final_command"):
            limit = camera_max if source == "camera" else (motion_max if source in motion_sources else state_max)
            health[source] = source_health(self._source_receipts.get(source), max_age_s=limit, now_s=now)
        # OccupancyGrid with transient-local durability is intentionally static:
        # receiving it once is sufficient until the map server disappears/restarts.
        if self._source_receipts.get("map") is not None:
            health["map"]["state"] = "ready"
        health["control_service"] = {
            "state": "ready" if self._control_client.service_is_ready() else "missing",
            "age_s": 0.0 if self._control_client.service_is_ready() else None,
            "max_age_s": state_max,
        }
        health["updated_wall_s"] = time.time()
        self._store.update("sources", health)

    def _submit_control(self, command: str, reason: str, timeout_s: float) -> ControlResult:
        result = self._mailbox.submit(command, reason, timeout_s)
        if not isinstance(result, ControlResult):
            raise RuntimeError("invalid control result")
        return result

    def _drain_control(self) -> None:
        if self._control_future is not None:
            return
        pending = self._mailbox.take()
        if pending is None:
            return
        if not self._control_client.service_is_ready():
            self._mailbox.complete(pending, ControlResult(503, {"accepted": False, "reason": "control_service_unavailable"}))
            return
        command_value = getattr(ControlMission.Request, pending.command, None)
        if command_value is None:
            self._mailbox.complete(pending, ControlResult(400, {"accepted": False, "reason": "invalid_command"}))
            return
        request = ControlMission.Request()
        request.command = int(command_value)
        request.reason = pending.reason
        self._control_pending = pending
        self._control_future = self._control_client.call_async(request)
        self._control_future.add_done_callback(self._on_control_result)

    def _on_control_result(self, future) -> None:
        pending = self._control_pending
        self._control_future = None
        self._control_pending = None
        if pending is None:
            return
        try:
            response = future.result()
            payload = {
                "accepted": bool(response.accepted), "run_id": str(response.run_id),
                "phase": int(response.phase), "reason": str(response.reason),
                "command": pending.command,
            }
            status = 200 if response.accepted else 409
            self._store.append_event("info" if response.accepted else "warning", "control", f"{pending.command}: {response.reason}", payload)
            result = ControlResult(status, payload)
        except Exception as exc:
            result = ControlResult(503, {"accepted": False, "command": pending.command, "reason": str(exc)})
        self._mailbox.complete(pending, result)

    def close(self) -> None:
        self._server.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrobCityWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
