"""Deterministic isolated ROS feed for manual dashboard verification."""

import math

import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster

from frob_city_interfaces.msg import (
    ApproachState, EdgeState, GuardState, MapState, MissionState,
    MotionPermit, SignDetection, SignDetectionArray, TargetState,
)
from frob_city_interfaces.srv import ControlMission


def pose(node, x, y, yaw=0.0):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


class FakeDashboardFeed(Node):
    def __init__(self):
        super().__init__("fake_dashboard_feed")
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", state_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", sensor_qos)
        self.map_state_pub = self.create_publisher(MapState, "/city/map_state", state_qos)
        self.mission_pub = self.create_publisher(MissionState, "/city/mission_state", state_qos)
        self.route_pub = self.create_publisher(Path, "/city/planned_route", state_qos)
        self.guard_pub = self.create_publisher(GuardState, "/city/guard_state", state_qos)
        self.permit_pub = self.create_publisher(MotionPermit, "/city/motion_permit", 5)
        self.detection_pub = self.create_publisher(SignDetectionArray, "/traffic_sign/detections", sensor_qos)
        self.image_pub = self.create_publisher(Image, "/traffic_sign/annotated_image", sensor_qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self.raw_pub = self.create_publisher(Twist, "/city/cmd_vel_raw", 5)
        self.final_pub = self.create_publisher(Twist, "/cmd_vel", 5)
        self.tf = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.phase = MissionState.PHASE_BOARDING
        self.sequence = 0
        self.t = 0.0
        self.create_service(ControlMission, "/city/control", self.control)
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_map)

    def control(self, request, response):
        phases = {
            request.START: MissionState.PHASE_BOARDING,
            request.PAUSE: MissionState.PHASE_PAUSED,
            request.RESUME: MissionState.PHASE_BOARDING,
            request.ABORT: MissionState.PHASE_ABORTED,
            request.RESET: MissionState.PHASE_IDLE,
        }
        self.phase = phases[int(request.command)]
        response.accepted = True
        response.run_id = "demo-run-012"
        response.phase = self.phase
        response.reason = f"fake accepted {request.reason}"
        return response

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = "map"
        msg.info = MapMetaData(); msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = 0.05; msg.info.width = 80; msg.info.height = 80
        msg.info.origin.position.x = -2.0; msg.info.origin.position.y = -2.0; msg.info.origin.orientation.w = 1.0
        cells = np.full((80, 80), 100, dtype=np.int8)
        cells[30:50, 4:76] = 0; cells[4:76, 30:50] = 0
        cells[35:45, 35:45] = 0; cells[8:22, 8:22] = -1
        msg.data = cells.flatten().tolist(); self.map_pub.publish(msg)

    def tick(self):
        self.t += 0.04; self.sequence += 1
        x = -1.45 + min(self.t, 2.8); y = 0.0; yaw = 0.0
        now = self.get_clock().now().to_msg()
        transform = TransformStamped(); transform.header.stamp = now; transform.header.frame_id = "map"; transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = x; transform.transform.translation.y = y; transform.transform.rotation.w = 1.0; self.tf.sendTransform(transform)
        odom = Odometry(); odom.header.stamp = now; odom.header.frame_id = "odom"; odom.child_frame_id = "base_footprint"; odom.pose.pose.position.x = x; odom.pose.pose.position.y = y; odom.pose.pose.orientation.w = 1.0; odom.twist.twist.linear.x = 0.18; self.odom_pub.publish(odom)
        self.raw_pub.publish(self.velocity(0.18, 0.04)); self.final_pub.publish(self.velocity(0.17, 0.04))
        route = Path(); route.header.stamp = now; route.header.frame_id = "map"; route.poses = [pose(self, -1.5, 0), pose(self, 0, 0), pose(self, 0, 1.45)]; self.route_pub.publish(route)

        mission = MissionState(); mission.header.stamp = now; mission.header.frame_id = "map"; mission.run_id = "demo-run-012"; mission.navigator_instance_id = "navigator-demo"; mission.phase = self.phase; mission.execution_state = MissionState.EXEC_MOVING if self.phase == MissionState.PHASE_BOARDING else MissionState.EXEC_HOLDING; mission.objective_id = "BUS_STOP_EAST"; mission.route_id = "route-0042"; mission.active_segment_id = "segment-east"; mission.goal_generation = 4; mission.required_unique_bus_stops = 2; mission.serviced_stop_ids = ["BUS_STOP_WEST"]; mission.current_dwell_s = min(2.0, self.t % 3); mission.parking_prior_state = MissionState.PARKING_PRIOR_FOUND; mission.reason = "Движение к восточной остановке"; self.mission_pub.publish(mission)

        guard = GuardState(); guard.header.stamp = now; guard.header.frame_id = "map"; guard.run_id = mission.run_id; guard.map_instance_id = "map-demo"; guard.navigator_instance_id = mission.navigator_instance_id; guard.goal_generation = 4; guard.last_permission_sequence = self.sequence; guard.active_segment_id = mission.active_segment_id; guard.motion_allowed = self.phase == MissionState.PHASE_BOARDING; guard.stopped = not guard.motion_allowed; guard.reason = "motion_permitted" if guard.motion_allowed else "permit_disabled"; self.guard_pub.publish(guard)
        self.publish_map_state(now); self.publish_permit(now, route, guard.motion_allowed); self.publish_detections(now); self.publish_diagnostics(now); self.publish_image(now)

    @staticmethod
    def velocity(linear, angular):
        msg = Twist(); msg.linear.x = linear; msg.angular.z = angular; return msg

    def publish_map_state(self, now):
        msg = MapState(); msg.header.stamp = now; msg.header.frame_id = "map"; msg.run_id = "demo-run-012"; msg.map_instance_id = "map-demo"; msg.field_id = "demo-field"; msg.revision = 7; msg.ready = True; msg.perception_healthy = True; msg.localization_healthy = True; msg.reason = "operational"
        approach = ApproachState(); approach.approach_id = "C1_0/FROM_WEST"; approach.version = 2; approach.state = ApproachState.RESOLVED_SIGNED; approach.confirmed_sign_classes = [SignDetection.LEFT_ONLY]; approach.allowed_movement_mask = ApproachState.MOVE_LEFT; approach.last_qualified_observation = now; msg.approaches = [approach]
        for ident, kind, px, py in (("BUS_STOP_EAST", TargetState.BUS_STOP, 1.35, 0.25), ("PARKING_NORTH", TargetState.PARKING, 0.2, 1.45)):
            target = TargetState(); target.target_id = ident; target.version = 1; target.kind = kind; target.state = TargetState.CONFIRMED; target.site_id = ident.lower(); target.face_id = "face"; target.directed_road_id = "road"; target.service_anchor_id = "anchor"; target.sign_pose = pose(self, px, py); target.geometry_valid = True; target.service_pose = pose(self, px-0.2, py); target.terminal_limit = pose(self, px, py); target.xy_tolerance_m = 0.05; target.yaw_tolerance_rad = 0.1; target.front_clearance_min_m = 0.2; target.last_seen = now; msg.targets.append(target)
        edge = EdgeState(); edge.edge_id = "east-road"; edge.version = 1; edge.state = EdgeState.OPEN; edge.reason = "clear"; msg.edges = [edge]; self.map_state_pub.publish(msg)

    def publish_permit(self, now, route, enabled):
        msg = MotionPermit(); msg.header.stamp = now; msg.header.frame_id = "map"; msg.run_id = "demo-run-012"; msg.map_instance_id = "map-demo"; msg.navigator_instance_id = "navigator-demo"; msg.field_id = "demo-field"; msg.goal_generation = 4; msg.sequence = self.sequence; msg.enabled = enabled; msg.valid_for_s = 0.2
        seg = msg.segment; seg.segment_id = "segment-east"; seg.kind = seg.CORRIDOR; seg.from_anchor_id = "west"; seg.to_anchor_id = "east"; seg.path = route; seg.terminal_limit = pose(self, 1.5, 0); seg.max_speed_mps = 0.2; seg.max_yaw_rate_radps = 0.5; seg.xy_tolerance_m = 0.05; seg.yaw_tolerance_rad = 0.1; seg.estimated_duration_s = 12.0
        region = PolygonStamped(); region.header.stamp = now; region.header.frame_id = "map"; region.polygon.points = [Point32(x=-1.8,y=-0.45),Point32(x=1.7,y=-0.45),Point32(x=1.7,y=0.45),Point32(x=-1.8,y=0.45)]; seg.allowed_regions = [region]; self.permit_pub.publish(msg)

    def publish_detections(self, now):
        msg = SignDetectionArray(); msg.header.stamp = now; msg.header.frame_id = "camera"; msg.detector_instance_id = "yolo-demo"; msg.frame_sequence = self.sequence; msg.image_width_px = 640; msg.image_height_px = 360; msg.calibration_id = "demo-cal"; msg.frame_valid = True
        for cls, conf, x in ((SignDetection.BUS_STOP, .93, 105), (SignDetection.LEFT_ONLY, .88, 405)):
            item = SignDetection(); item.sign_class = cls; item.confidence = conf; item.x_min_px = x; item.y_min_px = 90; item.width_px = 92; item.height_px = 125; item.range_valid = True; item.range_convention = SignDetection.OPTICAL_Z; item.range_m = 0.84 + x/1000; item.range_stddev_m = .07; item.detector_track_id = f"track-{cls}"; msg.detections.append(item)
        self.detection_pub.publish(msg)

    def publish_diagnostics(self, now):
        msg = DiagnosticArray(); msg.header.stamp = now
        for name, text in (("localization", "stable"), ("nav2", "controller active"), ("detector", "inference 32 ms")):
            status = DiagnosticStatus(); status.level = DiagnosticStatus.OK; status.name = name; status.message = text; status.hardware_id = "frob"; status.values = [KeyValue(key="state", value="ready")]; msg.status.append(status)
        self.diag_pub.publish(msg)

    def publish_image(self, now):
        image = np.zeros((360, 640, 3), dtype=np.uint8); image[:] = (18, 28, 33)
        cv2.line(image, (0, 340), (300, 170), (65, 82, 86), 18); cv2.line(image, (640, 340), (340, 170), (65, 82, 86), 18)
        cv2.rectangle(image, (105, 90), (197, 215), (55, 205, 245), 3); cv2.putText(image, "BUS STOP 0.93", (105, 80), cv2.FONT_HERSHEY_SIMPLEX, .55, (55, 205, 245), 2)
        cv2.rectangle(image, (405, 90), (497, 215), (120, 225, 160), 3); cv2.putText(image, "LEFT 0.88", (405, 80), cv2.FONT_HERSHEY_SIMPLEX, .55, (120, 225, 160), 2)
        cv2.putText(image, "FROB VISION / DEMO FEED", (18, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (210, 225, 220), 1)
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8"); msg.header.stamp = now; msg.header.frame_id = "camera"; self.image_pub.publish(msg)


def main():
    rclpy.init(); node = FakeDashboardFeed()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
