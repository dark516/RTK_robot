from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import shutil
from threading import Lock
import time
from urllib.request import Request, urlopen

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from rover_vision.calibration_library import (
    CalibrationLibrary,
    PROFILES,
    cardinal_direction,
)
from rover_vision.mjpeg_stream import AnnotatedStreamServer
from rover_vision.path_clearance import RoadAppearanceModel, TemporalDecision


DEFAULT_ROI = [0.40, 0.68, 0.60, 0.68, 0.64, 0.90, 0.36, 0.90]


class PathClearanceNode(Node):
    def __init__(self) -> None:
        super().__init__('path_clearance_node')
        self.declare_parameter('camera_url', 'http://127.0.0.1:8091/frame?mode=live')
        self.declare_parameter('camera_timeout_sec', 0.75)
        self.declare_parameter('grid_pose_topic', '/rover/grid_pose')
        self.declare_parameter('pose_timeout_sec', 5.0)
        self.declare_parameter('state_topic', '/road/state')
        self.declare_parameter('result_topic', '/path_clearance')
        self.declare_parameter('debug_topic', '/path_clearance/debug/compressed')
        self.declare_parameter('model_path', '~/.config/sverk-rover/path_clearance_cell_model.npz')
        self.declare_parameter('library_path', '~/.config/sverk-rover/path_clearance')
        self.declare_parameter('roi_points', DEFAULT_ROI)
        self.declare_parameter('max_processing_fps', 5.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('stream_enabled', True)
        self.declare_parameter('stream_host', '0.0.0.0')
        self.declare_parameter('stream_port', 8080)
        self.declare_parameter('blocked_fraction_threshold', 0.10)
        self.declare_parameter('component_fraction_threshold', 0.025)
        self.declare_parameter('width_fraction_threshold', 0.55)
        self.declare_parameter('use_edge_anomalies', False)

        self.camera_url = str(self.get_parameter('camera_url').value)
        self.camera_timeout_sec = max(.1, float(self.get_parameter('camera_timeout_sec').value))
        self.pose_timeout_sec = max(.5, float(self.get_parameter('pose_timeout_sec').value))
        self.max_processing_fps = max(.1, float(self.get_parameter('max_processing_fps').value))
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._minimum_period = 1.0 / self.max_processing_fps
        self._last_processed_at = 0.0
        self._processed_frames = 0
        self._processing_time_total = 0.0
        self._stream_server: AnnotatedStreamServer | None = None
        self._decision = TemporalDecision()
        self._active_profile = 'normal'
        self._latest_frame: np.ndarray | None = None
        self._latest_lock = Lock()
        self._actions: deque[tuple[str, dict]] = deque()
        self._actions_lock = Lock()
        self._pose: dict | None = None
        self._pose_received_at = 0.0

        self.library = CalibrationLibrary(
            str(self.get_parameter('library_path').value),
            self.get_parameter('roi_points').value,
        )
        self._models: dict[str, RoadAppearanceModel | None] = {
            profile: self._load_model(profile) for profile in PROFILES
        }
        self._migrate_legacy_model()
        if not self.library.bridge_poses():
            # Two supplied observations. The reverse approaches can be added
            # from the UI when the rover is physically positioned there.
            self.library.add_bridge_pose(2, 9, 'south')
            self.library.add_bridge_pose(0, 7, 'west')

        self._result_publisher = self.create_publisher(
            String, str(self.get_parameter('result_topic').value), 10
        )
        self._state_publisher = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), 10
        )
        self._debug_publisher = self.create_publisher(
            CompressedImage, str(self.get_parameter('debug_topic').value), 2
        )
        self._pose_subscription = self.create_subscription(
            String,
            str(self.get_parameter('grid_pose_topic').value),
            self._on_grid_pose,
            10,
        )
        self._http_timer = self.create_timer(self._minimum_period, self._poll_http_frame)
        if bool(self.get_parameter('stream_enabled').value):
            self._stream_server = AnnotatedStreamServer(
                str(self.get_parameter('stream_host').value),
                int(self.get_parameter('stream_port').value),
                self.library.roi('normal'),
                on_action=self._web_action,
            )
            self._stream_server.start()
            self.get_logger().info(f'Annotated stream listening on port {self._stream_server.port}')

    def _new_model(self, profile: str, sample_count: int = 0) -> RoadAppearanceModel:
        return RoadAppearanceModel(
            self.library.roi(profile),
            color_clusters=min(12, max(6, 4 + sample_count * 2)),
            blocked_fraction_threshold=float(self.get_parameter('blocked_fraction_threshold').value),
            component_fraction_threshold=float(self.get_parameter('component_fraction_threshold').value),
            width_fraction_threshold=float(self.get_parameter('width_fraction_threshold').value),
            use_edge_anomalies=bool(self.get_parameter('use_edge_anomalies').value),
        )

    def _load_model(self, profile: str) -> RoadAppearanceModel | None:
        path = self.library.model_path(profile)
        if not path.exists():
            return None
        try:
            return RoadAppearanceModel.load(path)
        except Exception as error:
            self.get_logger().error(f'Unable to load {profile} road model: {error}')
            return None

    def _migrate_legacy_model(self) -> None:
        if self._models['normal'] is not None:
            return
        legacy = Path(str(self.get_parameter('model_path').value)).expanduser()
        if not legacy.exists():
            return
        try:
            model = RoadAppearanceModel.load(legacy)
            self.library.set_roi('normal', model.roi_points.reshape(-1).tolist())
            shutil.copy2(legacy, self.library.model_path('normal'))
            self._models['normal'] = model
            self.get_logger().info('Migrated legacy road model to the normal profile')
        except Exception as error:
            self.get_logger().error(f'Unable to migrate legacy road model: {error}')

    def _on_grid_pose(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            cell = payload['cell']
            view = payload['view_direction']
            pose = {
                'row': int(cell['row']),
                'column': int(cell['column']),
                'direction': cardinal_direction(float(view['x']), float(view['y'])),
            }
            self._pose = pose
            self._pose_received_at = time.monotonic()
            selected = self.library.profile_for_pose(**pose)
            if selected != self._active_profile:
                self._active_profile = selected
                self._decision = TemporalDecision()
                self.get_logger().info(f'Path-clearance profile switched to {selected}')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'Invalid /rover/grid_pose payload: {error}')

    def _pose_fresh(self) -> bool:
        return self._pose is not None and time.monotonic() - self._pose_received_at <= self.pose_timeout_sec

    def _queue_action(self, action: str, payload: dict) -> None:
        with self._actions_lock:
            self._actions.append((action, dict(payload)))

    def _web_action(self, action: str, payload: dict) -> dict:
        profile = str(payload.get('profile', 'normal'))
        if action == 'config':
            if profile not in PROFILES:
                raise ValueError('profile must be normal or bridge')
            return {
                'active_profile': self._active_profile,
                'pose': self._pose if self._pose_fresh() else None,
                'profiles': {name: {'roi_points': self.library.roi(name)} for name in PROFILES},
                'samples': self.library.list_frames(profile),
                'bridge_poses': self.library.bridge_poses(),
            }
        if action == 'sample_path':
            return {'path': str(self.library.frame_path(profile, str(payload['id'])))}
        if action == 'set_profile':
            self.library.set_roi(profile, payload['roi_points'])
            self._queue_action('rebuild', {'profile': profile})
            return {'accepted': True, 'rebuild': True}
        if action == 'add_sample':
            with self._latest_lock:
                if self._latest_frame is None:
                    raise ValueError('No camera frame is available yet')
                frame = self._latest_frame.copy()
            item = self.library.add_frame(profile, frame)
            self._queue_action('rebuild', {'profile': profile})
            return {'accepted': True, 'sample': item}
        if action == 'delete_sample':
            removed = self.library.remove_frame(profile, str(payload['id']))
            self._queue_action('rebuild', {'profile': profile})
            return {'accepted': True, 'removed': removed}
        if action == 'add_bridge_pose':
            if not self._pose_fresh() or self._pose is None:
                raise ValueError('No fresh /rover/grid_pose value')
            return {'accepted': True, 'pose': self.library.add_bridge_pose(**self._pose)}
        if action == 'delete_bridge_pose':
            poses = self.library.bridge_poses()
            index = int(payload['index'])
            if index < 0 or index >= len(poses):
                raise ValueError('Bridge pose index is out of range')
            removed = self.library.remove_bridge_pose(**poses[index])
            return {'accepted': True, 'removed': removed}
        raise ValueError(f'Unknown action: {action}')

    def _apply_actions(self) -> None:
        with self._actions_lock:
            actions = list(self._actions)
            self._actions.clear()
        rebuild_profiles = {str(payload['profile']) for action, payload in actions if action == 'rebuild'}
        for profile in rebuild_profiles:
            frames = self.library.load_frames(profile)
            if not frames:
                self._models[profile] = None
                path = self.library.model_path(profile)
                if path.exists():
                    path.unlink()
                self.get_logger().warning(f'{profile} profile has no calibration frames')
                continue
            try:
                model = self._new_model(profile, len(frames))
                model.fit(frames)
                model.clear_cell_count = len(frames)
                model.save(self.library.model_path(profile))
                self._models[profile] = model
                if profile == self._active_profile:
                    self._decision = TemporalDecision()
                self.get_logger().info(f'Rebuilt {profile} profile from {len(frames)} full frames')
            except Exception as error:
                self.get_logger().error(f'Unable to rebuild {profile} profile: {error}')

    def _publish_result(self, result: dict) -> None:
        message = String(); message.data = json.dumps(result, ensure_ascii=False, separators=(',', ':'))
        self._result_publisher.publish(message)
        state_message = String(); state = str(result.get('state', 'unknown'))
        state_message.data = state if state in {'clear', 'blocked'} else 'unknown'
        self._state_publisher.publish(state_message)

    def _publish_debug(self, frame: np.ndarray, state: str,
                       warped: np.ndarray | None = None, mask: np.ndarray | None = None) -> None:
        model = self._models.get(self._active_profile)
        annotated = frame.copy()
        roi = model.roi_points if model is not None else np.asarray(self.library.roi(self._active_profile), dtype=np.float32).reshape(4, 2)
        height, width = frame.shape[:2]
        points = (roi * np.array([width - 1, height - 1], dtype=np.float32)).round().astype(np.int32)
        color = (0,220,0) if state == 'clear' else ((0,0,230) if state == 'blocked' else (0,190,255))
        cv2.polylines(annotated, [points], True, color, 4)
        cv2.putText(annotated, f'{self._active_profile} | path: {state}', (25,55), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3, cv2.LINE_AA)
        annotated = cv2.resize(annotated, (640,512))
        if warped is not None and mask is not None:
            preview=cv2.resize(warped,(240,360)); mask_color=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR);mask_color[:,:,:2]=0
            annotated=np.hstack([annotated,cv2.resize(np.vstack([preview,mask_color]),(171,512))])
        ok,encoded=cv2.imencode('.jpg',annotated,[cv2.IMWRITE_JPEG_QUALITY,80])
        source_ok,source_encoded=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,82])
        if not ok:return
        if self.publish_debug_image:
            message=CompressedImage();message.header.stamp=self.get_clock().now().to_msg();message.format='jpeg';message.data=encoded.tobytes();self._debug_publisher.publish(message)
        if self._stream_server:
            count=len(self.library.list_frames(self._active_profile))
            self._stream_server.update(encoded.tobytes(),source_encoded.tobytes() if source_ok else None,state,count,{'active_profile':self._active_profile,'pose':self._pose if self._pose_fresh() else None})

    def _poll_http_frame(self) -> None:
        self._apply_actions()
        try:
            request=Request(self.camera_url,headers={'Cache-Control':'no-cache'})
            with urlopen(request,timeout=self.camera_timeout_sec) as response:encoded=response.read(5*1024*1024+1)
            if len(encoded)>5*1024*1024:raise ValueError('Camera JPEG exceeds 5 MiB limit')
            frame=cv2.imdecode(np.frombuffer(encoded,dtype=np.uint8),cv2.IMREAD_COLOR)
            if frame is None:raise ValueError('Unable to decode JPEG from camera HTTP endpoint')
            with self._latest_lock:self._latest_frame=frame.copy()
            self._process_frame(frame)
        except Exception as error:
            self.get_logger().error(f'Camera HTTP request failed: {error}',throttle_duration_sec=5.0)
            self._publish_result({'state':'unknown','error':str(error)})
            if self._stream_server:
                self._stream_server.set_status('unknown', {
                    'active_profile': self._active_profile,
                    'pose': self._pose if self._pose_fresh() else None,
                    'camera_error': str(error),
                })

    def _process_frame(self, frame: np.ndarray) -> None:
        now=time.monotonic()
        if now-self._last_processed_at<self._minimum_period:return
        self._last_processed_at=now;started=time.perf_counter();model=self._models.get(self._active_profile)
        if not self._pose_fresh():
            result={'state':'unknown','reason':'grid_pose_stale','profile':self._active_profile}
            self._publish_result(result);self._publish_debug(frame,'unknown');return
        if model is None or not model.calibrated:
            result={'state':'unknown','reason':'profile_not_calibrated','profile':self._active_profile}
            self._publish_result(result);self._publish_debug(frame,'unknown');return
        try:
            metrics,warped,mask=model.analyze(frame);state=self._decision.update(metrics.blocked);processing_ms=(time.perf_counter()-started)*1000
            self._processed_frames+=1;self._processing_time_total+=processing_ms
            result={'state':state,'profile':self._active_profile,'pose':self._pose,'instantaneous_blocked':metrics.blocked,'anomaly_fraction':round(metrics.anomaly_fraction,5),'largest_component_fraction':round(metrics.largest_component_fraction,5),'max_width_fraction':round(metrics.max_width_fraction,5),'processing_ms':round(processing_ms,2),'mean_processing_ms':round(self._processing_time_total/self._processed_frames,2),'clear_cell_count':len(self.library.list_frames(self._active_profile))}
            self._publish_result(result);self._publish_debug(frame,state,warped,mask)
        except Exception as error:
            self.get_logger().error(f'Path-clearance processing failed: {error}')
            self._publish_result({'state':'unknown','error':str(error),'profile':self._active_profile})

    def destroy_node(self) -> None:
        if self._stream_server:self._stream_server.stop();self._stream_server=None
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args);node=PathClearanceNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':main()
