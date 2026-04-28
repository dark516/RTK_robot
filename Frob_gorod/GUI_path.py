import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_msgs.action import FollowWaypoints
import threading
import yaml
import os
import io
import json
import logging
from PIL import Image
from flask import Flask, render_template_string, send_file
from flask_socketio import SocketIO

# --- Silence Web Server Console Output ---
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# --- ASCII Art & Emojis ---
ASCII_DICT = {
    'FORWARD': r"""     .--------.
   /          \
  |     /\     |
  |    /  \    |
  |     ||     |
  |     ||     |
   \          /
     '--------'""",
    'LEFT': r"""     .--------.
   /          \
  |            |
  |  <-------. |
  |          | |
  |          | |
   \          /
     '--------'""",
    'RIGHT': r"""     .--------.
   /          \
  |            |
  |  .-------> |
  |  |         |
  |  |         |
   \          /
     '--------'""",
    'NO_LEFT': r"""     .--------.
   / \        \
  |   \        |
  |  <-\-----. |
  |     \    | |
  |      \   | |
   \      \   /
     '--------'""",
    'NO_RIGHT': r"""     .--------.
   / \        \
  |   \        |
  | .--\---->  |
  | |   \      |
  | |    \     |
   \      \   /
     '--------'""",
    'STOP_STATION': r"""    .----------.
    | .------. |
    | |[====]| |
    | |-o--o-| |
    | '------' |
    |          |
    '----------'""",
    'PARKING': r"""    .----------.
    |          |
    |   ____   |
    |   |   \  |
    |   |___/  |
    |   |      |
    '----------'"""
}

EMOJI_DICT = {
    'FORWARD': 'FORWARD', 'LEFT': 'LEFT', 'RIGHT': 'RIGHT',
    'NO_LEFT': 'NO_LEFT', 'NO_RIGHT': 'NO_RIGHT',
    'STOP_STATION': 'STOP_STATION', 'PARKING': 'PARKING'
}

# --- Backend Setup ---
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

map_meta = {}
signs_data = []

PATH_FILE = 'saved_path.json'
ENV_FILE = 'saved_environment.json'


class RobotPlannerNode(Node):
    def __init__(self):
        super().__init__('robot_planner_node')

        self.nav2_wp_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.current_goal_handle = None

        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.stop_pub = self.create_publisher(PoseArray, '/stops', 10)
        self.signs_pub = self.create_publisher(String, '/signs', 10)

        # Tracking /amcl_pose
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)

        self.current_x_m = 0.0
        self.current_y_m = 0.0

        self.waypoint_batches = []
        self.current_batch_idx = 0
        self.path_active = False

    def amcl_callback(self, msg):
        self.current_x_m = msg.pose.pose.position.x
        self.current_y_m = msg.pose.pose.position.y

        socketio.emit('robot_pose', {'x': self.current_x_m, 'y': self.current_y_m})

        if self.path_active:
            trigger_radius_m = 0.375
            for sign in signs_data:
                if not sign['triggered']:
                    dist = math.hypot(self.current_x_m - sign['x_m'], self.current_y_m - sign['y_m'])
                    if dist < trigger_radius_m:
                        sign['triggered'] = True
                        socketio.emit('triggers_updated', signs_data)

                        sign_name = sign['type']
                        ascii_art = ASCII_DICT.get(sign_name, "UNKNOWN SIGN")
                        emoji_art = EMOJI_DICT.get(sign_name, sign_name)

                        msg = String()
                        msg.data = emoji_art
                        self.signs_pub.publish(msg)
                        self.get_logger().info(f'\n---> TRIGGERED SIGN: "{sign_name}"\n{ascii_art}\n')

    def load_batches_and_publish_stops(self, batches, stops_meters):
        self.waypoint_batches = batches
        self.current_batch_idx = 0

        stops_msg = PoseArray()
        stops_msg.header.stamp = self.get_clock().now().to_msg()
        stops_msg.header.frame_id = 'map'
        for x_m, y_m in stops_meters:
            pose = Pose()
            pose.position.x = x_m
            pose.position.y = y_m
            stops_msg.poses.append(pose)
        self.stop_pub.publish(stops_msg)

    def send_next_batch(self):
        if self.current_batch_idx >= len(self.waypoint_batches): return

        current_batch_meters = self.waypoint_batches[self.current_batch_idx]
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        goal_msg = FollowWaypoints.Goal()

        for x_m, y_m in current_batch_meters:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = float(x_m)
            pose_stamped.pose.position.y = float(y_m)
            pose_stamped.pose.orientation.w = 1.0
            goal_msg.poses.append(pose_stamped)
            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)

        if not self.nav2_wp_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 /follow_waypoints server not available!')
            return

        self.send_goal_future = self.nav2_wp_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: return
        self.current_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        print("\n\n" + "=" * 50)
        print(f"✅ BATCH {self.current_batch_idx + 1} COMPLETED!")
        self.current_batch_idx += 1

        if self.current_batch_idx < len(self.waypoint_batches):
            print(f" -> STOP REACHED. Waiting 2 seconds before starting Batch {self.current_batch_idx + 1}...")
            print("=" * 50 + "\n")
            threading.Timer(2.0, self.send_next_batch).start()
        else:
            self.path_active = False
            print("=" * 50 + "\n")

    def cancel_nav2_goal(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None


ros_node = None


def load_hardcoded_map():
    global map_meta
    map_yaml_path = "/home/dark516/Documents/Frob_robot/ros2/src/ros2_ws/src/frob_navigation/maps/map.yaml"

    with open(map_yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)

    image_path = os.path.join(os.path.dirname(map_yaml_path), map_data['image'])
    img = Image.open(image_path)

    map_meta = {
        'res': float(map_data['resolution']),
        'origin_x': float(map_data['origin'][0]),
        'origin_y': float(map_data['origin'][1]),
        'w': img.width,
        'h': img.height,
        'img_path': image_path
    }
    print(f"Map loaded successfully! Resolution: {img.width}x{img.height}")


# --- Network Routes ---
@app.route('/map.png')
def serve_map_image():
    img = Image.open(map_meta['img_path'])
    img_io = io.BytesIO()
    img.save(img_io, 'PNG')
    img_io.seek(0)
    return send_file(img_io, mimetype='image/png')


@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, meta=map_meta, signs=list(ASCII_DICT.keys()))


@socketio.on('save_waypoints')
def handle_save_waypoints(data):
    global signs_data
    waypoints_m = data['waypoints']
    stops_m = data['stops']
    signs_data = data['signs']

    batches = []
    if not stops_m:
        batches.append(waypoints_m)
    else:
        stop_wp_indices = []
        for sx, sy in stops_m:
            min_d = float('inf')
            best_i = -1
            for i, (wx, wy) in enumerate(waypoints_m):
                d = math.hypot(sx - wx, sy - wy)
                if d < min_d:
                    min_d = d
                    best_i = i
            if best_i != -1 and min_d < 3.0:
                stop_wp_indices.append(best_i)

        stop_wp_indices = sorted(list(set(stop_wp_indices)))
        start_i = 0
        for idx in stop_wp_indices:
            batches.append(waypoints_m[start_i:idx + 1])
            start_i = idx + 1
        if start_i < len(waypoints_m):
            batches.append(waypoints_m[start_i:])

    batches = [b for b in batches if len(b) > 0]

    ros_node.path_active = False
    ros_node.load_batches_and_publish_stops(batches, stops_m)

    socketio.emit('triggers_updated', signs_data)
    print(f"\n[SYSTEM] Queued {len(waypoints_m)} WPs in {len(batches)} batches! Type 'start' to execute.")


@socketio.on('save_presets')
def handle_save_presets(data):
    try:
        with open(PATH_FILE, 'w') as f:
            json.dump({'waypoints': data['waypoints']}, f, indent=4)
        with open(ENV_FILE, 'w') as f:
            json.dump({'stops': data['stops'], 'signs': data['signs']}, f, indent=4)
        print("\n[SYSTEM] Layout successfully saved to local files.")
    except Exception as e:
        print(f"\n[ERROR] Failed to save layout files: {e}")


@socketio.on('load_path')
def handle_load_path():
    try:
        if os.path.exists(PATH_FILE):
            with open(PATH_FILE, 'r') as f:
                file_data = json.load(f)
                socketio.emit('load_data', {'waypoints': file_data.get('waypoints', [])})
                print("\n[SYSTEM] Path successfully loaded from local file.")
        else:
            print(f"\n[SYSTEM] Path file {PATH_FILE} not found.")
    except Exception as e:
        print(f"\n[ERROR] Failed to load path file: {e}")


@socketio.on('load_environment')
def handle_load_environment():
    try:
        if os.path.exists(ENV_FILE):
            with open(ENV_FILE, 'r') as f:
                file_data = json.load(f)
                socketio.emit('load_data', {'stops': file_data.get('stops', []), 'signs': file_data.get('signs', [])})
                print("\n[SYSTEM] Environment successfully loaded from local file.")
        else:
            print(f"\n[SYSTEM] Environment file {ENV_FILE} not found.")
    except Exception as e:
        print(f"\n[ERROR] Failed to load environment file: {e}")


# --- Terminal Loop ---
def terminal_command_loop():
    global signs_data
    print("\n" + "=" * 50)
    print("🤖 ROBOT CONTROL SYSTEM RUNNING")
    print("TERMINAL CONTROLS:")
    print(" -> Type 'restart' to CANCEL Nav2 goal & reset.")
    print(" -> Type 'start' to send the first batch of waypoints.")
    print("=" * 50 + "\n")

    while True:
        try:
            cmd = input().strip().lower()
            if cmd == 'restart':
                ros_node.path_active = False
                ros_node.cancel_nav2_goal()
                ros_node.current_batch_idx = 0
                for s in signs_data: s['triggered'] = False
                socketio.emit('triggers_updated', signs_data)
                print("\n🛑 ROBOT HALTED. Reset to Batch 1. Waiting for 'start'...")
            elif cmd == 'start':
                if ros_node.current_batch_idx < len(ros_node.waypoint_batches):
                    ros_node.path_active = True
                    print(f"\n🚀 EXECUTING BATCH {ros_node.current_batch_idx + 1}/{len(ros_node.waypoint_batches)}!")
                    ros_node.send_next_batch()
                else:
                    print("\n⚠️ Path complete or empty! Plan a route first.")
        except (EOFError, KeyboardInterrupt):
            break


# --- The HTML/JS/CSS Frontend ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Path Planner</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        body { font-family: Arial, sans-serif; background: #f0f0f0; margin: 0; padding: 20px; display: flex; flex-direction: column; align-items: center; }
        #toolbar { background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); margin-bottom: 10px; display: flex; gap: 15px; align-items: center; flex-wrap: wrap; justify-content: center; }
        button { padding: 8px 15px; border: none; border-radius: 4px; color: white; font-weight: bold; cursor: pointer; transition: background 0.2s; }
        .btn-clear { background: #ff4d4d; } .btn-save { background: #4CAF50; }
        .btn-file-load { background: #2196F3; } .btn-file-save { background: #9c27b0; }
        .btn-editor { background: #7f8c8d; }
        .btn-editor.active { background: #e67e22; }
        #canvas-container { width: 90vw; height: 75vh; border: 2px solid #ccc; background: white; cursor: crosshair; overflow: hidden; position: relative; border-radius: 8px; box-shadow: 0 4px 10px rgba(0,0,0,0.15);}
        canvas { display: block; }
        .instructions { margin-top: 10px; color: #555; text-align: center; }
        .instructions span { display: inline-block; margin: 0 10px; }
    </style>
</head>
<body>
    <div id="toolbar">
        <label><b>Select Sign:</b></label>
        <select id="sign-select">
            {% for sign in signs %} <option value="{{ sign }}">{{ sign }}</option> {% endfor %}
        </select>
        <button id="btn-editor" class="btn-editor" onclick="toggleEditor()">✏️ Editor Mode: OFF</button>
        <button class="btn-file-load" onclick="loadPath()">🗺️ Load Path</button>
        <button class="btn-file-load" onclick="loadEnvironment()">🛑 Load Env</button>
        <button class="btn-file-save" onclick="savePresets()">💾 Save Files</button>
        <button class="btn-clear" onclick="clearData()">🗑️ Clear Track</button>
        <button class="btn-save" onclick="queuePath()">🚀 Queue Path</button>
    </div>

    <div id="canvas-container">
        <canvas id="mapCanvas"></canvas>
    </div>

    <div class="instructions">
        <span>🖱️ <b>Left Click:</b> Waypoint</span> | 
        <span><b>Shift+Click:</b> Stop Marker</span> | 
        <span><b>Ctrl+Click:</b> Sign</span><br>
        <span>🔍 <b>Scroll:</b> Zoom</span> | 
        <span>✋ <b>Middle Mouse:</b> Pan</span> |
        <span>✏️ <b>Editor Mode ON + Click:</b> Delete Item</span>
    </div>

    <script>
        const socket = io();
        const canvas = document.getElementById('mapCanvas');
        const ctx = canvas.getContext('2d');
        const container = document.getElementById('canvas-container');
        const signSelect = document.getElementById('sign-select');
        const btnEditor = document.getElementById('btn-editor');

        const res = {{ meta.res }};
        const ox = {{ meta.origin_x }};
        const oy = {{ meta.origin_y }};
        const mw = {{ meta.w }};
        const mh = {{ meta.h }};

        let mapImg = new Image();
        mapImg.src = '/map.png';

        let waypoints = []; let stops = []; let signs = [];
        let robot = {x: 0, y: 0};

        let scale = 1.0; let panX = 0; let panY = 0;
        let isDragging = false; let startPanX, startPanY;
        let isEditorMode = false;

        function resizeCanvas() {
            canvas.width = container.clientWidth;
            canvas.height = container.clientHeight;
            draw();
        }
        window.addEventListener('resize', resizeCanvas);

        mapImg.onload = () => {
            canvas.width = container.clientWidth;
            canvas.height = container.clientHeight;

            scale = Math.min(canvas.width / mw, canvas.height / mh) * 0.9;
            panX = (canvas.width - (mw * scale)) / 2;
            panY = (canvas.height - (mh * scale)) / 2;

            draw();
        };

        function toggleEditor() {
            isEditorMode = !isEditorMode;
            if(isEditorMode) {
                btnEditor.innerHTML = "✏️ Editor Mode: ON";
                btnEditor.classList.add("active");
                container.style.cursor = "pointer";
            } else {
                btnEditor.innerHTML = "✏️ Editor Mode: OFF";
                btnEditor.classList.remove("active");
                container.style.cursor = "crosshair";
            }
        }

        function m_to_px(xm, ym) {
            let px = (xm - ox) / res;
            let py = mh - ((ym - oy) / res);
            return { x: (px * scale) + panX, y: (py * scale) + panY };
        }
        function px_to_m(cx, cy) {
            let px = (cx - panX) / scale;
            let py = (cy - panY) / scale;
            let xm = ox + (px * res);
            let ym = oy + ((mh - py) * res);
            return [xm, ym];
        }

        canvas.addEventListener('mousedown', (e) => {
            const rect = canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;

            if (e.button === 1) { 
                isDragging = true;
                startPanX = mouseX - panX;
                startPanY = mouseY - panY;
            } else if (e.button === 0) { 
                if (isEditorMode) {
                    let deleted = false;

                    // 1. Check Signs First (Radius based on scale)
                    let signR_px = (0.375 / res) * scale;
                    for (let i = signs.length - 1; i >= 0; i--) {
                        let p = m_to_px(signs[i].x_m, signs[i].y_m);
                        if (Math.hypot(p.x - mouseX, p.y - mouseY) <= signR_px) {
                            signs.splice(i, 1);
                            deleted = true;
                            break;
                        }
                    }

                    // 2. Check Stops
                    if (!deleted) {
                        for (let i = stops.length - 1; i >= 0; i--) {
                            let p = m_to_px(stops[i][0], stops[i][1]);
                            if (Math.hypot(p.x - mouseX, p.y - mouseY) <= 12) { // 10px radius + padding
                                stops.splice(i, 1);
                                deleted = true;
                                break;
                            }
                        }
                    }

                    // 3. Check Waypoints
                    if (!deleted) {
                        for (let i = waypoints.length - 1; i >= 0; i--) {
                            let p = m_to_px(waypoints[i][0], waypoints[i][1]);
                            if (Math.hypot(p.x - mouseX, p.y - mouseY) <= 10) { // 6px radius + padding
                                waypoints.splice(i, 1);
                                deleted = true;
                                break;
                            }
                        }
                    }

                    if(deleted) draw();
                    return; // Prevent adding elements in editor mode
                }

                // Add mode
                let [xm, ym] = px_to_m(mouseX, mouseY);
                if (e.shiftKey) { stops.push([xm, ym]); }
                else if (e.ctrlKey) { signs.push({x_m: xm, y_m: ym, type: signSelect.value, triggered: false}); }
                else { waypoints.push([xm, ym]); }
                draw();
            }
        });

        canvas.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const rect = canvas.getBoundingClientRect();
                panX = (e.clientX - rect.left) - startPanX;
                panY = (e.clientY - rect.top) - startPanY;
                draw();
            }
        });

        canvas.addEventListener('mouseup', () => isDragging = false);
        canvas.addEventListener('mouseleave', () => isDragging = false);

        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const rect = canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;

            let zoom = e.deltaY < 0 ? 1.1 : 0.9;
            scale *= zoom;

            panX = mouseX - (mouseX - panX) * zoom;
            panY = mouseY - (mouseY - panY) * zoom;
            draw();
        });

        // --- Sockets Data Handling ---
        socket.on('robot_pose', (data) => {
            robot.x = data.x; robot.y = data.y;
            draw();
        });

        socket.on('triggers_updated', (serverSigns) => {
            signs = serverSigns;
            draw();
        });

        socket.on('load_data', (data) => {
            if (data.waypoints !== undefined) waypoints = data.waypoints;
            if (data.stops !== undefined) stops = data.stops;
            if (data.signs !== undefined) signs = data.signs;
            draw();
        });

        function clearData() { waypoints = []; stops = []; signs = []; draw(); }

        function queuePath() {
            socket.emit('save_waypoints', {waypoints: waypoints, stops: stops, signs: signs});
            alert("Path queued! Type 'start' in the terminal to execute.");
        }

        function savePresets() {
            socket.emit('save_presets', {waypoints: waypoints, stops: stops, signs: signs});
            alert("Path and layout saved to system files!");
        }

        function loadPath() { socket.emit('load_path'); }
        function loadEnvironment() { socket.emit('load_environment'); }

        function draw() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(mapImg, panX, panY, mw * scale, mh * scale);

            ctx.beginPath();
            ctx.strokeStyle = '#007bff'; ctx.lineWidth = 2; ctx.setLineDash([5, 5]);
            for(let i=0; i<waypoints.length; i++) {
                let p = m_to_px(waypoints[i][0], waypoints[i][1]);
                if(i===0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
            }
            ctx.stroke(); ctx.setLineDash([]);

            for(let i=0; i<waypoints.length; i++) {
                let p = m_to_px(waypoints[i][0], waypoints[i][1]);
                ctx.beginPath(); ctx.arc(p.x, p.y, 6, 0, Math.PI*2);
                ctx.fillStyle = '#007bff'; ctx.fill(); ctx.stroke();
                ctx.fillStyle = '#004085'; ctx.font = '12px Arial';
                ctx.fillText('WP ' + (i+1), p.x - 15, p.y - 10);
            }

            for(let s of stops) {
                let p = m_to_px(s[0], s[1]);
                ctx.beginPath(); ctx.arc(p.x, p.y, 10, 0, Math.PI*2);
                ctx.fillStyle = '#ffca28'; ctx.fill(); ctx.strokeStyle = '#f57f17'; ctx.stroke();
            }

            let R = (0.375 / res) * scale;
            for(let s of signs) {
                let p = m_to_px(s.x_m, s.y_m);
                ctx.beginPath(); ctx.arc(p.x, p.y, R, 0, Math.PI*2);
                ctx.fillStyle = s.triggered ? 'rgba(209, 196, 233, 0.5)' : 'rgba(187, 222, 251, 0.5)';
                ctx.strokeStyle = s.triggered ? '#5e35b1' : '#1976d2'; 
                ctx.fill(); ctx.stroke();
                ctx.fillStyle = '#000'; ctx.font = '12px Arial'; ctx.fillText(s.type, p.x-10, p.y);
            }

            let rP = m_to_px(robot.x, robot.y);
            ctx.beginPath(); ctx.arc(rP.x, rP.y, 8, 0, Math.PI*2);
            ctx.fillStyle = 'red'; ctx.fill(); ctx.strokeStyle = 'darkred'; ctx.stroke();
        }
    </script>
</body>
</html>
"""


def main():
    global ros_node
    rclpy.init()

    load_hardcoded_map()
    ros_node = RobotPlannerNode()

    threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()
    threading.Thread(target=terminal_command_loop, daemon=True).start()

    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
