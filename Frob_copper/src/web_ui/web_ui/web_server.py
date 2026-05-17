#!/usr/bin/env python3
"""ROS2 node that subscribes to /dist and serves a web UI via Flask."""

import threading
from flask import Flask, render_template_string, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


HTML = """
<!DOCTYPE html>
<html>
<head>
<title>Robot UI</title>

<style>
body {
    margin: 0;
    overflow: hidden;
    font-family: Arial, sans-serif;

    background: var(--bg);
    color: var(--text);
}

/* ================= THEMES ================= */

body.theme-terracotta {
    --bg: #e8e1d7;
    --text: #111111;

    --rect-bg: #B86B3D;
    --road-bg: #b5b5b5;

    --rod-gray: #9a9a9a;
    --rod-red: #6d1f14;

    --number-bg: #ffffff;
    --number-text: #000000;
    --number-border: #000000;

    --admin-selected: rgba(255, 255, 0, 0.28);
    --admin-hover: rgba(255, 255, 255, 0.16);

    --passed-green: rgba(80, 255, 80, 0.28);
    --active-green: rgba(80, 255, 80, 0.88);

    --button-bg: #ffffff;
    --button-text: #000000;
}

body.theme-dark {
    --bg: #151515;
    --text: #f2f2f2;

    --rect-bg: #5a3427;
    --road-bg: #4c4c4c;

    --rod-gray: #888888;
    --rod-red: #2b0f0a;

    --number-bg: #eeeeee;
    --number-text: #000000;
    --number-border: #f2f2f2;

    --admin-selected: rgba(255, 220, 70, 0.35);
    --admin-hover: rgba(255, 255, 255, 0.12);

    --passed-green: rgba(80, 255, 120, 0.30);
    --active-green: rgba(80, 255, 120, 0.92);

    --button-bg: #2c2c2c;
    --button-text: #ffffff;
}

body.theme-blue {
    --bg: #dce8f5;
    --text: #0a1b2f;

    --rect-bg: #4c83b6;
    --road-bg: #9aaec2;

    --rod-gray: #d5dce5;
    --rod-red: #1d4268;

    --number-bg: #ffffff;
    --number-text: #0a1b2f;
    --number-border: #0a1b2f;

    --admin-selected: rgba(255, 230, 80, 0.38);
    --admin-hover: rgba(255, 255, 255, 0.22);

    --passed-green: rgba(70, 255, 160, 0.30);
    --active-green: rgba(70, 255, 160, 0.92);

    --button-bg: #ffffff;
    --button-text: #0a1b2f;
}

/* ================= BASE ================= */

.main-container {
    position: relative;
    width: 100vw;
    height: 100vh;
}

/* ================= TITLE ================= */

.title-display {
    position: absolute;
    top: 18px;
    left: 50%;
    transform: translateX(-50%);
    font-size: 34px;
    font-weight: bold;
    z-index: 20;
    color: var(--text);
}

.title-editor {
    position: absolute;
    top: 18px;
    left: 50%;
    transform: translateX(-50%);
    display: none;
    z-index: 30;
}

.title-editor input {
    font-size: 26px;
    padding: 6px 10px;
    width: 420px;
}

/* ================= ADMIN ================= */

.top-controls {
    position: absolute;
    top: 18px;
    right: 20px;
    z-index: 30;

    display: flex;
    align-items: center;
    gap: 10px;
}

button,
select {
    padding: 10px 16px;
    font-size: 16px;
    cursor: pointer;

    background: var(--button-bg);
    color: var(--button-text);

    border: 2px solid var(--number-border);
    border-radius: 6px;
}

.theme-control {
    display: flex;
    align-items: center;
    gap: 6px;
    font-weight: bold;
}

/* ================= LAYOUT ================= */

.column {
    position: absolute;
    top: 50%;
    transform: translateY(-50%);
    display: flex;
    flex-direction: column-reverse;
    gap: 26px;
}

.left-column {
    left: 40px;
}

.right-column {
    right: 40px;
}

.rect-wrapper {
    display: flex;
    align-items: center;
    gap: 14px;
}

/* ================= NUMBER BOXES ================= */

.number-box {
    width: 42px;
    height: 42px;

    background: var(--number-bg);
    color: var(--number-text);

    border: 2px solid var(--number-border);

    display: flex;
    align-items: center;
    justify-content: center;

    font-weight: bold;
    user-select: none;
    cursor: pointer;
}

/* ================= RECTANGLES ================= */

.rect {
    width: 340px;
    height: 84px;

    background: var(--rect-bg);

    border-radius: 12px;
    overflow: hidden;

    display: flex;
    align-items: center;
    justify-content: center;
}

.rod-row {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 100%;
}

/*
   Each rectangle has 12 pairs:
   gray rod + selectable gap + dark red rod
*/

.rod {
    width: 5px;
    height: 74px;
    margin: 0 2px;
    border-radius: 3px;
    pointer-events: none;
}

.rod.gray {
    background: var(--rod-gray);
}

.rod.red {
    background: var(--rod-red);
}

.gap {
    width: 10px;
    height: 84px;
    border-radius: 5px;
    cursor: pointer;
    background: transparent;
}

/* selected gaps are visible ONLY in admin mode */
.admin-mode .gap.selected {
    background: var(--admin-selected);
}

.admin-mode .gap:hover {
    background: var(--admin-hover);
}

.admin-mode .gap.selected:hover {
    background: var(--admin-selected);
}

/* selected + passed by robot */
.gap.passed {
    background: var(--passed-green);
}

/* selected + current active row */
.gap.active {
    background: var(--active-green);
    box-shadow: 0 0 14px rgba(0, 255, 0, 0.85);
}

/* ================= ROBOT LANE ================= */

.robot-lane {
    position: absolute;
    left: 50%;
    top: 80px;
    transform: translateX(-50%);

    width: 90px;
    height: calc(100vh - 160px);

    background: var(--road-bg);

    border-radius: 18px;
    box-shadow: inset 0 0 8px rgba(0,0,0,0.18);
}

.robot {
    position: absolute;

    width: 28px;
    height: 28px;

    background: red;
    border-radius: 50%;

    left: 50%;
    transform: translateX(-50%);

    transition: top 0.25s ease;

    box-shadow:
        0 0 16px rgba(255,0,0,0.9),
        0 0 28px rgba(255,0,0,0.45);
}

/* ================= SLIDER ================= */

.slider-container {
    position: absolute;
    bottom: 25px;
    left: 50%;
    transform: translateX(-50%);

    width: 520px;
    text-align: center;

    color: var(--text);
}

input[type=range] {
    width: 100%;
}
</style>
</head>

<body class="theme-terracotta">

<div class="main-container">

    <div class="title-display" id="titleDisplay">ROBOT SYSTEM</div>

    <div class="title-editor" id="titleEditor">
        <input id="titleInput" value="ROBOT SYSTEM">
    </div>

    <div class="top-controls" id="topControls" style="display:none;">

        <div class="theme-control">
            Theme:
            <select id="themeSelect">
                <option value="terracotta">Terracotta</option>
                <option value="dark">Dark Industrial</option>
                <option value="blue">Blue Tech</option>
            </select>
        </div>

        <button onclick="exitAdmin()">Exit Admin</button>

    </div>

    <div class="column left-column" id="leftColumn"></div>
    <div class="column right-column" id="rightColumn"></div>

    <div class="robot-lane" id="robotLane">
        <div class="robot" id="robot"></div>
    </div>

    <div class="slider-container">
        Robot: <span id="cmValue">0</span> cm
        <input type="range" min="0" max="160" value="0" id="robotSlider">
    </div>

</div>

<script>
let adminMode = false;
let selectedSet = new Set();
let currentRobotCm = 0;
let currentTheme = "terracotta";
let sliderDragging = false;

const root = document.body;
const leftColumn = document.getElementById("leftColumn");
const rightColumn = document.getElementById("rightColumn");
const robot = document.getElementById("robot");
const robotLane = document.getElementById("robotLane");
const slider = document.getElementById("robotSlider");
const cmValue = document.getElementById("cmValue");
const titleDisplay = document.getElementById("titleDisplay");
const titleInput = document.getElementById("titleInput");
const themeSelect = document.getElementById("themeSelect");

/* ================= THEME ================= */

function applyTheme(theme) {
    root.classList.remove("theme-terracotta", "theme-dark", "theme-blue");

    if (theme === "dark") {
        root.classList.add("theme-dark");
    } else if (theme === "blue") {
        root.classList.add("theme-blue");
    } else {
        root.classList.add("theme-terracotta");
    }

    currentTheme = theme;
    themeSelect.value = theme;
}

/* ================= ADMIN ================= */

function setAdmin(state) {
    adminMode = state;

    document.getElementById("topControls").style.display =
        state ? "flex" : "none";

    document.getElementById("titleEditor").style.display =
        state ? "block" : "none";

    root.classList.toggle("admin-mode", state);

    renderSelectionAndLights();
}

function toggleAdmin() {
    setAdmin(!adminMode);
}

function exitAdmin() {
    setAdmin(false);
}

/* ================= TITLE ================= */

titleInput.addEventListener("input", async () => {
    titleDisplay.innerText = titleInput.value;

    await fetch("/set_title", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({
            title: titleInput.value
        })
    });
});

/* ================= THEME CONTROL ================= */

themeSelect.addEventListener("change", async () => {
    const newTheme = themeSelect.value;

    applyTheme(newTheme);

    await fetch("/set_theme", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({
            theme: newTheme
        })
    });
});

/* ================= RECTANGLES ================= */

function createRect(id, side) {
    const wrapper = document.createElement("div");
    wrapper.className = "rect-wrapper";

    const number = document.createElement("div");
    number.className = "number-box";
    number.innerText = id;

    // Admin panel opens by pressing number 7
    number.onclick = () => {
        if (id === 7) {
            toggleAdmin();
        }
    };

    const rect = document.createElement("div");
    rect.className = "rect";
    rect.dataset.rect = id;

    const row = document.createElement("div");
    row.className = "rod-row";

    // 12 pairs: gray rod + selectable gap + dark red rod
    for (let i = 0; i < 12; i++) {
        const grayRod = document.createElement("div");
        grayRod.className = "rod gray";

        const gap = document.createElement("div");
        gap.className = "gap";
        gap.dataset.rect = id;
        gap.dataset.index = i;

        gap.onclick = async () => {
            if (!adminMode) return;

            await fetch("/toggle_gap", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json"
                },
                body: JSON.stringify({
                    rect: id,
                    gap: i
                })
            });

            await updateFromServer();
        };

        const redRod = document.createElement("div");
        redRod.className = "rod red";

        row.appendChild(grayRod);
        row.appendChild(gap);
        row.appendChild(redRod);
    }

    rect.appendChild(row);

    if (side === "left") {
        wrapper.appendChild(number);
        wrapper.appendChild(rect);
    } else {
        wrapper.appendChild(rect);
        wrapper.appendChild(number);
    }

    return wrapper;
}

function buildUI() {
    leftColumn.innerHTML = "";
    rightColumn.innerHTML = "";

    // left side: 1 bottom, 4 top
    for (let i = 1; i <= 4; i++) {
        leftColumn.appendChild(createRect(i, "left"));
    }

    // right side: 5 bottom, 8 top
    for (let i = 5; i <= 8; i++) {
        rightColumn.appendChild(createRect(i, "right"));
    }
}

/* ================= ROBOT POSITION ================= */

/*
   Robot jump logic:
   0-29 cm    -> bottom of gray path
   30-69 cm   -> level of row 1
   70-109 cm  -> level of row 2
   110-149 cm -> level of row 3
   150-160 cm -> level of row 4
*/

function getRobotRow(cm) {
    if (cm < 30) return 0;
    if (cm < 70) return 1;
    if (cm < 110) return 2;
    if (cm < 150) return 3;
    return 4;
}

function getRobotTopForRow(row) {
    const laneRect = robotLane.getBoundingClientRect();
    const robotHeight = robot.offsetHeight || 28;

    // bottom start position
    if (row === 0) {
        return robotLane.clientHeight - robotHeight - 12;
    }

    // align to actual rectangle centers
    const rect = document.querySelector('.rect[data-rect="' + row + '"]');

    if (rect) {
        const rectBox = rect.getBoundingClientRect();
        const rectCenterY = rectBox.top + rectBox.height / 2;
        return rectCenterY - laneRect.top - robotHeight / 2;
    }

    // fallback if layout is not ready
    const fallback = [0, 0.25, 0.5, 0.75, 1.0];
    return robotLane.clientHeight - fallback[row] * robotLane.clientHeight;
}

function updateRobotPosition(cm) {
    const row = getRobotRow(cm);
    let top = getRobotTopForRow(row);

    const minTop = 0;
    const maxTop = robotLane.clientHeight - (robot.offsetHeight || 28);

    top = Math.max(minTop, Math.min(maxTop, top));

    robot.style.top = top + "px";
}

/* ================= LIGHTING LOGIC ================= */

/*
   Light-up thresholds:
   <30 cm      -> no selected gaps light
   30-69 cm    -> row 1 selected gaps light
   70-109 cm   -> row 2 selected gaps light
   110-149 cm  -> row 3 selected gaps light
   >=150 cm    -> row 4 selected gaps light

   Passed rows stay dim green.
   Current row is bright green.
*/

function getLightRow(cm) {
    if (cm < 30) return -1;
    if (cm < 70) return 1;
    if (cm < 110) return 2;
    if (cm < 150) return 3;
    return 4;
}

function getRowForRect(rectId) {
    if (rectId === 1 || rectId === 5) return 1;
    if (rectId === 2 || rectId === 6) return 2;
    if (rectId === 3 || rectId === 7) return 3;
    return 4;
}

function renderSelectionAndLights() {
    const lightRow = getLightRow(currentRobotCm);

    document.querySelectorAll(".gap").forEach(gap => {
        gap.classList.remove("selected", "passed", "active");

        const rectId = parseInt(gap.dataset.rect);
        const gapIndex = parseInt(gap.dataset.index);
        const key = rectId + "_" + gapIndex;

        if (!selectedSet.has(key)) return;

        gap.classList.add("selected");

        const row = getRowForRect(rectId);

        if (lightRow === -1) return;

        if (row < lightRow) {
            gap.classList.add("passed");
        }

        if (row === lightRow) {
            gap.classList.add("active");
        }
    });
}

/* ================= SERVER SYNC ================= */

async function updateFromServer() {
    const response = await fetch("/state");
    const data = await response.json();

    selectedSet = new Set(data.selected);
    currentRobotCm = data.robot_position;

    cmValue.innerText = currentRobotCm;

    // Only update slider from server if the user is NOT dragging it
    if (!sliderDragging) {
        slider.value = currentRobotCm;
    }

    titleDisplay.innerText = data.title;

    if (document.activeElement !== titleInput) {
        titleInput.value = data.title;
    }

    applyTheme(data.theme);

    updateRobotPosition(currentRobotCm);
    renderSelectionAndLights();
}

slider.addEventListener("mousedown", () => {
    sliderDragging = true;
});

slider.addEventListener("touchstart", () => {
    sliderDragging = true;
});

slider.addEventListener("input", async () => {
    const value = parseInt(slider.value);

    currentRobotCm = value;
    cmValue.innerText = value;

    updateRobotPosition(value);
    renderSelectionAndLights();

    await fetch("/set_robot_position", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({
            position: value
        })
    });
});

slider.addEventListener("change", () => {
    sliderDragging = false;
});

slider.addEventListener("mouseup", () => {
    sliderDragging = false;
});

slider.addEventListener("touchend", () => {
    sliderDragging = false;
});

/* ================= INIT ================= */

buildUI();
updateFromServer();

window.addEventListener("resize", () => {
    updateRobotPosition(currentRobotCm);
});

setInterval(updateFromServer, 200);
</script>

</body>
</html>
"""


# Global state shared between ROS2 node and Flask
selected_gaps = set()
robot_position = 0
title_text = "ROBOT SYSTEM"
theme_name = "terracotta"
state_lock = threading.Lock()

app = Flask(__name__)


@app.route("/")
def home():
    return render_template_string(HTML)


@app.route("/state")
def state():
    with state_lock:
        return jsonify({
            "selected": sorted(list(selected_gaps)),
            "robot_position": robot_position,
            "title": title_text,
            "theme": theme_name,
        })


@app.route("/toggle_gap", methods=["POST"])
def toggle_gap():
    data = request.json
    rect = int(data["rect"])
    gap = int(data["gap"])
    key = f"{rect}_{gap}"

    with state_lock:
        if key in selected_gaps:
            selected_gaps.remove(key)
        else:
            selected_gaps.add(key)

    return jsonify(success=True)


@app.route("/set_robot_position", methods=["POST"])
def set_robot_position():
    global robot_position

    data = request.json
    value = int(data["position"])

    with state_lock:
        robot_position = max(0, min(160, value))

    return jsonify(success=True)


@app.route("/set_title", methods=["POST"])
def set_title():
    global title_text

    data = request.json
    with state_lock:
        title_text = str(data.get("title", "ROBOT SYSTEM"))

    return jsonify(success=True)


@app.route("/set_theme", methods=["POST"])
def set_theme():
    global theme_name

    data = request.json
    requested_theme = str(data.get("theme", "terracotta"))

    if requested_theme not in ["terracotta", "dark", "blue"]:
        requested_theme = "terracotta"

    with state_lock:
        theme_name = requested_theme

    return jsonify(success=True)


class WebUINode(Node):
    """ROS2 node: subscribes to /dist and updates shared robot_position."""

    def __init__(self):
        super().__init__('web_ui_node')

        self.declare_parameter('web_port', 5000)
        self.declare_parameter('host', '0.0.0.0')

        self._port = self.get_parameter('web_port').value
        self._host = self.get_parameter('host').value

        self.create_subscription(Float32, '/dist', self._dist_callback, 10)

        self.get_logger().info(
            f'WebUINode started — subscribing to /dist, web server on {self._host}:{self._port}'
        )

    def _dist_callback(self, msg):
        """Update robot position from /dist topic value (in cm)."""
        with state_lock:
            global robot_position
            cm = int(round(float(msg.data)))
            robot_position = max(0, min(160, cm))

    def _run_flask(self):
        app.run(host=self._host, port=self._port, debug=False, use_reloader=False)


def main(args=None):
    rclpy.init(args=args)
    node = WebUINode()

    # Start Flask in a background thread
    flask_thread = threading.Thread(target=node._run_flask, daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
