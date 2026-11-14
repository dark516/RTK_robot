"""Local VLM diagnostic engine and HTTPS dashboard.

The engine deliberately keeps the camera transport, VLM request and dashboard
state in one small module so it can be driven either by a ROS node or by unit
tests without starting ROS.
"""

from __future__ import annotations

import base64
from collections import Counter, deque
import json
from pathlib import Path
import queue
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np


EVENT_TYPES = [
    "traffic_accident",
    "fuel_truck",
    "fallen_tree",
    "building_debris",
]
# Keep the neutral outcome first. Some constrained-generation backends have a
# small first-enum bias; putting ``present`` first caused false QR positives on
# an entirely blank sign face.
QR_STATES = ("absent", "present", "unknown")


def _clean_types(values, fallback):
    if isinstance(values, str):
        values = values.replace("\n", ",").split(",")
    result = []
    for value in values or []:
        value = str(value).strip().lower().replace(" ", "_")
        if value and value not in result:
            result.append(value)
    return result or list(fallback)


def make_schema(event_types=None):
    events = _clean_types(event_types, EVENT_TYPES)
    # none/unknown are control outcomes and must always remain available.
    events = [item for item in events if item not in {"none", "unknown"}]
    events += [item for item in ("none", "unknown") if item not in events]
    return {
        "type": "object",
        "properties": {
            "event_type": {"type": "string", "enum": events},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            # QR contents are deliberately not part of the contract: the
            # organisers may replace the code before a run. We only classify
            # the currently visible face of a sign as having a QR or not.
            "qr": {"type": "string", "enum": list(QR_STATES)},
        },
        "required": ["event_type", "confidence", "qr"],
        "additionalProperties": False,
    }


def make_event_schema(event_types=None):
    combined = make_schema(event_types)
    return {
        "type": "object",
        "properties": {
            key: combined["properties"][key]
            for key in ("event_type", "confidence")
        },
        "required": ["event_type", "confidence"],
        "additionalProperties": False,
    }


def make_qr_schema():
    return {
        "type": "object",
        "properties": {"qr": {"type": "string", "enum": list(QR_STATES)}},
        "required": ["qr"],
        "additionalProperties": False,
    }


def make_prompt(event_types):
    event_values = list(event_types)
    for control_value in ("none", "unknown"):
        if control_value not in event_values:
            event_values.append(control_value)
    events = ", ".join(event_values)
    return (
        "You classify one emergency event for a rescue robot. Inspect only the "
        "current camera frame. Return exactly one event_type, never a list.\n"
        f"Allowed event_type values: {events}.\n"
        "Event definitions: traffic_accident means three or more cars gathered "
        "on the road or a collision scene; do not require visible damage. In "
        "this tabletop field, several toy cars across the driving corridor count "
        "as traffic_accident even when the cars are intact. Do not output none "
        "just because the cars are small or undamaged. "
        "fuel_truck means the orange/yellow tanker body, cab or tanker wheels, "
        "including an overturned tanker; fallen_tree means a large tree lying "
        "across the road; building_debris means a collapsed wall/building or "
        "its rubble.\n"
        "Choose the single nearest, largest foreground emergency object that is "
        "close enough to block the robot's current path. Ignore distant objects "
        "that do not occupy the immediate foreground, people, intact scenery, "
        "road markings and background buildings. If every configured event is "
        "far away or absent, return none (no nearby obstacle). If an orange or "
        "yellow tanker or its overturned undercarriage is visible in the "
        "immediate foreground, always choose fuel_truck. If a collapsed "
        "building and its rubble are visible nearby, choose building_debris. Use "
        "unknown only when the frame is too blurry, dark or ambiguous. The image "
        "may show a scale model."
    )


def make_qr_prompt():
    return (
        "Inspect this image only. Look at the rectangular face of the small white "
        "physical sign, if it is visible. Set qr=present only if you actually see "
        "a dense square black-and-white QR matrix with large square finder markers "
        "in its corners. Set qr=absent when that face is plain white, when there is "
        "no sign, or when no QR pattern is visible. Never infer anything printed "
        "on the hidden reverse side. Do not decode or identify the QR contents; "
        "they change between runs. Use unknown only if image quality prevents any "
        "visual assessment. Return JSON only."
    )


def _response_content(response):
    try:
        content = response["choices"][0]["message"]["content"]
        if isinstance(content, list):
            content = "".join(item.get("text", "") for item in content)
        content = str(content).strip()
        if content.startswith("```"):
            content = content.strip("`").removeprefix("json").strip()
        return json.loads(content)
    except (KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError("malformed VLM response") from exc


def parse_vlm_response(response, schema):
    result = _response_content(response)

    allowed_events = set(schema["properties"]["event_type"]["enum"])
    event_type = result.get("event_type")
    if event_type not in allowed_events:
        raise ValueError(f"unsupported event_type: {event_type}")
    confidence = result.get("confidence")
    if not isinstance(confidence, (int, float)) or not 0 <= confidence <= 1:
        raise ValueError("confidence must be between 0 and 1")
    qr = result.get("qr")
    if qr not in QR_STATES:
        raise ValueError(f"unsupported qr state: {qr}")
    return result


def parse_event_response(response, schema):
    result = _response_content(response)
    event_type = result.get("event_type")
    if event_type not in set(schema["properties"]["event_type"]["enum"]):
        raise ValueError(f"unsupported event_type: {event_type}")
    confidence = result.get("confidence")
    if not isinstance(confidence, (int, float)) or not 0 <= confidence <= 1:
        raise ValueError("confidence must be between 0 and 1")
    return result


def parse_qr_response(response):
    result = _response_content(response)
    qr = result.get("qr")
    if qr not in QR_STATES:
        raise ValueError(f"unsupported qr state: {qr}")
    return qr


def focus_near_field(jpeg):
    """Keep the road-facing lower part of a fisheye frame for VLM attention."""
    array = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if array is None or array.shape[0] < 10:
        return jpeg
    height, width = array.shape[:2]
    crop = array[int(height * 0.27) :, :]
    if crop.shape[1] > 1536:
        scale = 1536.0 / crop.shape[1]
        crop = cv2.resize(crop, (1536, max(1, int(crop.shape[0] * scale))), interpolation=cv2.INTER_AREA)
    encoded, output = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return output.tobytes() if encoded else jpeg


def _jpeg_from_multipart(buffer):
    start = buffer.find(b"\xff\xd8")
    if start < 0:
        return buffer[-1:]
    end = buffer.find(b"\xff\xd9", start + 2)
    if end < 0:
        return buffer[start:]
    return buffer[end + 2 :], buffer[start : end + 2]


class MjpegReader:
    """Read complete JPEGs from a multipart MJPEG stream."""

    def __init__(self, url, timeout):
        self.url = url
        self.timeout = timeout
        self.response = None
        self.buffer = b""

    def close(self):
        if self.response is not None:
            try:
                self.response.close()
            except Exception:
                pass
        self.response = None
        self.buffer = b""

    def read_one(self):
        if self.response is None:
            self.response = urllib.request.urlopen(self.url, timeout=self.timeout)
        while True:
            parsed = _jpeg_from_multipart(self.buffer)
            if isinstance(parsed, tuple):
                self.buffer, jpeg = parsed
                return jpeg
            self.buffer = parsed
            chunk = self.response.read(65536)
            if not chunk:
                raise urllib.error.URLError("camera stream ended")
            self.buffer += chunk


class VlmDiagnosticEngine:
    def __init__(
        self,
        robot_url,
        vlm_server_url,
        mode="service",
        capture_interval=0.5,
        camera_timeout=2.0,
        event_types=None,
        result_callback=None,
    ):
        self.lock = threading.RLock()
        self.robot_url = str(robot_url)
        self.vlm_server_url = str(vlm_server_url)
        self.mode = mode if mode in {"continuous", "service"} else "service"
        self.capture_interval = max(0.05, float(capture_interval))
        self.camera_timeout = max(0.2, float(camera_timeout))
        self.event_types = _clean_types(event_types, EVENT_TYPES)
        self.latest_frame = None
        self.latest_frame_at = 0.0
        self.frame_sequence = 0
        self.analysis_sequence = 0
        self.analysis_busy = False
        self.analysis_started_at = 0.0
        self.analysis_frame_sequence = None
        self.analysis_stage = "idle"
        self.status = "starting"
        self.last_error = None
        self.last_record = None
        self.history = deque(maxlen=3)
        self.dropped_frames = 0
        self.result_callback = result_callback
        self._stop = threading.Event()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name="vlm-camera", daemon=True
        )
        self._capture_thread.start()

    @property
    def schema(self):
        return make_schema(self.event_types)

    def state(self):
        with self.lock:
            record = self.last_record
            return {
                "mode": self.mode,
                "status": self.status,
                "robot_url": self.robot_url,
                "vlm_server_url": self.vlm_server_url,
                "frame_sequence": self.frame_sequence,
                "analysis_sequence": self.analysis_sequence,
                "analysis_busy": self.analysis_busy,
                "analysis_frame_sequence": self.analysis_frame_sequence,
                "analysis_elapsed_sec": round(
                    (time.monotonic() - self.analysis_started_at)
                    if self.analysis_busy
                    else (record.get("latency_sec", 0.0) if record else 0.0),
                    1,
                ),
                "analysis_stage": self.analysis_stage,
                "analysis_progress": None,
                "latest_frame_at": self.latest_frame_at,
                "dropped_frames": self.dropped_frames,
                "last_error": self.last_error,
                "last_record": record,
                "confirmed_accident": self._confirmed_accident(),
                "event_types": list(self.event_types),
                "schema": self.schema,
            }

    def set_mode(self, mode):
        if mode not in {"continuous", "service"}:
            raise ValueError("mode must be continuous or service")
        with self.lock:
            self.mode = mode
            self.status = "waiting" if mode == "service" else "watching"
            self.last_error = None

    def set_config(self, robot_url=None, event_types=None):
        with self.lock:
            if robot_url:
                self.robot_url = str(robot_url).strip()
            if event_types is not None:
                self.event_types = _clean_types(event_types, EVENT_TYPES)
            self.history.clear()

    def stop(self):
        self._stop.set()
        self._capture_thread.join(timeout=2)

    def _fetch_one(self, reader):
        if "/stream" in self.robot_url:
            return reader.read_one()
        with urllib.request.urlopen(self.robot_url, timeout=self.camera_timeout) as response:
            return response.read(5 * 1024 * 1024)

    def _capture_loop(self):
        reader = None
        next_auto = 0.0
        while not self._stop.is_set():
            try:
                with self.lock:
                    url = self.robot_url
                if "/stream" in url:
                    if reader is None or reader.url != url:
                        if reader is not None:
                            reader.close()
                        reader = MjpegReader(url, self.camera_timeout)
                else:
                    reader = None
                jpeg = self._fetch_one(reader)
                if not jpeg.startswith(b"\xff\xd8"):
                    raise ValueError("camera response is not a JPEG")
                with self.lock:
                    self.latest_frame = jpeg
                    self.latest_frame_at = time.time()
                    self.frame_sequence += 1
                    mode = self.mode
                    busy = self.analysis_busy
                    if not busy:
                        self.status = "watching" if mode == "continuous" else "waiting"
                if mode == "continuous":
                    now = time.monotonic()
                    if now >= next_auto:
                        if busy:
                            with self.lock:
                                self.dropped_frames += 1
                        else:
                            self.request_analysis(jpeg)
                        next_auto = now + self.capture_interval
                if "/stream" not in url:
                    time.sleep(self.capture_interval)
            except Exception as exc:
                if reader is not None:
                    reader.close()
                reader = None
                with self.lock:
                    self.status = "camera_error"
                    self.last_error = str(exc)
                self._stop.wait(1.0)

    def request_analysis(self, jpeg=None, completion=None):
        with self.lock:
            if self.analysis_busy:
                return False
            jpeg = jpeg or self.latest_frame
            if jpeg is None:
                self.status = "waiting_for_frame"
                return False
            self.analysis_busy = True
            self.status = "analyzing"
            self.last_error = None
            sequence = self.frame_sequence
            self.analysis_started_at = time.monotonic()
            self.analysis_frame_sequence = sequence
            self.analysis_stage = "preparing"
        threading.Thread(
            target=self._analyze_worker,
            args=(jpeg, sequence, completion),
            name="vlm-inference",
            daemon=True,
        ).start()
        return True

    def request_analysis_sync(self, jpeg=None, timeout=125.0):
        """Run one analysis and wait for its JSON result for a ROS service call."""
        done = threading.Event()
        holder = {}
        if not self.request_analysis(jpeg, completion=(done, holder)):
            return False, "busy or camera frame unavailable"
        if not done.wait(timeout):
            return False, f"VLM timeout after {timeout:.0f}s"
        if holder.get("record") is not None:
            return True, json.dumps(holder["record"], ensure_ascii=False)
        return False, holder.get("error", "VLM inference failed")

    def _analyze_worker(self, jpeg, frame_sequence, completion=None):
        started = time.monotonic()
        try:
            event_schema = make_event_schema(self.event_types)
            qr_schema = make_qr_schema()
            model_jpeg = focus_near_field(jpeg)
            image_url = "data:image/jpeg;base64," + base64.b64encode(model_jpeg).decode("ascii")
            payload = {
                "model": "local-vlm",
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": make_prompt(self.event_types),
                            },
                            {
                                "type": "image_url",
                                "image_url": {"url": image_url},
                            },
                        ],
                    }
                ],
                "temperature": 0,
                "seed": 42,
                "max_tokens": 180,
                "response_format": {"type": "json_schema", "json_schema": {"schema": event_schema}},
            }
            request = urllib.request.Request(
                self.vlm_server_url,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with self.lock:
                self.analysis_stage = "inference"
            with urllib.request.urlopen(request, timeout=120) as response:
                with self.lock:
                    self.analysis_stage = "parsing"
                parsed = parse_event_response(json.load(response), event_schema)

            qr_payload = {
                "model": "local-vlm",
                "messages": [{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": make_qr_prompt()},
                        {"type": "image_url", "image_url": {"url": image_url}},
                    ],
                }],
                "temperature": 0,
                "seed": 42,
                "max_tokens": 30,
                "response_format": {"type": "json_schema", "json_schema": {"schema": qr_schema}},
            }
            qr_request = urllib.request.Request(
                self.vlm_server_url,
                data=json.dumps(qr_payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with self.lock:
                self.analysis_stage = "qr_inference"
            with urllib.request.urlopen(qr_request, timeout=120) as response:
                parsed["qr"] = parse_qr_response(json.load(response))
            parsed["qr_source"] = "vlm"
            record = {
                "timestamp": time.time(),
                "frame_sequence": frame_sequence,
                "latency_sec": round(time.monotonic() - started, 2),
                "observation": parsed,
            }
            with self.lock:
                self.last_record = record
                self.analysis_sequence += 1
                self.history.append(parsed)
                self.status = "ready"
                self.last_error = None
        except Exception as exc:
            with self.lock:
                self.status = "vlm_error"
                self.last_error = str(exc)
            record = None
        finally:
            with self.lock:
                self.analysis_busy = False
                self.analysis_stage = "idle"
            if completion is not None:
                done, holder = completion
                if record is not None:
                    holder["record"] = record
                else:
                    holder["error"] = self.last_error or "VLM inference failed"
                done.set()
            if record is not None and self.result_callback is not None:
                self.result_callback(record)

    def _confirmed_accident(self):
        values = [
            item.get("event_type")
            for item in list(self.history)[-3:]
            if item.get("event_type") not in {"none", "unknown"}
        ]
        if not values:
            return None
        value, count = Counter(values).most_common(1)[0]
        return value if count >= 2 else None

    def current_frame(self):
        with self.lock:
            return self.latest_frame


HTML = r"""<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VLM / field diagnostics</title>
<style>
:root{--ink:#172129;--paper:#f1efe8;--panel:#fbfaf6;--line:#c9c5b9;--accent:#ee6338;--teal:#1c7a73;--muted:#69736f;--mono:"IBM Plex Mono","Cascadia Code",monospace}
*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font-family:Georgia,"Times New Roman",serif;min-height:100vh}
body:before{content:"";position:fixed;inset:0;pointer-events:none;opacity:.22;background-image:linear-gradient(115deg,transparent 0 49%,#d8d4c8 50% 50.3%,transparent 50.5%),radial-gradient(#8e8a7e 0.55px,transparent .6px);background-size:100% 100%,9px 9px}
.shell{position:relative;max-width:1440px;margin:auto;padding:26px 30px 40px}.mast{display:flex;justify-content:space-between;gap:24px;border-bottom:1px solid var(--ink);padding-bottom:16px;align-items:end}.eyebrow{font:11px var(--mono);letter-spacing:.15em;text-transform:uppercase;color:var(--accent)}h1{font-size:clamp(34px,5vw,70px);line-height:.9;margin:8px 0 0;font-weight:500;letter-spacing:-.06em}.stamp{font:11px var(--mono);border:1px solid var(--ink);padding:9px 11px;white-space:nowrap}
.grid{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(330px,.75fr);gap:20px;margin-top:20px}.card{background:rgba(251,250,246,.92);border:1px solid var(--line);box-shadow:7px 7px 0 rgba(23,33,41,.08)}.view{padding:12px}.screen{position:relative;aspect-ratio:16/10;background:#101a1d;overflow:hidden;border:1px solid var(--ink)}#frame{width:100%;height:100%;object-fit:contain;display:block}.screen:after{content:"LIVE FEED";position:absolute;top:13px;left:14px;background:var(--accent);color:#fff;padding:5px 8px;font:10px var(--mono);letter-spacing:.12em}.metricbar{display:grid;grid-template-columns:repeat(4,1fr);border-top:1px solid var(--ink);margin-top:12px}.metric{padding:11px 9px;border-right:1px solid var(--line)}.metric:last-child{border:0}.metric b{display:block;font:22px var(--mono);letter-spacing:-.08em}.metric span{font:10px var(--mono);color:var(--muted);text-transform:uppercase}
.side{display:flex;flex-direction:column;gap:14px}.section{padding:17px}.section h2{font-size:18px;margin:0 0 13px;font-weight:500}.label{display:block;font:10px var(--mono);text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin:12px 0 6px}input,textarea{width:100%;font:12px var(--mono);color:var(--ink);background:#f5f3eb;border:1px solid var(--line);padding:10px 11px;outline:0}input:focus,textarea:focus{border-color:var(--accent)}textarea{min-height:130px;resize:vertical}.row{display:flex;gap:8px}.btn{border:1px solid var(--ink);background:var(--ink);color:#fff;font:11px var(--mono);padding:10px 13px;cursor:pointer;text-transform:uppercase;letter-spacing:.04em}.btn:hover{background:var(--accent);border-color:var(--accent)}.btn.secondary{background:transparent;color:var(--ink)}.btn.active{background:var(--teal);border-color:var(--teal)}.toggle{display:grid;grid-template-columns:1fr 1fr;gap:4px;background:#dfddd4;padding:4px}.toggle .btn{border:0;background:transparent;color:var(--muted)}.toggle .btn.active{color:#fff}.status{font:12px var(--mono);padding:10px;border-left:4px solid var(--teal);background:#e7f0ed}.status.bad{border-color:var(--accent);background:#f9e7df}.status.wait{border-color:#bc9141;background:#f4eedb}.progress-card{border:1px solid var(--line);background:#f5f3eb;padding:11px 12px;margin-top:12px}.progress-head{display:flex;justify-content:space-between;gap:12px;font:11px var(--mono);text-transform:uppercase}.progress-time{font-size:16px;color:var(--accent)}.progress-track{height:7px;background:#d8d5ca;overflow:hidden;margin:9px 0 7px}.progress-bar{height:100%;width:0;background:var(--teal)}.progress-bar.running{width:42%;background:var(--accent);animation:indeterminate 1.25s ease-in-out infinite}@keyframes indeterminate{0%{transform:translateX(-120%)}100%{transform:translateX(260%)}}.result{font:12px var(--mono);white-space:pre-wrap;max-height:280px;overflow:auto;background:#172129;color:#e9e4d6;padding:14px}.hint{font:11px var(--mono);color:var(--muted);line-height:1.45}.foot{margin-top:22px;font:10px var(--mono);color:var(--muted);display:flex;justify-content:space-between;gap:15px}
@media(max-width:900px){.grid{grid-template-columns:1fr}.shell{padding:18px 14px}.metricbar{grid-template-columns:repeat(2,1fr)}.metric:nth-child(2){border-right:0}.metric:nth-child(-n+2){border-bottom:1px solid var(--line)}}
</style></head><body><main class="shell">
<header class="mast"><div><div class="eyebrow">Rescue robotics / visual language model</div><h1>Field<br>diagnostics.</h1></div><div class="stamp">VLM NODE · HTTPS :8081</div></header>
<div class="grid"><section class="card view"><div class="screen"><img id="frame" src="/frame.jpg" alt="Кадр камеры робота"></div><div class="metricbar"><div class="metric"><b id="state">starting</b><span>node state</span></div><div class="metric"><b id="frameNo">—</b><span>frame no.</span></div><div class="metric"><b id="latency">—</b><span>last VLM</span></div><div class="metric"><b id="confirmed">—</b><span>confirmed</span></div></div><div class="progress-card"><div class="progress-head"><span id="progressLabel">Waiting for analysis</span><span class="progress-time" id="progressTime">0.0s</span></div><div class="progress-track"><div class="progress-bar" id="progressBar"></div></div><div class="hint" id="progressHint">Exact token percentage is not exposed by the current VLM HTTP endpoint.</div></div></section>
<aside class="side"><section class="card section"><h2>Analysis mode</h2><div class="toggle"><button class="btn active" id="continuous">Every frame</button><button class="btn" id="service">On service</button></div><button class="btn" id="analyze" style="width:100%;margin-top:10px">Analyze current frame</button><p class="hint">Every frame uses the newest available frame and waits for the previous VLM request. On service sends only when you press the button or call <code>/vlm/analyze_frame</code>.</p></section>
<section class="card section"><h2>Camera source</h2><label class="label" for="cameraUrl">MJPEG stream URL</label><input id="cameraUrl" value="http://192.168.1.70:8091/stream?mode=live"><button class="btn secondary" id="connect" style="margin-top:9px">Apply camera URL</button><p class="hint">The browser receives the frame through this HTTPS page, so the robot URL does not need CORS.</p></section>
<section class="card section"><h2>Event schema</h2><label class="label" for="events">One event per label, comma separated</label><input id="events" value="traffic_accident, fuel_truck, fallen_tree, building_debris"><p class="hint">The model returns exactly one nearby event. Far-away or absent events produce <code>none</code>. Add your own labels here; <code>none</code> and <code>unknown</code> are added automatically.</p><button class="btn" id="saveSchema" style="margin-top:11px">Update JSON schema</button><details style="margin-top:13px"><summary class="label" style="cursor:pointer">Generated schema</summary><pre class="result" id="schema" style="max-height:190px;margin-top:8px"></pre></details></section>
<section class="card section"><h2>Last observation</h2><div class="status wait" id="status">Connecting to camera…</div><pre class="result" id="result">No VLM response yet.</pre></section></aside></div>
<footer class="foot"><span id="source">source: —</span><span>local inference · JSON constrained output</span></footer></main>
<script>
const $=id=>document.getElementById(id);let pollTimer;function comma(v){return v.split(',').map(x=>x.trim()).filter(Boolean)}
async function api(url,opts={}){const r=await fetch(url,{...opts,headers:{'Content-Type':'application/json',...(opts.headers||{})}});if(!r.ok)throw Error(await r.text());return r.json()}
function render(s){$('state').textContent=s.status;$('frameNo').textContent=s.frame_sequence||'—';$('confirmed').textContent=s.confirmed_accident||'—';$('latency').textContent=s.last_record?s.last_record.latency_sec+'s':'—';$('source').textContent='source: '+s.robot_url;$('result').textContent=s.last_record?JSON.stringify(s.last_record.observation,null,2):'No VLM response yet.';$('schema').textContent=JSON.stringify(s.schema,null,2);const elapsed=Number(s.analysis_elapsed_sec||0);$('progressTime').textContent=elapsed.toFixed(1)+'s';$('progressLabel').textContent=s.analysis_busy?'Processing · '+s.analysis_stage:(s.last_record?'Completed · frame '+s.last_record.frame_sequence:'Waiting for analysis');$('progressBar').classList.toggle('running',Boolean(s.analysis_busy));$('progressHint').textContent=s.analysis_busy?'Live elapsed time; exact token percentage is not exposed by the current VLM HTTP endpoint.':'Last completed inference: '+(s.last_record?s.last_record.latency_sec+'s':'—');const el=$('status');el.textContent=s.last_error||((s.analysis_busy?'VLM is analyzing frame '+(s.analysis_frame_sequence||''):'Camera connected · '+s.mode));el.className='status '+(s.last_error?'bad':s.analysis_busy?'wait':'');['continuous','service'].forEach(id=>$(id).classList.toggle('active',s.mode===(id==='continuous'?'continuous':'service')))}
async function poll(){try{const s=await api('/api/state');render(s);$('frame').src='/frame.jpg?seq='+s.frame_sequence}catch(e){$('status').textContent=e;$('status').className='status bad'}pollTimer=setTimeout(poll,700)}
async function setMode(mode){await api('/api/mode',{method:'POST',body:JSON.stringify({mode})});poll()}$('continuous').onclick=()=>setMode('continuous');$('service').onclick=()=>setMode('service');$('analyze').onclick=async()=>{try{await api('/api/analyze',{method:'POST',body:'{}'});poll()}catch(e){alert(e)}};$('connect').onclick=async()=>{try{await api('/api/config',{method:'POST',body:JSON.stringify({robot_url:$('cameraUrl').value.trim()})});poll()}catch(e){alert(e)}};$('saveSchema').onclick=async()=>{try{await api('/api/schema',{method:'POST',body:JSON.stringify({event_types:comma($('events').value)})});poll()}catch(e){alert(e)}};poll();
</script></body></html>"""


class DashboardHandler:
    def __init__(self, engine):
        self.engine = engine

    def __call__(self, *args, **kwargs):
        from http.server import BaseHTTPRequestHandler

        engine = self.engine

        class Handler(BaseHTTPRequestHandler):
            def _send(self, status, content_type, body):
                if isinstance(body, str):
                    body = body.encode("utf-8")
                try:
                    self.send_response(status)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                except OSError:
                    # Browsers routinely cancel the previous polling request
                    # when they refresh the next frame over HTTPS.
                    pass

            def do_GET(self):
                if self.path.split("?", 1)[0] == "/":
                    self._send(200, "text/html; charset=utf-8", HTML)
                elif self.path.split("?", 1)[0] == "/api/state":
                    self._send(200, "application/json", json.dumps(engine.state(), ensure_ascii=False))
                elif self.path.split("?", 1)[0] == "/frame.jpg":
                    frame = engine.current_frame()
                    if frame is None:
                        self._send(503, "text/plain; charset=utf-8", "camera frame is not available")
                    else:
                        self._send(200, "image/jpeg", frame)
                else:
                    self._send(404, "text/plain; charset=utf-8", "not found")

            def do_POST(self):
                length = int(self.headers.get("Content-Length", "0"))
                try:
                    body = json.loads(self.rfile.read(length) or b"{}")
                    route = self.path.split("?", 1)[0]
                    if route == "/api/mode":
                        engine.set_mode(body.get("mode"))
                        result = {"ok": True}
                    elif route == "/api/analyze":
                        result = {"accepted": engine.request_analysis()}
                    elif route == "/api/config":
                        engine.set_config(robot_url=body.get("robot_url"))
                        result = {"ok": True}
                    elif route == "/api/schema":
                        engine.set_config(event_types=body.get("event_types"))
                        result = {"ok": True, "schema": engine.schema}
                    else:
                        self._send(404, "text/plain; charset=utf-8", "not found")
                        return
                    self._send(200, "application/json", json.dumps(result, ensure_ascii=False))
                except Exception as exc:
                    self._send(400, "application/json", json.dumps({"error": str(exc)}))

            def log_message(self, *_args):
                return

        return Handler(*args, **kwargs)
