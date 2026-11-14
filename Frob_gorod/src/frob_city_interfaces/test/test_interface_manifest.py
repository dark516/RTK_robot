from pathlib import Path
import hashlib
import json
import re

PACKAGE = Path(__file__).resolve().parents[1]

EXPECTED_FILES = {
    "msg": {
        "SignDetection.msg", "SignDetectionArray.msg", "ApproachState.msg",
        "TargetState.msg", "EdgeState.msg", "CoverageState.msg", "MapState.msg",
        "KnownSignState.msg",
        "RouteSegment.msg", "MotionPermit.msg", "GuardState.msg", "MissionState.msg",
    },
    "srv": {
        "PlanRoute.srv", "ValidateSegment.srv", "ReportExecution.srv",
        "ResetMap.srv", "ControlMission.srv",
    },
    "action": {"Observe.action"},
}

EXPECTED_CONSTANTS = {
    "msg/SignDetection.msg": {
        "UNKNOWN": 0, "STRAIGHT_ONLY": 1, "LEFT_ONLY": 2, "RIGHT_ONLY": 3,
        "NO_LEFT": 4, "NO_RIGHT": 5, "BUS_STOP": 6, "PARKING": 7,
        "HAZARD_MARKER": 8, "RANGE_UNKNOWN": 0, "OPTICAL_Z": 1, "RAY_RANGE": 2,
    },
    "msg/ApproachState.msg": {
        "UNKNOWN": 0, "OBSERVING": 1, "RESOLVED_CLEAR": 2, "RESOLVED_SIGNED": 3,
        "CONFLICT": 4, "BLOCKED": 5, "MOVE_STRAIGHT": 1, "MOVE_LEFT": 2, "MOVE_RIGHT": 4,
    },
    "msg/TargetState.msg": {"PROVISIONAL": 0, "CONFIRMED": 1, "CONFLICT": 2, "BUS_STOP": 6, "PARKING": 7},
    "msg/EdgeState.msg": {"OPEN": 0, "TEMP_BLOCKED": 1, "HAZARD_BLOCKED": 2},
    "msg/CoverageState.msg": {"UNKNOWN": 0, "COMPLETE": 1, "INCONCLUSIVE": 2, "UNREACHABLE_NOW": 3},
    "msg/RouteSegment.msg": {
        "CORRIDOR": 0, "JUNCTION": 1, "SERVICE_APPROACH": 2, "OBSERVATION_ADJUST": 3,
        "HOLD": 0, "OBSERVE": 1, "BUS_SERVICE": 2, "PARK": 3,
        "MOVE_STRAIGHT": 1, "MOVE_LEFT": 2, "MOVE_RIGHT": 4,
    },
    "msg/MissionState.msg": {
        "PHASE_IDLE": 0, "PHASE_BOARDING": 1, "PHASE_PARKING_PRIOR": 2,
        "PHASE_PARKING_SEARCH": 3, "PHASE_PARKING_APPROACH": 4,
        "PHASE_DONE": 5, "PHASE_PAUSED": 6, "PHASE_ABORTED": 7,
        "EXEC_HOLDING": 0, "EXEC_REQUESTING_PLAN": 1, "EXEC_VALIDATING_SEGMENT": 2,
        "EXEC_SENDING_GOAL": 3, "EXEC_MOVING": 4, "EXEC_CANCELING": 5,
        "EXEC_OBSERVING": 6, "EXEC_VERIFYING_STOP": 7, "EXEC_DWELLING": 8,
        "PARKING_PRIOR_NOT_CHECKED": 0, "PARKING_PRIOR_FOUND": 1,
        "PARKING_PRIOR_CLEAR_ABSENT": 2, "PARKING_PRIOR_INCONCLUSIVE": 3,
        "PARKING_PRIOR_UNREACHABLE_NOW": 4, "PARKING_PRIOR_INVALID": 5,
    },
    "srv/PlanRoute.srv": {
        "TARGET": 0, "EXPLORE_STOPS": 1, "EXPLORE_PARKING": 2, "CHECK_POI": 3,
        "ROUTE_READY": 0, "NEED_OBSERVATION": 1, "ALREADY_AT_TERMINAL": 2,
        "NO_ROUTE": 3, "NO_REACHABLE_FRONTIER": 4, "INVALID_REQUEST": 5,
        "NOT_READY": 6, "STALE_SESSION": 7,
    },
    "srv/ValidateSegment.srv": {"VALID": 0, "INVALID": 1, "NEED_OBSERVATION": 2, "NOT_READY": 3, "STALE_SESSION": 4},
    "srv/ReportExecution.srv": {"COMPLETED": 0, "FAILED": 1, "CANCELED": 2},
    "srv/ControlMission.srv": {"START": 0, "PAUSE": 1, "RESUME": 2, "ABORT": 3, "RESET": 4},
    "action/Observe.action": {
        "APPROACH_RESOLVED": 0, "TARGET_FOUND": 1, "CLEAR_ABSENT": 2,
        "INCONCLUSIVE": 3, "CONFLICT": 4, "NOT_IN_ZONE": 5, "STALE_SESSION": 6,
    },
}

EXPECTED_FIELDS = {
    "msg/MapState.msg": [
        "std_msgs/Header header", "string run_id", "string map_instance_id", "string field_id",
        "uint64 revision", "bool ready", "bool perception_healthy", "bool localization_healthy",
        "string[] precautionary_hold_ids", "frob_city_interfaces/ApproachState[] approaches",
        "frob_city_interfaces/KnownSignState[] known_signs",
        "frob_city_interfaces/TargetState[] targets", "frob_city_interfaces/EdgeState[] edges",
        "frob_city_interfaces/CoverageState[] coverage", "string reason",
    ],
    "msg/RouteSegment.msg": [
        "string segment_id", "uint8 kind", "string from_anchor_id", "string to_anchor_id",
        "string approach_id", "uint8 selected_movement", "nav_msgs/Path path",
        "geometry_msgs/PolygonStamped[] allowed_regions", "string[] dependency_ids",
        "uint64[] dependency_versions", "string[] traversed_edge_ids", "uint8 terminal_kind",
        "string terminal_id", "geometry_msgs/PoseStamped terminal_limit", "bool commitment_gate_valid",
        "geometry_msgs/PoseStamped commitment_gate", "float32 max_speed_mps",
        "float32 max_yaw_rate_radps", "float32 xy_tolerance_m", "float32 yaw_tolerance_rad",
        "float32 estimated_duration_s",
    ],
}

CONST_RE = re.compile(r"^\s*[A-Za-z0-9_/\[\]]+\s+([A-Z][A-Z0-9_]*)=(-?\d+)\s*$")


def _meaningful_lines(path: Path):
    out = []
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            out.append(line)
    return out


def _constants(path: Path):
    constants = {}
    for line in _meaningful_lines(path):
        match = CONST_RE.match(line)
        if match:
            constants[match.group(1)] = int(match.group(2))
    return constants


def _fields(path: Path):
    return [
        line for line in _meaningful_lines(path)
        if line != "---" and not CONST_RE.match(line)
    ]


def test_complete_interface_file_set():
    for kind, expected in EXPECTED_FILES.items():
        ext = {"msg": "msg", "srv": "srv", "action": "action"}[kind]
        actual = {p.name for p in (PACKAGE / kind).glob(f"*.{ext}")}
        # glob suffix for msg/srv/action already matches the directory name.
        assert actual == expected, f"{kind}: expected {sorted(expected)}, got {sorted(actual)}"


def test_numeric_constants_are_frozen():
    for relpath, expected in EXPECTED_CONSTANTS.items():
        actual = _constants(PACKAGE / relpath)
        for name, value in expected.items():
            assert actual.get(name) == value, f"{relpath}: {name} expected {value}, got {actual.get(name)}"


def test_high_risk_wire_fields_are_exact():
    for relpath, expected in EXPECTED_FIELDS.items():
        assert _fields(PACKAGE / relpath) == expected


def test_no_json_or_std_msgs_string_payload_shortcut():
    for kind in ("msg", "srv", "action"):
        for path in (PACKAGE / kind).glob("*"):
            if path.is_file():
                text = path.read_text().lower()
                assert "json" not in text
                assert "std_msgs/string" not in text


def test_golden_semantic_manifest_has_no_interface_drift():
    manifest_path = PACKAGE / "test" / "interface_manifest.json"
    expected = json.loads(manifest_path.read_text())
    actual = {}
    for kind, names in EXPECTED_FILES.items():
        for name in sorted(names):
            relpath = f"{kind}/{name}"
            normalized = "\n".join(_meaningful_lines(PACKAGE / relpath)) + "\n"
            actual[relpath] = hashlib.sha256(normalized.encode()).hexdigest()
    assert actual == expected
