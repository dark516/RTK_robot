from __future__ import annotations

import json
from typing import Any


def decode_json_object(payload: str | bytes) -> dict[str, Any]:
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    value = json.loads(payload)
    if not isinstance(value, dict):
        raise ValueError("Expected a JSON object")
    return value


def validate_command(data: dict[str, Any], expected_robot_id: str) -> None:
    for key in ("message_id", "robot_id", "text"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"Missing or invalid {key}")
    if data["robot_id"] != expected_robot_id:
        raise ValueError("robot_id does not match bridge configuration")


def _best_text(data: dict[str, Any], raw_payload: str) -> str:
    for key in ("text", "message", "reply", "answer"):
        value = data.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    event = data.get("event")
    if isinstance(event, str) and event.strip():
        return event.strip()
    if data:
        return json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    return raw_payload.strip()


def normalize_agent_payload(
    payload: str,
    *,
    expected_robot_id: str,
    active_message_id: str | None,
    answer: bool,
) -> dict[str, str]:
    """Normalize new protocol envelopes and legacy agent output.

    New `rover_agent_mcp` versions already publish a complete JSON envelope.
    For backward compatibility, this function also accepts:

    * a plain-text `/agent/answer`;
    * an old JSON status such as `{"event":"thinking", ...}`;
    * JSON missing `message_id` and/or `robot_id` while one command is active.

    Legacy output can only be correlated safely when the bridge has exactly one
    active command, which is why the bridge serializes dispatch to the agent.
    """

    raw = str(payload or "").strip()
    if not raw:
        raise ValueError("ROS payload is empty")

    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        parsed = None

    data: dict[str, Any]
    if isinstance(parsed, dict):
        data = parsed
    else:
        data = {"text": raw}

    supplied_robot_id = data.get("robot_id")
    if supplied_robot_id not in (None, "", expected_robot_id):
        raise ValueError("robot_id does not match bridge configuration")

    message_id = data.get("message_id")
    if not isinstance(message_id, str) or not message_id.strip():
        message_id = active_message_id
    if not isinstance(message_id, str) or not message_id.strip():
        raise ValueError("Cannot correlate ROS payload: no active message_id")

    text = _best_text(data, raw)
    if answer:
        status = data.get("status")
        if status not in ("completed", "error"):
            event = str(data.get("event") or "").lower()
            lowered = text.lower()
            status = (
                "error"
                if event in {"error", "busy", "failed"}
                or lowered.startswith("ошибка")
                else "completed"
            )
    else:
        status = data.get("status")
        if not isinstance(status, str) or not status.strip():
            event = str(data.get("event") or "").lower()
            status = "error" if event in {"error", "busy", "failed"} else "running"

    return {
        "message_id": message_id.strip(),
        "robot_id": expected_robot_id,
        "status": str(status),
        "text": text,
    }


def validate_outgoing(
    data: dict[str, Any],
    expected_robot_id: str,
    *,
    answer: bool,
) -> None:
    if data.get("robot_id") != expected_robot_id:
        raise ValueError("robot_id does not match bridge configuration")
    required = ("message_id", "robot_id", "status", "text")
    for key in required:
        if not isinstance(data.get(key), str):
            raise ValueError(f"Missing or invalid {key}")
    if answer and data["status"] not in {"completed", "error"}:
        raise ValueError("Answer status must be completed or error")
