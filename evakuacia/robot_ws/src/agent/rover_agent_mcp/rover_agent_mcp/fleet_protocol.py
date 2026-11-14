from __future__ import annotations

from dataclasses import dataclass
import json
import uuid
from typing import Any


@dataclass(frozen=True, slots=True)
class CommandContext:
    """Correlation metadata kept outside the LLM-visible command text."""

    message_id: str
    robot_id: str
    text: str
    source_robot_id: str | None = None
    came_from_envelope: bool = False


def _non_empty_string(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    value = value.strip()
    return value or None


def parse_command_payload(payload: str, configured_robot_id: str) -> CommandContext:
    """Parse a fleet JSON envelope or a legacy plain-text ROS command.

    Fleet envelopes have the form::

        {"message_id": "...", "robot_id": "rover-01", "text": "..."}

    The IDs are retained only for correlation.  Only ``text`` is passed to the
    language model and MCP executor.  For a legacy plain-text command a local
    UUID is generated so that status/answer messages still follow the same
    output protocol.
    """

    raw = str(payload or "").strip()
    if not raw:
        raise ValueError("Command payload is empty")

    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        value = None

    if isinstance(value, dict):
        text = _non_empty_string(value.get("text"))
        if text is not None:
            message_id = _non_empty_string(value.get("message_id")) or str(uuid.uuid4())
            source_robot_id = _non_empty_string(value.get("robot_id"))
            return CommandContext(
                message_id=message_id,
                robot_id=configured_robot_id,
                text=text,
                source_robot_id=source_robot_id,
                came_from_envelope=True,
            )

    return CommandContext(
        message_id=str(uuid.uuid4()),
        robot_id=configured_robot_id,
        text=raw,
        source_robot_id=None,
        came_from_envelope=False,
    )


def make_status_payload(
    context: CommandContext,
    *,
    status: str,
    text: str,
) -> dict[str, str]:
    return {
        "message_id": context.message_id,
        "robot_id": context.robot_id,
        "status": str(status or "running"),
        "text": str(text or ""),
    }


def make_answer_payload(
    context: CommandContext,
    *,
    status: str,
    text: str,
) -> dict[str, str]:
    normalized_status = "completed" if status == "completed" else "error"
    return {
        "message_id": context.message_id,
        "robot_id": context.robot_id,
        "status": normalized_status,
        "text": str(text or ""),
    }
