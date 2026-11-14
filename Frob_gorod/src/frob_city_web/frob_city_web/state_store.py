"""Thread-safe state exchange between ROS callbacks and HTTP clients."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import math
import threading
import time
from typing import Any


class StateStore:
    """Keep bounded dashboard state and change history.

    Camera bytes use a separate condition and are intentionally excluded from
    JSON snapshots. All returned values are copies so HTTP handlers cannot
    mutate ROS-owned state.
    """

    def __init__(
        self,
        *,
        delta_limit: int = 256,
        event_limit: int = 250,
        trail_limit: int = 500,
    ) -> None:
        if min(delta_limit, event_limit, trail_limit) <= 0:
            raise ValueError("state-store limits must be positive")
        self._condition = threading.Condition(threading.RLock())
        self._sections: dict[str, Any] = {}
        self._revision = 0
        self._changes: deque[tuple[int, str, Any]] = deque(maxlen=delta_limit)
        self._events: deque[dict[str, Any]] = deque(maxlen=event_limit)
        self._trail: deque[dict[str, float]] = deque(maxlen=trail_limit)
        self._camera_sequence = 0
        self._camera_stamp_s = 0.0
        self._camera_jpeg = b""

    def update(self, section: str, value: dict[str, Any] | list[Any]) -> int:
        if not section or section in {"events", "trail"}:
            raise ValueError("invalid state section")
        copied = deepcopy(value)
        with self._condition:
            if self._sections.get(section) == copied:
                return self._revision
            self._sections[section] = copied
            self._record_change(section, copied)
            return self._revision

    def snapshot(self) -> dict[str, Any]:
        with self._condition:
            sections = deepcopy(self._sections)
            sections["events"] = deepcopy(list(self._events))
            sections["trail"] = deepcopy(list(self._trail))
            return {"revision": self._revision, "sections": sections}

    def delta_after(self, revision: int, timeout_s: float) -> dict[str, Any] | None:
        revision = int(revision)
        timeout_s = max(0.0, float(timeout_s))
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._revision <= revision:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)

            if revision < 0 or (
                self._changes and revision < self._changes[0][0] - 1
            ):
                snapshot = self.snapshot()
                snapshot["full"] = True
                return snapshot

            sections: dict[str, Any] = {}
            for change_revision, section, value in self._changes:
                if change_revision > revision:
                    sections[section] = deepcopy(value)
            return {
                "revision": self._revision,
                "full": False,
                "sections": sections,
            }

    def append_event(
        self,
        level: str,
        source: str,
        message: str,
        data: dict[str, Any] | None = None,
    ) -> int:
        event = {
            "time_s": time.time(),
            "level": str(level),
            "source": str(source),
            "message": str(message),
            "data": deepcopy(data or {}),
        }
        with self._condition:
            self._events.append(event)
            self._record_change("events", list(self._events))
            return self._revision

    def append_trail(self, x_m: float, y_m: float, stamp_s: float) -> int:
        values = (float(x_m), float(y_m), float(stamp_s))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("trail values must be finite")
        point = {"x_m": values[0], "y_m": values[1], "stamp_s": values[2]}
        with self._condition:
            if self._trail:
                previous = self._trail[-1]
                if math.hypot(
                    previous["x_m"] - point["x_m"],
                    previous["y_m"] - point["y_m"],
                ) < 0.005:
                    return self._revision
            self._trail.append(point)
            self._record_change("trail", list(self._trail))
            return self._revision

    def set_camera(self, jpeg: bytes, stamp_s: float) -> int:
        if not jpeg:
            raise ValueError("camera JPEG must not be empty")
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s) or stamp_s < 0.0:
            raise ValueError("camera timestamp must be finite and nonnegative")
        with self._condition:
            self._camera_sequence += 1
            self._camera_stamp_s = stamp_s
            self._camera_jpeg = bytes(jpeg)
            self._condition.notify_all()
            return self._camera_sequence

    def camera_after(
        self, sequence: int, timeout_s: float
    ) -> tuple[int, float, bytes] | None:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while self._camera_sequence <= int(sequence):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            return (
                self._camera_sequence,
                self._camera_stamp_s,
                self._camera_jpeg,
            )

    def _record_change(self, section: str, value: Any) -> None:
        self._revision += 1
        self._changes.append((self._revision, section, deepcopy(value)))
        self._condition.notify_all()
