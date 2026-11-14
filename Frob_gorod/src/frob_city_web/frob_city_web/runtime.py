"""ROS-independent runtime helpers used by the web node."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import queue
import threading
import time
from typing import Any


@dataclass(slots=True)
class PendingControl:
    command: str
    reason: str
    expires_at_s: float
    done: threading.Event = field(default_factory=threading.Event)
    result: Any = None
    expired: bool = False


class ControlMailbox:
    def __init__(self, *, limit: int = 8) -> None:
        if limit <= 0:
            raise ValueError("control mailbox limit must be positive")
        self._queue: queue.Queue[PendingControl] = queue.Queue(maxsize=limit)

    def submit(self, command: str, reason: str, timeout_s: float) -> Any:
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("control timeout must be finite and positive")
        pending = PendingControl(str(command), str(reason), time.monotonic() + timeout_s)
        try:
            self._queue.put_nowait(pending)
        except queue.Full as exc:
            raise RuntimeError("control_queue_full") from exc
        if not pending.done.wait(timeout_s):
            pending.expired = True
            raise TimeoutError("control_timeout")
        return pending.result

    def take(self) -> PendingControl | None:
        while True:
            try:
                pending = self._queue.get_nowait()
            except queue.Empty:
                return None
            if pending.expired or time.monotonic() >= pending.expires_at_s:
                pending.expired = True
                pending.done.set()
                continue
            return pending

    @staticmethod
    def complete(pending: PendingControl, result: Any) -> None:
        if pending.expired:
            return
        pending.result = result
        pending.done.set()


class ImageThrottle:
    def __init__(self, *, max_fps: float, jpeg_quality: int) -> None:
        max_fps = float(max_fps)
        jpeg_quality = int(jpeg_quality)
        if not math.isfinite(max_fps) or max_fps <= 0.0:
            raise ValueError("camera max_fps must be finite and positive")
        if jpeg_quality < 1 or jpeg_quality > 100:
            raise ValueError("JPEG quality must be between 1 and 100")
        self.period_s = 1.0 / max_fps
        self.jpeg_quality = jpeg_quality
        self._last_encoded_s = -math.inf

    def should_encode(self, *, now_s: float, viewers: int) -> bool:
        if viewers <= 0:
            return False
        now_s = float(now_s)
        if now_s + 1e-12 < self._last_encoded_s + self.period_s:
            return False
        self._last_encoded_s = now_s
        return True


def source_health(last_received_s: float | None, *, max_age_s: float, now_s: float | None = None) -> dict[str, Any]:
    max_age_s = float(max_age_s)
    now_s = time.monotonic() if now_s is None else float(now_s)
    if last_received_s is None:
        return {"state": "missing", "age_s": None, "max_age_s": max_age_s}
    age = max(0.0, now_s - float(last_received_s))
    return {
        "state": "fresh" if age <= max_age_s else "stale",
        "age_s": age,
        "max_age_s": max_age_s,
    }
