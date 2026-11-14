from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import serial


@dataclass(frozen=True)
class EncoderSample:
    counts: tuple[int, int, int, int]
    measured_mps: tuple[float, float, float, float]
    monotonic_time: float
    sequence: int


class QuadMdProtocol:
    """Private hardware protocol implementation; public ROS API stays vendor-neutral."""

    def __init__(
        self,
        device: str,
        baudrate: int,
        command_order: Sequence[int],
        command_signs: Sequence[int],
        feedback_order: Sequence[int],
        feedback_signs: Sequence[int],
    ) -> None:
        self.command_order = self._order(command_order)
        self.command_signs = self._signs(command_signs)
        self.feedback_order = self._order(feedback_order)
        self.feedback_signs = self._signs(feedback_signs)
        self.port = serial.Serial(
            device,
            baudrate=baudrate,
            timeout=0.05,
            write_timeout=0.5,
        )
        self.write_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.closed = False
        self.latest_counts: Optional[tuple[int, int, int, int]] = None
        self.latest_speeds: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
        self.latest_sample: Optional[EncoderSample] = None
        self.latest_battery: Optional[float] = None
        self.sequence = 0

        time.sleep(0.25)
        self.port.reset_input_buffer()
        self.write('$upload:0,0,0#')
        self.write('$pwm:0,0,0,0#')
        self.reader = threading.Thread(target=self._reader, daemon=True)
        self.reader.start()
        self.write('$upload:1,0,1#')

    @staticmethod
    def _signs(values: Sequence[int]) -> tuple[int, int, int, int]:
        result = tuple(int(v) for v in values)
        if len(result) != 4 or any(v not in (-1, 1) for v in result):
            raise ValueError('sign arrays must contain four values, each -1 or +1')
        return result  # type: ignore[return-value]

    @staticmethod
    def _order(values: Sequence[int]) -> tuple[int, int, int, int]:
        result = tuple(int(v) for v in values)
        if len(result) != 4 or sorted(result) != [0, 1, 2, 3]:
            raise ValueError('order arrays must be a permutation of 0, 1, 2, 3')
        return result  # type: ignore[return-value]

    def write(self, command: str) -> None:
        if not command.startswith('$') or not command.endswith('#'):
            raise ValueError('invalid controller command framing')
        with self.write_lock:
            self.port.write(command.encode('ascii'))
            self.port.flush()

    def command_speed(self, wheel_mps: Sequence[float]) -> tuple[int, int, int, int]:
        values = tuple(float(v) for v in wheel_mps)
        if len(values) != 4 or not all(math.isfinite(v) for v in values):
            raise ValueError('four finite wheel speeds are required')
        board = tuple(
            max(-1000, min(1000, int(round(values[index] * 1000.0)) * sign))
            for index, sign in zip(self.command_order, self.command_signs)
        )
        self.write('$spd:' + ','.join(str(v) for v in board) + '#')
        return board  # type: ignore[return-value]

    def hold_stop(self) -> None:
        self.write('$spd:0,0,0,0#')

    def release(self) -> None:
        self.write('$pwm:0,0,0,0#')

    def request_battery(self) -> None:
        self.write('$read_vol#')

    def sample(self) -> Optional[EncoderSample]:
        with self.state_lock:
            return self.latest_sample

    def battery(self) -> Optional[float]:
        with self.state_lock:
            return self.latest_battery

    def _reader(self) -> None:
        buffer = b''
        while not self.stop_event.is_set():
            try:
                chunk = self.port.read(256)
            except serial.SerialException:
                return
            if not chunk:
                continue
            buffer += chunk
            while b'#' in buffer:
                raw, buffer = buffer.split(b'#', 1)
                frame = raw.decode('ascii', errors='ignore').strip()
                if frame:
                    self._frame(frame)

    def _frame(self, frame: str) -> None:
        now = time.monotonic()
        if frame.startswith('$MAll:'):
            fields = frame[6:].split(',')
            if len(fields) != 4:
                return
            try:
                raw = tuple(int(v) for v in fields)
            except ValueError:
                return
            counts = tuple(
                raw[index] * sign
                for index, sign in zip(self.feedback_order, self.feedback_signs)
            )
            with self.state_lock:
                self.latest_counts = counts  # type: ignore[assignment]
                self.sequence += 1
                self.latest_sample = EncoderSample(
                    counts=counts,  # type: ignore[arg-type]
                    measured_mps=self.latest_speeds,
                    monotonic_time=now,
                    sequence=self.sequence,
                )
            return

        if frame.startswith('$MSPD:'):
            fields = frame[6:].split(',')
            if len(fields) != 4:
                return
            try:
                raw = tuple(float(v) for v in fields)
            except ValueError:
                return
            speeds = tuple(
                raw[index] * sign / 1000.0
                for index, sign in zip(self.feedback_order, self.feedback_signs)
            )
            with self.state_lock:
                self.latest_speeds = speeds  # type: ignore[assignment]
            return

        if frame.startswith('$Battery:'):
            text = frame[len('$Battery:'):].strip().rstrip('Vv')
            try:
                value = float(text)
            except ValueError:
                return
            with self.state_lock:
                self.latest_battery = value

    def close(self) -> None:
        if self.closed:
            return
        try:
            self.write('$upload:0,0,0#')
            self.release()
        except Exception:
            pass
        self.stop_event.set()
        if self.reader.is_alive():
            self.reader.join(timeout=1.0)
        if self.port.is_open:
            self.port.close()
        self.closed = True
