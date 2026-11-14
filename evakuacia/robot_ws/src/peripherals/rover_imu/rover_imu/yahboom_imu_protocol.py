"""Parsers for Yahboom/YB-MRA02 IMU serial protocols.

The official Yahboom Raspberry Pi example uses fixed 11-byte frames:
  0x55, frame_type, 8 payload bytes, checksum

Supported and safely skipped frame types in this driver:
  0x50 time
  0x51 acceleration
  0x52 angular velocity
  0x53 Euler orientation
  0x54 magnetic field (diagnostic/optional)
  0x55 port status
  0x56 pressure/altitude
  0x57 GPS longitude/latitude
  0x58 GPS ground speed
  0x59 quaternion
  0x5A satellite positioning accuracy
  0x5F read register response

The checksum is the low byte of the sum of the first 10 bytes.

New YB-MRA02-V1.0 modules use variable-length frames:
  0x7E, 0x23, frame_length, function, payload bytes, checksum
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Iterable, Optional


FRAME_LENGTH = 11
FRAME_HEADER = 0x55
TYPE_ACCELERATION = 0x51
TYPE_GYROSCOPE = 0x52
TYPE_EULER = 0x53
TYPE_MAGNETIC = 0x54
TYPE_YB_MRA02_RAW = 0x04
TYPE_YB_MRA02_QUATERNION = 0x16
TYPE_YB_MRA02_EULER = 0x26
TYPE_YB_MRA02_BARO = 0x34
TYPE_YB_MRA02_VERSION = 0x07
TYPE_YB_MRA02_RETURN_STATE = 0x32
SUPPORTED_TYPES = {
    0x50,
    TYPE_ACCELERATION,
    TYPE_GYROSCOPE,
    TYPE_EULER,
    TYPE_MAGNETIC,
    0x55,
    0x56,
    0x57,
    0x58,
    0x59,
    0x5A,
    0x5F,
}
YB_MRA02_FRAME_HEAD1 = 0x7E
YB_MRA02_FRAME_HEAD2 = 0x23
YB_MRA02_MAX_FRAME_LENGTH = 64
YB_MRA02_SUPPORTED_TYPES = {
    TYPE_YB_MRA02_RAW,
    TYPE_YB_MRA02_QUATERNION,
    TYPE_YB_MRA02_EULER,
    TYPE_YB_MRA02_BARO,
    TYPE_YB_MRA02_VERSION,
    TYPE_YB_MRA02_RETURN_STATE,
}
STANDARD_GRAVITY = 9.80665


@dataclass(frozen=True)
class ParsedFrame:
    frame_type: int
    payload: bytes
    raw: bytes
    protocol: str = 'wit_0x55'


def int16_le(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def validate_frame(frame: bytes) -> bool:
    if len(frame) != FRAME_LENGTH:
        return False
    if frame[0] != FRAME_HEADER:
        return False
    if frame[1] not in SUPPORTED_TYPES:
        return False
    return (sum(frame[:10]) & 0xFF) == frame[10]


def validate_yb_mra02_frame(frame: bytes) -> bool:
    if len(frame) < 5:
        return False
    if frame[0] != YB_MRA02_FRAME_HEAD1 or frame[1] != YB_MRA02_FRAME_HEAD2:
        return False
    if len(frame) != frame[2] or frame[2] > YB_MRA02_MAX_FRAME_LENGTH:
        return False
    if frame[3] not in YB_MRA02_SUPPORTED_TYPES:
        return False
    return (sum(frame[:-1]) & 0xFF) == frame[-1]


class YahboomFrameParser:
    """Incremental parser for old 0x55 and new YB-MRA02 7E23 frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.valid_frames = 0
        self.invalid_frames = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes | bytearray) -> list[ParsedFrame]:
        if data:
            self._buffer.extend(data)

        frames: list[ParsedFrame] = []
        while True:
            header_candidates = [
                index
                for index in (
                    self._find_byte(FRAME_HEADER),
                    self._find_byte(YB_MRA02_FRAME_HEAD1),
                )
                if index >= 0
            ]
            if not header_candidates:
                self.discarded_bytes += len(self._buffer)
                self._buffer.clear()
                break
            header_index = min(header_candidates)

            if header_index:
                self.discarded_bytes += header_index
                del self._buffer[:header_index]

            if not self._buffer:
                break

            if self._buffer[0] == FRAME_HEADER:
                if len(self._buffer) < FRAME_LENGTH:
                    break

                candidate = bytes(self._buffer[:FRAME_LENGTH])
                if validate_frame(candidate):
                    frames.append(
                        ParsedFrame(
                            frame_type=candidate[1],
                            payload=candidate[2:10],
                            raw=candidate,
                        )
                    )
                    self.valid_frames += 1
                    del self._buffer[:FRAME_LENGTH]
                else:
                    self.invalid_frames += 1
                    self.discarded_bytes += 1
                    del self._buffer[0]
                continue

            if self._buffer[0] == YB_MRA02_FRAME_HEAD1:
                if len(self._buffer) < 2:
                    break
                if self._buffer[1] != YB_MRA02_FRAME_HEAD2:
                    self.invalid_frames += 1
                    self.discarded_bytes += 1
                    del self._buffer[0]
                    continue
                if len(self._buffer) < 4:
                    break

                frame_length = int(self._buffer[2])
                if frame_length < 5 or frame_length > YB_MRA02_MAX_FRAME_LENGTH:
                    self.invalid_frames += 1
                    self.discarded_bytes += 1
                    del self._buffer[0]
                    continue
                if len(self._buffer) < frame_length:
                    break

                candidate = bytes(self._buffer[:frame_length])
                if validate_yb_mra02_frame(candidate):
                    frames.append(
                        ParsedFrame(
                            frame_type=candidate[3],
                            payload=candidate[4:-1],
                            raw=candidate,
                            protocol='yb_mra02_v1',
                        )
                    )
                    self.valid_frames += 1
                    del self._buffer[:frame_length]
                else:
                    self.invalid_frames += 1
                    self.discarded_bytes += 1
                    del self._buffer[0]
                continue

        return frames

    def _find_byte(self, value: int) -> int:
        try:
            return self._buffer.index(value)
        except ValueError:
            return -1


def decode_acceleration(payload: bytes) -> tuple[float, float, float]:
    """Return acceleration in m/s^2."""
    scale = 16.0 * STANDARD_GRAVITY / 32768.0
    return tuple(int16_le(payload, i) * scale for i in (0, 2, 4))  # type: ignore[return-value]


def decode_gyroscope(payload: bytes) -> tuple[float, float, float]:
    """Return angular velocity in rad/s."""
    scale = 2000.0 * math.pi / 180.0 / 32768.0
    return tuple(int16_le(payload, i) * scale for i in (0, 2, 4))  # type: ignore[return-value]


def decode_euler(payload: bytes) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in radians."""
    scale = math.pi / 32768.0
    return tuple(int16_le(payload, i) * scale for i in (0, 2, 4))  # type: ignore[return-value]


def decode_magnetic_raw(payload: bytes) -> tuple[int, int, int]:
    """Return raw signed magnetometer channels."""
    return tuple(int16_le(payload, i) for i in (0, 2, 4))  # type: ignore[return-value]


def decode_yb_mra02_raw(
    payload: bytes,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    """Return acceleration (m/s^2), gyro (rad/s) and magnetic field (tesla)."""
    if len(payload) < 18:
        raise ValueError('YB-MRA02 raw IMU frame must contain at least 18 bytes')
    accel_scale = 16.0 * STANDARD_GRAVITY / 32767.0
    gyro_scale = 2000.0 * math.pi / 180.0 / 32767.0
    magnetic_scale_tesla = 800.0e-6 / 32767.0
    acceleration = tuple(int16_le(payload, i) * accel_scale for i in (0, 2, 4))
    gyro = tuple(int16_le(payload, i) * gyro_scale for i in (6, 8, 10))
    magnetic = tuple(
        int16_le(payload, i) * magnetic_scale_tesla for i in (12, 14, 16)
    )
    return acceleration, gyro, magnetic  # type: ignore[return-value]


def decode_yb_mra02_euler(payload: bytes) -> tuple[float, float, float]:
    """Return roll, pitch and yaw in radians from the YB-MRA02 float frame."""
    if len(payload) < 12:
        raise ValueError('YB-MRA02 Euler frame must contain at least 12 bytes')
    return struct.unpack_from('<fff', payload, 0)


def decode_yb_mra02_quaternion(payload: bytes) -> tuple[float, float, float, float]:
    """Return ROS quaternion x, y, z, w from the YB-MRA02 q0, q1, q2, q3 frame."""
    if len(payload) < 16:
        raise ValueError('YB-MRA02 quaternion frame must contain at least 16 bytes')
    qw, qx, qy, qz = struct.unpack_from('<ffff', payload, 0)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def quaternion_from_euler(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """ROS quaternion x, y, z, w from intrinsic roll/pitch/yaw."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def remap_vector(
    vector: tuple[float, float, float],
    axis_map: Iterable[int],
    axis_signs: Iterable[int],
) -> tuple[float, float, float]:
    mapping = tuple(int(value) for value in axis_map)
    signs = tuple(int(value) for value in axis_signs)
    if sorted(mapping) != [0, 1, 2]:
        raise ValueError("axis_map must be a permutation of [0, 1, 2]")
    if len(signs) != 3 or any(sign not in (-1, 1) for sign in signs):
        raise ValueError("axis_signs must contain three values, each +1 or -1")
    return tuple(vector[mapping[i]] * signs[i] for i in range(3))  # type: ignore[return-value]


def count_valid_frames(data: bytes) -> tuple[int, set[int]]:
    parser = YahboomFrameParser()
    types: set[int] = set()
    for frame in parser.feed(data):
        types.add(frame.frame_type)
    return parser.valid_frames, types
