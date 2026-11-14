from __future__ import annotations

import glob
import json
import os
import subprocess
import tempfile
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

import serial


YAHBOOM_FRAME_HEADER = 0x55
YAHBOOM_FRAME_LENGTH = 11
YAHBOOM_FRAME_TYPES = {
    0x50,  # time
    0x51,  # acceleration
    0x52,  # angular velocity
    0x53,  # Euler angle
    0x54,  # magnetic field
    0x55,  # port status
    0x56,  # pressure/altitude
    0x57,  # GPS longitude/latitude
    0x58,  # GPS ground speed
    0x59,  # quaternion
    0x5A,  # satellite positioning accuracy
    0x5F,  # read register response
}
YAHBOOM_REQUIRED_IMU_TYPES = {0x51, 0x52}
YAHBOOM_USEFUL_IMU_TYPES = {0x51, 0x52, 0x53, 0x54, 0x59}
YB_MRA02_FRAME_HEAD1 = 0x7E
YB_MRA02_FRAME_HEAD2 = 0x23
YB_MRA02_MAX_FRAME_LENGTH = 64
YB_MRA02_FUNC_RAW = 0x04
YB_MRA02_FUNC_QUATERNION = 0x16
YB_MRA02_FUNC_EULER = 0x26
YB_MRA02_FRAME_TYPES = {
    YB_MRA02_FUNC_RAW,
    YB_MRA02_FUNC_QUATERNION,
    YB_MRA02_FUNC_EULER,
    0x07,  # version
    0x32,  # return state
    0x34,  # barometer
}
DEFAULT_IMU_BAUDRATES = (
    115200,
    921600,
    9600,
    230400,
    460800,
    57600,
    38400,
    19200,
    4800,
)
DEFAULT_SLLIDAR_BAUDRATES = (460800, 115200, 256000, 1000000)
SLLIDAR_GET_INFO = b'\xA5\x50'
SLLIDAR_STOP = b'\xA5\x25'
SLLIDAR_ANSWER_SYNC = b'\xA5\x5A'
SLLIDAR_DEVICE_INFO_TYPE = 0x04
SLLIDAR_DEVICE_INFO_LENGTH = 20
DEFAULT_DEVICE_CONFIG = '~/.config/rover/devices.json'
CONFIG_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class DeviceResult:
    role: str
    device: str
    resolved_device: str
    baudrate: int
    confidence: str
    reason: str
    protocol: str = ''
    profile: str = ''
    parameters: dict[str, Any] = field(default_factory=dict)


def serial_candidates(extra: Iterable[str] = ()) -> list[str]:
    """Return one useful path for every physical serial device.

    /dev/serial/by-path is preferred because it follows the physical USB socket
    and therefore does not require unique serial numbers on a production fleet.
    """
    patterns = (
        '/dev/serial/by-path/*',
        '/dev/serial/by-id/*',
        '/dev/ttyUSB*',
        '/dev/ttyACM*',
    )
    raw: list[str] = []
    for pattern in patterns:
        raw.extend(sorted(glob.glob(pattern)))
    raw.extend(extra)

    result: list[str] = []
    seen: set[str] = set()
    for path in raw:
        try:
            resolved = os.path.realpath(path)
        except OSError:
            continue
        if not os.path.exists(resolved) or resolved in seen:
            continue
        seen.add(resolved)
        result.append(path)
    return result


def physical_serial_devices() -> dict[str, str]:
    """Map resolved tty paths to their preferred stable aliases."""
    return {
        os.path.realpath(path): preferred_stable_path(path)
        for path in serial_candidates()
    }


def serial_aliases(device: str) -> list[str]:
    """Return all stable aliases that currently point at a serial device."""
    resolved = os.path.realpath(device)
    aliases: list[str] = []
    for pattern in ('/dev/serial/by-id/*', '/dev/serial/by-path/*'):
        for alias in sorted(glob.glob(pattern)):
            if os.path.realpath(alias) == resolved:
                aliases.append(alias)
    return aliases


def preferred_stable_path(device: str) -> str:
    resolved = os.path.realpath(device)
    for pattern in ('/dev/serial/by-path/*', '/dev/serial/by-id/*'):
        for alias in sorted(glob.glob(pattern)):
            if os.path.realpath(alias) == resolved:
                return alias
    return resolved


def udev_properties(device: str) -> dict[str, str]:
    """Read useful USB metadata without making it part of runtime identity."""
    resolved = os.path.realpath(device)
    try:
        completed = subprocess.run(
            ['udevadm', 'info', '--query=property', f'--name={resolved}'],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return {}

    keep = {
        'ID_VENDOR_ID',
        'ID_MODEL_ID',
        'ID_VENDOR',
        'ID_MODEL',
        'ID_SERIAL',
        'ID_SERIAL_SHORT',
        'ID_PATH',
        'ID_PATH_TAG',
    }
    properties: dict[str, str] = {}
    for line in completed.stdout.splitlines():
        if '=' not in line:
            continue
        key, value = line.split('=', 1)
        if key in keep:
            properties[key] = value
    return properties


def _unique_paths(paths: Iterable[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for path in paths:
        text = str(path).strip()
        if not text or text in seen:
            continue
        seen.add(text)
        result.append(text)
    return result


def _unique_ints(values: Iterable[int]) -> list[int]:
    result: list[int] = []
    seen: set[int] = set()
    for value in values:
        try:
            number = int(value)
        except (TypeError, ValueError):
            continue
        if number <= 0 or number in seen:
            continue
        seen.add(number)
        result.append(number)
    return result


def _role_defaults(role: str) -> tuple[int, str, str]:
    if role == 'motor_controller':
        return 115200, 'quad_md_ascii', 'quad_md'
    if role == 'imu':
        return 115200, 'yahboom_serial', 'yb_mra02_v1'
    if role == 'lidar':
        return 460800, 'sllidar_serial', 'c1'
    return 0, '', ''


def _configured_baudrate(entry: dict[str, Any], role: str) -> int:
    default_baudrate, _protocol, _profile = _role_defaults(role)
    try:
        baudrate = int(entry.get('baudrate', default_baudrate))
    except (TypeError, ValueError):
        baudrate = default_baudrate
    return baudrate if baudrate > 0 else default_baudrate


def _probe_baudrates_with_saved_first(
    saved_baudrate: int,
    fallback_baudrates: Sequence[int],
) -> tuple[int, ...]:
    return tuple(_unique_ints([saved_baudrate, *fallback_baudrates]))


def _entry_aliases(entry: dict[str, Any]) -> list[str]:
    raw_aliases = entry.get('aliases', [])
    aliases = raw_aliases if isinstance(raw_aliases, list) else []
    return _unique_paths([
        str(entry.get('device', '')).strip(),
        *(str(alias).strip() for alias in aliases),
    ])


def _usb_identity_conflicts(saved: dict[str, str], current: dict[str, str]) -> bool:
    """Return true when saved USB metadata says this is clearly another device."""
    if not saved or not current:
        return False
    for key in ('ID_SERIAL_SHORT', 'ID_SERIAL'):
        saved_value = saved.get(key, '').strip()
        current_value = current.get(key, '').strip()
        if saved_value and current_value and saved_value != current_value:
            return True
    for key in ('ID_VENDOR_ID', 'ID_MODEL_ID'):
        saved_value = saved.get(key, '').strip()
        current_value = current.get(key, '').strip()
        if saved_value and current_value and saved_value != current_value:
            return True
    return False


def _usb_identity_score(saved: dict[str, str], current: dict[str, str]) -> int:
    """Score topology-independent USB matches from strongest to weakest."""
    if not saved or not current:
        return 0
    for key in ('ID_SERIAL_SHORT', 'ID_SERIAL'):
        saved_value = saved.get(key, '').strip()
        if saved_value and saved_value == current.get(key, '').strip():
            return 100
    saved_vid = saved.get('ID_VENDOR_ID', '').strip()
    saved_pid = saved.get('ID_MODEL_ID', '').strip()
    if (
        saved_vid
        and saved_pid
        and saved_vid == current.get('ID_VENDOR_ID', '').strip()
        and saved_pid == current.get('ID_MODEL_ID', '').strip()
    ):
        return 20
    return 0


def _usb_identity_candidates(
    entry: dict[str, Any],
    used_realpaths: set[str],
) -> list[tuple[str, str]]:
    saved_usb = entry.get('usb', {})
    if not isinstance(saved_usb, dict):
        return []
    matches: list[tuple[int, str, str]] = []
    for candidate in serial_candidates():
        resolved = os.path.realpath(candidate)
        if resolved in used_realpaths:
            continue
        score = _usb_identity_score(saved_usb, udev_properties(candidate))
        if score <= 0:
            continue
        reason = (
            'matched saved USB serial metadata'
            if score >= 100
            else 'matched saved USB vendor/product metadata'
        )
        matches.append((score, candidate, reason))
    return [
        (candidate, reason)
        for _score, candidate, reason in sorted(
            matches,
            key=lambda item: (-item[0], item[1]),
        )
    ]


def _configured_result(
    role: str,
    entry: dict[str, Any],
    device: str,
    confidence: str,
    reason: str,
) -> DeviceResult:
    _default_baudrate, default_protocol, default_profile = _role_defaults(role)
    return DeviceResult(
        role=role,
        device=device,
        resolved_device=os.path.realpath(device),
        baudrate=_configured_baudrate(entry, role),
        confidence=confidence,
        reason=reason,
        protocol=str(entry.get('protocol', default_protocol) or default_protocol),
        profile=str(entry.get('profile', default_profile) or default_profile),
        parameters=dict(entry.get('parameters', {})),
    )


def _verify_configured_candidate(
    role: str,
    entry: dict[str, Any],
    candidate: str,
    confidence: str,
    reason: str,
    imu_baudrates: Sequence[int] = DEFAULT_IMU_BAUDRATES,
    lidar_baudrates: Sequence[int] = DEFAULT_SLLIDAR_BAUDRATES,
) -> tuple[Optional[DeviceResult], str]:
    """Verify a relocated candidate before assigning it to a configured role."""
    baudrate = _configured_baudrate(entry, role)
    if role == 'motor_controller':
        ok, probe_reason = probe_motor_controller(candidate, baudrate)
        if ok:
            return (
                _configured_result(
                    role,
                    entry,
                    preferred_stable_path(candidate),
                    confidence,
                    f'{reason}; {probe_reason}',
                ),
                '',
            )
        return None, probe_reason

    if role == 'imu':
        ok, detected_baudrate, probe_reason = probe_yahboom_imu(
            candidate,
            baudrates=_probe_baudrates_with_saved_first(baudrate, imu_baudrates),
        )
        if ok:
            result = _configured_result(
                role,
                entry,
                preferred_stable_path(candidate),
                confidence,
                f'{reason}; {probe_reason}',
            )
            return DeviceResult(**{**asdict(result), 'baudrate': detected_baudrate}), ''
        return None, probe_reason

    if role == 'lidar':
        baudrates = (baudrate,) if baudrate > 0 else tuple(lidar_baudrates)
        ok, detected_baudrate, probe_reason, profile, parameters = probe_sllidar(
            candidate,
            baudrates=baudrates,
        )
        if ok:
            result = _configured_result(
                role,
                entry,
                preferred_stable_path(candidate),
                confidence,
                f'{reason}; {probe_reason}',
            )
            return (
                DeviceResult(
                    **{
                        **asdict(result),
                        'baudrate': detected_baudrate,
                        'profile': profile or result.profile,
                        'parameters': parameters or result.parameters,
                    }
                ),
                '',
            )
        return None, probe_reason

    return None, f'unsupported configured role {role!r}'


def _valid_yahboom_frame(frame: bytes) -> bool:
    return (
        len(frame) == YAHBOOM_FRAME_LENGTH
        and frame[0] == YAHBOOM_FRAME_HEADER
        and frame[1] in YAHBOOM_FRAME_TYPES
        and (sum(frame[:10]) & 0xFF) == frame[10]
    )


def _scan_yahboom_frames(data: bytes) -> tuple[int, set[int]]:
    valid = 0
    frame_types: set[int] = set()
    index = 0
    while index + YAHBOOM_FRAME_LENGTH <= len(data):
        if data[index] != YAHBOOM_FRAME_HEADER:
            index += 1
            continue
        frame = data[index:index + YAHBOOM_FRAME_LENGTH]
        if _valid_yahboom_frame(frame):
            valid += 1
            frame_types.add(frame[1])
            index += YAHBOOM_FRAME_LENGTH
        else:
            index += 1
    return valid, frame_types


def _valid_yb_mra02_frame(frame: bytes) -> bool:
    return (
        len(frame) >= 5
        and frame[0] == YB_MRA02_FRAME_HEAD1
        and frame[1] == YB_MRA02_FRAME_HEAD2
        and len(frame) == frame[2]
        and frame[2] <= YB_MRA02_MAX_FRAME_LENGTH
        and frame[3] in YB_MRA02_FRAME_TYPES
        and (sum(frame[:-1]) & 0xFF) == frame[-1]
    )


def _scan_yb_mra02_frames(data: bytes) -> tuple[int, set[int]]:
    valid = 0
    frame_types: set[int] = set()
    index = 0
    while index + 5 <= len(data):
        if (
            data[index] != YB_MRA02_FRAME_HEAD1
            or data[index + 1] != YB_MRA02_FRAME_HEAD2
        ):
            index += 1
            continue
        frame_length = int(data[index + 2])
        if frame_length < 5 or frame_length > YB_MRA02_MAX_FRAME_LENGTH:
            index += 1
            continue
        frame_end = index + frame_length
        if frame_end > len(data):
            break
        frame = data[index:frame_end]
        if _valid_yb_mra02_frame(frame):
            valid += 1
            frame_types.add(frame[3])
            index = frame_end
        else:
            index += 1
    return valid, frame_types


def _byte_sample(data: bytes, limit: int = 48) -> str:
    if not data:
        return 'empty'
    sample = data[:limit]
    hex_text = sample.hex(' ')
    ascii_text = ''.join(chr(value) if 32 <= value < 127 else '.' for value in sample)
    printable = sum(1 for value in data if value in (9, 10, 13) or 32 <= value < 127)
    printable_ratio = printable / max(1, len(data))
    headers = data.count(bytes([YAHBOOM_FRAME_HEADER]))
    v2_headers = data.count(bytes([YB_MRA02_FRAME_HEAD1, YB_MRA02_FRAME_HEAD2]))
    return (
        f'hex[{len(sample)}/{len(data)}]={hex_text}; '
        f'ascii={ascii_text!r}; printable={printable_ratio:.0%}; '
        f'0x55_count={headers}; 7e23_count={v2_headers}'
    )


def _read_serial_bytes(
    port: serial.Serial,
    *,
    duration_sec: float,
    target_bytes: int,
) -> bytes:
    deadline = time.monotonic() + duration_sec
    data = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(512)
        if chunk:
            data.extend(chunk)
        if len(data) >= target_bytes:
            break
    return bytes(data)


def _enable_yahboom_imu_outputs(port: serial.Serial) -> None:
    """Ask Yahboom/Wit-style IMUs to stream ACC/GYRO/ANGLE/MAG frames.

    YB-MRA02-V1.0 can ship with a different output mask from the older rover
    IMU. The command is intentionally not followed by SAVE, so this is a
    runtime compatibility nudge rather than permanent module reconfiguration.
    """
    unlock = b'\xFF\xAA\x69\x88\xB5'
    output_acc_gyro_angle_mag_port = b'\xFF\xAA\x02\x3E\x00'
    output_rate_10hz = b'\xFF\xAA\x03\x06\x00'
    port.write(unlock)
    port.flush()
    time.sleep(0.03)
    port.write(output_acc_gyro_angle_mag_port)
    port.flush()
    time.sleep(0.03)
    port.write(output_rate_10hz)
    port.flush()
    time.sleep(0.12)


def probe_yahboom_imu(
    device: str,
    baudrates: Sequence[int] = DEFAULT_IMU_BAUDRATES,
) -> tuple[bool, int, str]:
    """Identify Yahboom/Wit-style 0x55/11-byte IMU streams.

    Older rover IMUs already stream acceleration and gyro frames. Newer
    YB-MRA02-V1.0 boards can expose a different output mask, so verification
    performs a passive read first and then asks the module to enable the core
    IMU frame set before retrying.
    """
    failures: list[str] = []
    probe_baudrates = list(baudrates)
    if 115200 in probe_baudrates:
        # The YB-MRA02 USB bridge can be quiet on the first open immediately
        # after hot-plug, then start streaming a moment later. Retry the native
        # baud after the other attempts before declaring setup failure.
        probe_baudrates.append(115200)

    for attempt_index, baudrate in enumerate(probe_baudrates):
        try:
            with serial.Serial(
                device,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.06,
                write_timeout=0.4,
            ) as port:
                time.sleep(0.20 if baudrate == 115200 else 0.08)
                data = _read_serial_bytes(
                    port,
                    duration_sec=1.60 if baudrate == 115200 else 0.85,
                    target_bytes=520 if baudrate == 115200 else 260,
                )
                valid, types = _scan_yahboom_frames(data)
                v2_valid, v2_types = _scan_yb_mra02_frames(data)
                type_text = ','.join(f'0x{value:02X}' for value in sorted(types))
                v2_type_text = ','.join(
                    f'0x{value:02X}' for value in sorted(v2_types)
                )
                has_core = YAHBOOM_REQUIRED_IMU_TYPES.issubset(types)
                if valid >= 2 and has_core:
                    return (
                        True,
                        baudrate,
                        f'Yahboom/YB-MRA02 0x55 frames verified: '
                        f'{valid} frames, types {type_text}',
                    )
                if v2_valid >= 2 and YB_MRA02_FUNC_RAW in v2_types:
                    return (
                        True,
                        baudrate,
                        f'Yahboom YB-MRA02 7E23 frames verified: '
                        f'{v2_valid} frames, functions {v2_type_text}',
                    )

                active_valid = 0
                active_types: set[int] = set()
                active_type_text = ''
                active_v2_valid = 0
                active_v2_types: set[int] = set()
                active_v2_type_text = ''
                try_enable_outputs = (
                    bool(types & YAHBOOM_USEFUL_IMU_TYPES)
                    or data.count(bytes([YAHBOOM_FRAME_HEADER])) > 0
                )
                if try_enable_outputs:
                    try:
                        _enable_yahboom_imu_outputs(port)
                        data = _read_serial_bytes(
                            port,
                            duration_sec=1.05,
                            target_bytes=320,
                        )
                        active_valid, active_types = _scan_yahboom_frames(data)
                        active_type_text = ','.join(
                            f'0x{value:02X}' for value in sorted(active_types)
                        )
                        active_v2_valid, active_v2_types = _scan_yb_mra02_frames(
                            data
                        )
                        active_v2_type_text = ','.join(
                            f'0x{value:02X}'
                            for value in sorted(active_v2_types)
                        )
                        if (
                            active_valid >= 2
                            and YAHBOOM_REQUIRED_IMU_TYPES.issubset(active_types)
                        ):
                            return (
                                True,
                                baudrate,
                                'Yahboom/YB-MRA02 0x55 frames verified after '
                                f'enabling IMU outputs: {active_valid} frames, '
                                f'types {active_type_text}',
                            )
                        if (
                            active_v2_valid >= 2
                            and YB_MRA02_FUNC_RAW in active_v2_types
                        ):
                            return (
                                True,
                                baudrate,
                                'Yahboom YB-MRA02 7E23 frames verified after '
                                f'enabling outputs: {active_v2_valid} frames, '
                                f'functions {active_v2_type_text}',
                            )
                    except (OSError, serial.SerialException) as exc:
                        failures.append(
                            f'{baudrate}: output enable failed after '
                            f'{valid} valid frames from {len(data)} bytes'
                            + (f', types {type_text}' if type_text else '')
                            + f'; {exc}'
                        )
                        continue

                failures.append(
                    f'{baudrate}'
                    + (f'#{attempt_index + 1}' if baudrate == 115200 else '')
                    + f': {valid} valid frames from {len(data)} bytes'
                    + (f', types {type_text}' if type_text else '')
                    + (
                        f'; {v2_valid} valid 7E23 frames'
                        + (f', functions {v2_type_text}' if v2_type_text else '')
                    )
                    + (
                        f'; after enable: {active_valid} valid frames'
                        + (f', types {active_type_text}' if active_type_text else '')
                        if try_enable_outputs
                        else ''
                    )
                    + (
                        f'; after enable: {active_v2_valid} valid 7E23 frames'
                        + (
                            f', functions {active_v2_type_text}'
                            if active_v2_type_text
                            else ''
                        )
                        if try_enable_outputs
                        else ''
                    )
                    + f'; sample: {_byte_sample(data)}'
                )
        except (OSError, serial.SerialException) as exc:
            failures.append(f'{baudrate}: {exc}')

    return False, 0, '; '.join(failures)


def _parse_sllidar_device_info(data: bytes) -> Optional[dict[str, Any]]:
    start = 0
    while True:
        index = data.find(SLLIDAR_ANSWER_SYNC, start)
        if index < 0 or index + 7 > len(data):
            return None

        descriptor = data[index:index + 7]
        size_and_subtype = int.from_bytes(descriptor[2:6], 'little')
        payload_size = size_and_subtype & 0x3FFFFFFF
        answer_type = descriptor[6]
        payload_start = index + 7
        payload_end = payload_start + payload_size

        if (
            answer_type == SLLIDAR_DEVICE_INFO_TYPE
            and payload_size == SLLIDAR_DEVICE_INFO_LENGTH
            and payload_end <= len(data)
        ):
            payload = data[payload_start:payload_end]
            firmware = int.from_bytes(payload[1:3], 'little')
            return {
                'model_code': int(payload[0]),
                'firmware': f'{firmware >> 8}.{firmware & 0xFF:02d}',
                'hardware_revision': int(payload[3]),
                'lidar_serial_number': payload[4:20].hex().upper(),
            }
        start = index + 1


def _sllidar_profile(baudrate: int) -> tuple[str, dict[str, Any]]:
    if baudrate == 460800:
        return 'c1', {
            'scan_mode': 'Standard',
            'range_min': 0.17,
            'scan_frequency': 10.0,
        }
    if baudrate == 115200:
        return 'serial_115200', {
            'scan_mode': 'Sensitivity',
            'range_min': 0.15,
            'scan_frequency': 10.0,
        }
    if baudrate == 256000:
        return 'serial_256000', {
            'scan_mode': 'Sensitivity',
            'range_min': 0.15,
            'scan_frequency': 10.0,
        }
    if baudrate == 1000000:
        return 'serial_1000000', {
            'scan_mode': 'DenseBoost',
            'range_min': 0.05,
            'scan_frequency': 10.0,
        }
    return 'serial', {
        'scan_mode': 'Standard',
        'range_min': 0.15,
        'scan_frequency': 10.0,
    }


def probe_sllidar(
    device: str,
    baudrates: Sequence[int] = DEFAULT_SLLIDAR_BAUDRATES,
) -> tuple[bool, int, str, str, dict[str, Any]]:
    """Identify a serial SLLIDAR and infer its runtime serial profile.

    STOP and GET_DEVICE_INFO do not start autonomous rover motion. The lidar
    motor can briefly react to serial/DTR state on some models, so the setup
    wizard still asks for a stationary, safely supported rover.
    """
    failures: list[str] = []
    for baudrate in baudrates:
        try:
            with serial.Serial(
                device,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.04,
                write_timeout=0.5,
            ) as port:
                time.sleep(0.10)
                try:
                    port.write(SLLIDAR_STOP)
                    port.flush()
                except serial.SerialException:
                    pass
                time.sleep(0.06)
                port.reset_input_buffer()

                data = bytearray()
                info: Optional[dict[str, Any]] = None
                for _attempt in range(2):
                    port.write(SLLIDAR_GET_INFO)
                    port.flush()
                    deadline = time.monotonic() + 0.85
                    while time.monotonic() < deadline:
                        chunk = port.read(256)
                        if chunk:
                            data.extend(chunk)
                            info = _parse_sllidar_device_info(bytes(data))
                            if info:
                                break
                    if info:
                        break

                try:
                    port.write(SLLIDAR_STOP)
                    port.flush()
                except serial.SerialException:
                    pass

                if info:
                    profile, parameters = _sllidar_profile(baudrate)
                    parameters = {**parameters, **info}
                    reason = (
                        'SLLIDAR device info verified: '
                        f"model {info['model_code']}, firmware {info['firmware']}, "
                        f"hardware {info['hardware_revision']}, "
                        f"serial {info['lidar_serial_number']}"
                    )
                    return True, baudrate, reason, profile, parameters
                failures.append(
                    f'{baudrate}: no valid device-info response '
                    f'({len(data)} bytes read)'
                )
        except (OSError, serial.SerialException) as exc:
            failures.append(f'{baudrate}: {exc}')

    return False, 0, '; '.join(failures), '', {}


def probe_motor_controller(
    device: str,
    baudrate: int = 115200,
) -> tuple[bool, str]:
    """Verify Quad-MD using feedback and an explicit zero-speed command."""
    try:
        with serial.Serial(
            device,
            baudrate=baudrate,
            timeout=0.05,
            write_timeout=0.5,
        ) as port:
            time.sleep(0.12)
            port.reset_input_buffer()
            port.write(b'$upload:1,0,1#')
            port.flush()

            deadline = time.monotonic() + 1.2
            data = bytearray()
            while time.monotonic() < deadline:
                chunk = port.read(256)
                if chunk:
                    data.extend(chunk)
                    text = data.decode('ascii', errors='ignore')
                    if '$MAll:' in text or '$MSPD:' in text:
                        try:
                            port.write(b'$upload:0,0,0#$spd:0,0,0,0#')
                            port.flush()
                        except serial.SerialException:
                            pass
                        return True, 'received Quad-MD $MAll/$MSPD feedback frame'

            try:
                port.write(b'$upload:0,0,0#$spd:0,0,0,0#')
                port.flush()
            except serial.SerialException:
                pass
            return False, f'no Quad-MD frame ({len(data)} bytes read)'
    except (OSError, serial.SerialException) as exc:
        return False, f'open/probe failed: {exc}'


def _make_link(link: Path, target: str) -> None:
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(os.path.realpath(target))


def _clear_runtime(runtime: Path) -> None:
    runtime.mkdir(parents=True, exist_ok=True)
    for name in ('motor_controller', 'imu', 'lidar', 'devices.json'):
        path = runtime / name
        if path.is_symlink() or path.exists():
            path.unlink()


def _write_runtime(
    runtime_dir: str,
    results: dict[str, DeviceResult],
) -> None:
    runtime = Path(runtime_dir)
    _clear_runtime(runtime)
    for role, result in results.items():
        _make_link(runtime / role, result.device)
    payload = {name: asdict(result) for name, result in results.items()}
    (runtime / 'devices.json').write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )


def expand_config_path(config_path: str = DEFAULT_DEVICE_CONFIG) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(config_path)))


def load_device_config(
    config_path: str = DEFAULT_DEVICE_CONFIG,
) -> dict[str, Any]:
    path = expand_config_path(config_path)
    if not path.exists():
        raise RuntimeError(
            f'Device configuration not found: {path}. Run: '
            'ros2 run rover_device_manager setup_devices'
        )
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f'Cannot read device configuration {path}: {exc}') from exc
    if payload.get('schema_version') != CONFIG_SCHEMA_VERSION:
        raise RuntimeError(
            f'Unsupported device configuration schema in {path}: '
            f"{payload.get('schema_version')!r}"
        )
    if not isinstance(payload.get('devices'), dict):
        raise RuntimeError(f'Invalid device configuration: {path}')
    return payload


def save_device_config(
    devices: dict[str, dict[str, Any]],
    config_path: str = DEFAULT_DEVICE_CONFIG,
) -> Path:
    path = expand_config_path(config_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'schema_version': CONFIG_SCHEMA_VERSION,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'identity_strategy': 'physical_usb_path_with_protocol_relocation',
        'devices': devices,
    }
    serialized = json.dumps(payload, indent=2, ensure_ascii=False) + '\n'
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=path.parent,
        prefix=f'.{path.name}.',
        delete=False,
    ) as temporary:
        temporary.write(serialized)
        temp_path = Path(temporary.name)
    os.chmod(temp_path, 0o600)
    temp_path.replace(path)
    return path


def _required_roles(require_imu: bool, require_lidar: bool) -> list[str]:
    required = ['motor_controller']
    if require_imu:
        required.append('imu')
    if require_lidar:
        required.append('lidar')
    return required


def _configured_role_result(
    role: str,
    entry: dict[str, Any],
    used_realpaths: set[str],
    imu_baudrates: Sequence[int],
    lidar_baudrates: Sequence[int],
) -> tuple[Optional[DeviceResult], str]:
    failures: list[str] = []
    aliases = _entry_aliases(entry)
    configured_device = aliases[0] if aliases else ''
    saved_usb = entry.get('usb', {})
    saved_usb = saved_usb if isinstance(saved_usb, dict) else {}

    if configured_device and os.path.exists(configured_device):
        resolved = os.path.realpath(configured_device)
        if resolved in used_realpaths:
            return (
                None,
                f'configured path collision: {configured_device} resolves to {resolved}',
            )
        current_usb = udev_properties(configured_device)
        if not _usb_identity_conflicts(saved_usb, current_usb):
            return (
                _configured_result(
                    role,
                    entry,
                    configured_device,
                    'configured_physical_path',
                    'loaded from persistent setup configuration',
                ),
                '',
            )
        result, reason = _verify_configured_candidate(
            role,
            entry,
            configured_device,
            'configured_path_protocol_verified',
            'configured path exists but USB metadata changed',
            imu_baudrates,
            lidar_baudrates,
        )
        if result is not None:
            return result, ''
        failures.append(f'{configured_device}: USB metadata changed and {reason}')

    for alias in aliases[1:]:
        if not os.path.exists(alias):
            continue
        resolved = os.path.realpath(alias)
        if resolved in used_realpaths:
            continue
        result, reason = _verify_configured_candidate(
            role,
            entry,
            alias,
            'configured_alias_protocol_verified',
            f'configured alias {alias} is available',
            imu_baudrates,
            lidar_baudrates,
        )
        if result is not None:
            return result, ''
        failures.append(f'{alias}: {reason}')

    for candidate, match_reason in _usb_identity_candidates(entry, used_realpaths):
        result, reason = _verify_configured_candidate(
            role,
            entry,
            candidate,
            'configured_usb_identity_protocol_verified',
            match_reason,
            imu_baudrates,
            lidar_baudrates,
        )
        if result is not None:
            return result, ''
        failures.append(f'{candidate}: {reason}')

    if configured_device:
        failures.insert(
            0,
            f'configured path is unavailable: {configured_device}',
        )
    return None, '; '.join(failures) or 'no configured candidate paths'


def _mark_protocol_relocated(
    results: dict[str, DeviceResult],
    reason: str,
) -> dict[str, DeviceResult]:
    relocated: dict[str, DeviceResult] = {}
    for role, result in results.items():
        relocated[role] = DeviceResult(
            **{
                **asdict(result),
                'confidence': f'{result.confidence}_configured_fallback',
                'reason': f'{reason}; {result.reason}',
            }
        )
    return relocated


def _configured_results(
    config_path: str,
    require_imu: bool,
    require_lidar: bool,
    imu_baudrates: Sequence[int] = DEFAULT_IMU_BAUDRATES,
    lidar_baudrates: Sequence[int] = DEFAULT_SLLIDAR_BAUDRATES,
    allow_protocol_relocation: bool = True,
) -> dict[str, DeviceResult]:
    payload = load_device_config(config_path)
    configured = payload['devices']
    required = _required_roles(require_imu, require_lidar)

    results: dict[str, DeviceResult] = {}
    used: set[str] = set()
    failures: list[str] = []
    for role in required:
        entry = configured.get(role)
        if not isinstance(entry, dict):
            failures.append(
                f'role {role!r} is absent from {expand_config_path(config_path)}'
            )
            continue

        result, failure = _configured_role_result(
            role,
            entry,
            used,
            imu_baudrates,
            lidar_baudrates,
        )
        if result is None:
            failures.append(f'{role}: {failure}')
            continue

        used.add(result.resolved_device)
        results[role] = result

    if not failures:
        return results

    if allow_protocol_relocation:
        reason = (
            'configured USB paths could not be used; relocated by protocol '
            f'discovery ({"; ".join(failures)})'
        )
        return _mark_protocol_relocated(
            _discover_roles(
                require_imu=require_imu,
                require_lidar=require_lidar,
                imu_baudrates=imu_baudrates,
                lidar_baudrates=lidar_baudrates,
            ),
            reason,
        )

    raise RuntimeError(
        'Configured device paths are unavailable or invalid. '
        + '; '.join(failures)
    )


def verify_results(results: dict[str, DeviceResult]) -> dict[str, DeviceResult]:
    verified: dict[str, DeviceResult] = {}
    for role, result in results.items():
        if role == 'motor_controller':
            ok, reason = probe_motor_controller(result.device, result.baudrate)
            if not ok:
                raise RuntimeError(f'Configured motor controller failed verification: {reason}')
        elif role == 'imu':
            fallback_baudrates = _probe_baudrates_with_saved_first(
                result.baudrate,
                DEFAULT_IMU_BAUDRATES,
            )
            ok, baudrate, reason = probe_yahboom_imu(
                result.device,
                baudrates=fallback_baudrates,
            )
            if not ok:
                raise RuntimeError(f'Configured IMU failed verification: {reason}')
            result = DeviceResult(**{**asdict(result), 'baudrate': baudrate})
        elif role == 'lidar':
            ok, baudrate, reason, profile, parameters = probe_sllidar(
                result.device,
                baudrates=(result.baudrate,),
            )
            if not ok:
                raise RuntimeError(f'Configured lidar failed verification: {reason}')
            result = DeviceResult(
                **{
                    **asdict(result),
                    'baudrate': baudrate,
                    'profile': profile or result.profile,
                    'parameters': parameters or result.parameters,
                }
            )
        verified[role] = DeviceResult(
            **{
                **asdict(result),
                'confidence': 'configured_and_protocol_verified',
                'reason': reason,
            }
        )
    return verified


def prepare_devices(
    mode: str = 'configured',
    config_path: str = DEFAULT_DEVICE_CONFIG,
    runtime_dir: str = '/tmp/rover_devices',
    require_imu: bool = True,
    require_lidar: bool = False,
    motor_device: Optional[str] = None,
    imu_device: Optional[str] = None,
    lidar_device: Optional[str] = None,
    imu_baudrates: Sequence[int] = DEFAULT_IMU_BAUDRATES,
    lidar_baudrates: Sequence[int] = DEFAULT_SLLIDAR_BAUDRATES,
) -> dict[str, DeviceResult]:
    normalized = mode.strip().lower()
    if any((motor_device, imu_device, lidar_device)):
        normalized = 'full'
    if normalized == 'configured':
        results = _configured_results(
            config_path,
            require_imu,
            require_lidar,
            imu_baudrates=imu_baudrates,
            lidar_baudrates=lidar_baudrates,
            allow_protocol_relocation=True,
        )
    elif normalized == 'verify':
        configured_results = _configured_results(
            config_path,
            require_imu,
            require_lidar,
            imu_baudrates=imu_baudrates,
            lidar_baudrates=lidar_baudrates,
            allow_protocol_relocation=True,
        )
        try:
            results = verify_results(configured_results)
        except RuntimeError as exc:
            results = _mark_protocol_relocated(
                _discover_roles(
                    require_imu=require_imu,
                    require_lidar=require_lidar,
                    motor_device=motor_device,
                    imu_device=imu_device,
                    lidar_device=lidar_device,
                    imu_baudrates=imu_baudrates,
                    lidar_baudrates=lidar_baudrates,
                ),
                f'configured verification failed; relocated by protocol discovery ({exc})',
            )
    elif normalized == 'full':
        results = _discover_roles(
            require_imu=require_imu,
            require_lidar=require_lidar,
            motor_device=motor_device,
            imu_device=imu_device,
            lidar_device=lidar_device,
            imu_baudrates=imu_baudrates,
            lidar_baudrates=lidar_baudrates,
        )
    else:
        raise RuntimeError(
            f'Unknown discovery mode {mode!r}; use configured, verify or full'
        )

    _write_runtime(runtime_dir, results)
    return results


def _discover_roles(
    require_imu: bool = True,
    require_lidar: bool = False,
    motor_device: Optional[str] = None,
    imu_device: Optional[str] = None,
    lidar_device: Optional[str] = None,
    imu_baudrates: Sequence[int] = DEFAULT_IMU_BAUDRATES,
    lidar_baudrates: Sequence[int] = DEFAULT_SLLIDAR_BAUDRATES,
) -> dict[str, DeviceResult]:
    candidates = serial_candidates(
        path for path in (motor_device, imu_device, lidar_device) if path
    )
    if not candidates:
        raise RuntimeError(
            'No /dev/ttyUSB*, /dev/ttyACM* or serial aliases found'
        )

    results: dict[str, DeviceResult] = {}
    used_realpaths: set[str] = set()

    imu_search = [imu_device] if imu_device else candidates
    imu_failures: list[str] = []
    if require_imu or imu_device:
        for candidate in imu_search:
            if not candidate:
                continue
            ok, baudrate, reason = probe_yahboom_imu(
                candidate,
                baudrates=imu_baudrates,
            )
            if ok:
                resolved = os.path.realpath(candidate)
                results['imu'] = DeviceResult(
                    role='imu',
                    device=preferred_stable_path(candidate),
                    resolved_device=resolved,
                    baudrate=baudrate,
                    confidence='protocol_verified',
                    reason=reason,
                    protocol='yahboom_serial',
                    profile='yb_mra02_v1',
                )
                used_realpaths.add(resolved)
                break
            imu_failures.append(f'{candidate}: {reason}')

        if require_imu and 'imu' not in results:
            details = '; '.join(imu_failures) or 'no candidates'
            raise RuntimeError(f'Yahboom/YB-MRA02 IMU not detected. {details}')

    # Check Quad-MD before sending SLLIDAR binary commands to all remaining
    # ports. This avoids treating a continuous $MAll/$MSPD stream as lidar data.
    motor_search = [motor_device] if motor_device else candidates
    motor_failures: list[str] = []
    for candidate in motor_search:
        if not candidate:
            continue
        resolved = os.path.realpath(candidate)
        if resolved in used_realpaths:
            continue
        ok, reason = probe_motor_controller(candidate)
        if ok:
            results['motor_controller'] = DeviceResult(
                role='motor_controller',
                device=preferred_stable_path(candidate),
                resolved_device=resolved,
                baudrate=115200,
                confidence='protocol_verified',
                reason=reason,
                protocol='quad_md_ascii',
                profile='quad_md',
            )
            used_realpaths.add(resolved)
            break
        motor_failures.append(f'{candidate}: {reason}')

    if 'motor_controller' not in results:
        details = '; '.join(motor_failures) or 'no candidates'
        raise RuntimeError(f'Motor controller not detected. {details}')

    lidar_search = [lidar_device] if lidar_device else candidates
    lidar_failures: list[str] = []
    if require_lidar or lidar_device:
        for candidate in lidar_search:
            if not candidate:
                continue
            resolved = os.path.realpath(candidate)
            if resolved in used_realpaths:
                continue
            ok, baudrate, reason, profile, parameters = probe_sllidar(
                candidate,
                baudrates=lidar_baudrates,
            )
            if ok:
                results['lidar'] = DeviceResult(
                    role='lidar',
                    device=preferred_stable_path(candidate),
                    resolved_device=resolved,
                    baudrate=baudrate,
                    confidence='protocol_verified',
                    reason=reason,
                    protocol='sllidar_serial',
                    profile=profile,
                    parameters=parameters,
                )
                used_realpaths.add(resolved)
                break
            lidar_failures.append(f'{candidate}: {reason}')

        if require_lidar and 'lidar' not in results:
            details = '; '.join(lidar_failures) or 'no candidates'
            raise RuntimeError(f'SLLIDAR not detected. {details}')

    resolved_roles = [result.resolved_device for result in results.values()]
    if len(resolved_roles) != len(set(resolved_roles)):
        raise RuntimeError('One physical serial device was assigned more than once')

    return results


def discover(
    runtime_dir: str = '/tmp/rover_devices',
    require_imu: bool = True,
    require_lidar: bool = False,
    motor_device: Optional[str] = None,
    imu_device: Optional[str] = None,
    lidar_device: Optional[str] = None,
    imu_baudrates: Sequence[int] = DEFAULT_IMU_BAUDRATES,
    lidar_baudrates: Sequence[int] = DEFAULT_SLLIDAR_BAUDRATES,
) -> dict[str, DeviceResult]:
    """Full protocol discovery. Intended for diagnostics, not normal launch."""
    results = _discover_roles(
        require_imu=require_imu,
        require_lidar=require_lidar,
        motor_device=motor_device,
        imu_device=imu_device,
        lidar_device=lidar_device,
        imu_baudrates=imu_baudrates,
        lidar_baudrates=lidar_baudrates,
    )
    _write_runtime(runtime_dir, results)
    return results
