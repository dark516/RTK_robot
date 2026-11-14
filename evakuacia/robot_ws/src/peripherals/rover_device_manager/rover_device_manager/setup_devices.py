from __future__ import annotations

import argparse
import os
import re
import selectors
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from .discovery import (
    DEFAULT_DEVICE_CONFIG,
    DEFAULT_IMU_BAUDRATES,
    DEFAULT_SLLIDAR_BAUDRATES,
    DeviceResult,
    physical_serial_devices,
    preferred_stable_path,
    prepare_devices,
    probe_motor_controller,
    probe_sllidar,
    probe_yahboom_imu,
    save_device_config,
    serial_aliases,
    udev_properties,
)


ROLE_LABELS = {
    'motor_controller': 'Quad-MD motor controller',
    'imu': 'Yahboom YB-MRA02-V1.0 IMU',
    'lidar': 'SLLIDAR',
}


SLLIDAR_SDK_PROFILES = (
    (460800, 'c1', 'Standard', 0.17),
    (115200, 'serial_115200', 'Sensitivity', 0.15),
    (256000, 'serial_256000', 'Sensitivity', 0.15),
    (1000000, 'serial_1000000', 'DenseBoost', 0.05),
)


def _stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=7.0)
        return
    except (OSError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=2.0)
        return
    except (OSError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=1.0)
    except (OSError, subprocess.TimeoutExpired):
        pass


def _probe_sllidar_with_official_sdk(
    device: str,
) -> tuple[bool, int, str, str, dict[str, Any]]:
    """Use the installed Slamtec SDK node as the authoritative setup probe."""
    if shutil.which('ros2') is None:
        return False, 0, 'ros2 executable not found', '', {}

    failures: list[str] = []
    for baudrate, profile, scan_mode, range_min in SLLIDAR_SDK_PROFILES:
        print(
            f'  Trying official SLLIDAR SDK: {baudrate} baud, '
            f'scan_mode={scan_mode}...'
        )
        command = [
            'ros2', 'run', 'sllidar_ros2', 'sllidar_node',
            '--ros-args',
            '-r', '__node:=sllidar_setup_probe',
            '-p', 'channel_type:=serial',
            '-p', f'serial_port:={device}',
            '-p', f'serial_baudrate:={baudrate}',
            '-p', 'frame_id:=lidar_link',
            '-p', 'inverted:=false',
            '-p', 'angle_compensate:=true',
            '-p', f'scan_mode:={scan_mode}',
            '-p', f'range_min:={range_min}',
        ]
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=False,
                bufsize=0,
                start_new_session=True,
            )
        except OSError as exc:
            return False, 0, f'cannot start official SLLIDAR SDK: {exc}', '', {}

        selector = selectors.DefaultSelector()
        assert process.stdout is not None
        output_fd = process.stdout.fileno()
        selector.register(output_fd, selectors.EVENT_READ)
        deadline = time.monotonic() + 7.0
        output = bytearray()
        success = False
        while time.monotonic() < deadline:
            events = selector.select(timeout=0.2)
            for key, _mask in events:
                try:
                    chunk = os.read(key.fd, 4096)
                except OSError:
                    chunk = b''
                if chunk:
                    output.extend(chunk)
                    decoded = output.decode('utf-8', errors='replace')
                    if (
                        'SLLidar health status : OK' in decoded
                        and 'current scan mode:' in decoded
                    ):
                        success = True
                        break
            if success or process.poll() is not None:
                break
        selector.close()
        _stop_process(process)

        text = output.decode('utf-8', errors='replace')
        lines = text.splitlines()
        if success:
            parameters: dict[str, Any] = {
                'scan_mode': scan_mode,
                'range_min': range_min,
                'scan_frequency': 10.0,
            }
            patterns = {
                'lidar_serial_number': r'SLLidar S/N:\s*([0-9A-Fa-f]+)',
                'firmware': r'Firmware Ver:\s*([^\s]+)',
                'hardware_revision': r'Hardware Rev:\s*(\d+)',
            }
            for key, pattern in patterns.items():
                match = re.search(pattern, text)
                if match:
                    value: Any = match.group(1)
                    if key == 'hardware_revision':
                        value = int(value)
                    parameters[key] = value
            mode_match = re.search(
                r'current scan mode:\s*([^,]+).*?max_distance:\s*([0-9.]+)\s*m,\s*'
                r'scan frequency:\s*([0-9.]+)\s*Hz',
                text,
            )
            if mode_match:
                parameters['scan_mode'] = mode_match.group(1).strip()
                parameters['max_distance'] = float(mode_match.group(2))
                parameters['scan_frequency'] = float(mode_match.group(3))
            reason = (
                'official SLLIDAR SDK reported health OK; '
                f'baudrate {baudrate}, scan mode {parameters["scan_mode"]}'
            )
            return True, baudrate, reason, profile, parameters

        last_lines = ' | '.join(lines[-4:]) if lines else 'no output'
        failures.append(f'{baudrate}/{scan_mode}: {last_lines}')

    return False, 0, '; '.join(failures), '', {}


def _print_devices(title: str, devices: dict[str, str]) -> None:
    print(title)
    if not devices:
        print('  (none)')
        return
    for resolved, alias in sorted(devices.items()):
        print(f'  {alias} -> {resolved}')


def _wait_for_new_device(
    known_realpaths: set[str],
    timeout_sec: float,
) -> tuple[str, str]:
    deadline = time.monotonic() + timeout_sec
    last_new: dict[str, str] = {}
    stable_since: float | None = None

    while time.monotonic() < deadline:
        current = physical_serial_devices()
        new = {
            resolved: alias
            for resolved, alias in current.items()
            if resolved not in known_realpaths
        }
        if new != last_new:
            last_new = new
            stable_since = time.monotonic() if new else None
        elif new and stable_since is not None and time.monotonic() - stable_since >= 1.0:
            if len(new) == 1:
                return next(iter(new.items()))
            print('\nMore than one new serial device appeared:')
            items = sorted(new.items())
            for index, (resolved, alias) in enumerate(items, start=1):
                print(f'  {index}. {alias} -> {resolved}')
            while True:
                answer = input(
                    'Enter the number of the device just connected, or press '
                    'Enter after unplugging the extra device(s): '
                ).strip()
                if not answer:
                    last_new = {}
                    stable_since = None
                    break
                try:
                    selected = int(answer) - 1
                    if 0 <= selected < len(items):
                        return items[selected]
                except ValueError:
                    pass
                print('Invalid selection.')
        time.sleep(0.25)

    raise TimeoutError(
        f'No new serial device detected within {timeout_sec:.0f} seconds'
    )


def _confirm_retry(reason: str) -> bool:
    print(f'  Verification failed: {reason}')
    while True:
        answer = input('  Retry this device? [Y/n]: ').strip().lower()
        if answer in ('', 'y', 'yes'):
            return True
        if answer in ('n', 'no'):
            return False


def _confirm_accept_unverified_imu(reason: str) -> bool:
    print(f'  Verification failed: {reason}')
    print(
        '  If this is definitely the only newly connected IMU, it can be saved '
        'by its stable USB path and verified later by rover_imu.'
    )
    while True:
        answer = input(
            '  Accept this port as IMU without protocol verification? [y/N]: '
        ).strip().lower()
        if answer in ('y', 'yes'):
            return True
        if answer in ('', 'n', 'no'):
            return False


def _detect_role(
    role: str,
    known_realpaths: set[str],
    timeout_sec: float,
) -> DeviceResult:
    label = ROLE_LABELS[role]
    while True:
        print()
        print(f'Connect only the {label} now.')
        print('Leave already accepted rover devices connected.')
        input('Press Enter after connecting it... ')
        resolved, detected_alias = _wait_for_new_device(
            known_realpaths,
            timeout_sec,
        )
        stable_path = preferred_stable_path(detected_alias)
        print(f'  New device: {stable_path} -> {resolved}')

        if role == 'motor_controller':
            ok, reason = probe_motor_controller(stable_path, 115200)
            if ok:
                return DeviceResult(
                    role=role,
                    device=stable_path,
                    resolved_device=resolved,
                    baudrate=115200,
                    confidence='setup_protocol_verified',
                    reason=reason,
                    protocol='quad_md_ascii',
                    profile='quad_md',
                )
        elif role == 'imu':
            time.sleep(1.0)
            ok, baudrate, reason = probe_yahboom_imu(
                stable_path,
                DEFAULT_IMU_BAUDRATES,
            )
            if ok:
                return DeviceResult(
                    role=role,
                    device=stable_path,
                    resolved_device=resolved,
                    baudrate=baudrate,
                    confidence='setup_protocol_verified',
                    reason=reason,
                    protocol='yahboom_serial',
                    profile='yb_mra02_v1',
                )
            if _confirm_accept_unverified_imu(reason):
                return DeviceResult(
                    role=role,
                    device=stable_path,
                    resolved_device=resolved,
                    baudrate=115200,
                    confidence='setup_physical_path_only',
                    reason=(
                        'Accepted by operator as YB-MRA02 IMU after protocol '
                        f'verification failed: {reason}'
                    ),
                    protocol='yahboom_serial',
                    profile='yb_mra02_v1',
                )
        else:
            ok, baudrate, reason, profile, parameters = (
                _probe_sllidar_with_official_sdk(stable_path)
            )
            if not ok:
                print('  Official SDK probe did not succeed; trying direct device-info probe.')
                ok, baudrate, reason, profile, parameters = probe_sllidar(
                    stable_path,
                    DEFAULT_SLLIDAR_BAUDRATES,
                )
            if ok:
                return DeviceResult(
                    role=role,
                    device=stable_path,
                    resolved_device=resolved,
                    baudrate=baudrate,
                    confidence='setup_official_sdk_verified',
                    reason=reason,
                    protocol='sllidar_serial',
                    profile=profile,
                    parameters=parameters,
                )

        if role == 'imu':
            print('  IMU was not accepted without verification.')
            while True:
                answer = input('  Retry this device? [Y/n]: ').strip().lower()
                if answer in ('', 'y', 'yes'):
                    retry = True
                    break
                if answer in ('n', 'no'):
                    retry = False
                    break
            if not retry:
                raise RuntimeError(f'{label} was not configured: {reason}')
            continue

        if not _confirm_retry(reason):
            raise RuntimeError(f'{label} was not configured: {reason}')


def _config_entry(result: DeviceResult) -> dict[str, Any]:
    entry = {
        'device': result.device,
        'aliases': serial_aliases(result.device),
        'baudrate': result.baudrate,
        'protocol': result.protocol,
        'profile': result.profile,
        'parameters': result.parameters,
        'usb': udev_properties(result.device),
        'setup_verification': result.reason,
    }
    return entry


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
        'Interactive one-time setup of Quad-MD, Yahboom YB-MRA02 IMU and SLLIDAR '
            'using stable USB paths plus protocol relocation fallback.'
        )
    )
    parser.add_argument('--config', default=DEFAULT_DEVICE_CONFIG)
    parser.add_argument('--runtime-dir', default='/tmp/rover_devices')
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()

    if not sys.stdin.isatty():
        raise SystemExit('setup_devices requires an interactive terminal')

    print('Rover serial-device setup')
    print('=========================')
    print('Stop robot.launch.py and any node that may have a serial port open.')
    print('Keep the rover stationary; for first hardware setup, raise the wheels.')
    print('This setup saves physical USB paths and any available USB aliases.')
    print()
    print('Disconnect the rover Quad-MD, IMU and lidar USB cables.')
    print('Unrelated serial devices may remain connected.')
    input('Press Enter when the three rover devices are disconnected... ')

    baseline = physical_serial_devices()
    _print_devices('Baseline serial devices:', baseline)
    known_realpaths = set(baseline)
    results: dict[str, DeviceResult] = {}

    try:
        for role in ('motor_controller', 'imu', 'lidar'):
            result = _detect_role(role, known_realpaths, args.timeout)
            results[role] = result
            known_realpaths.add(result.resolved_device)
            print(
                f'  Accepted {ROLE_LABELS[role]}: {result.device} '
                f'@ {result.baudrate}'
            )
            print(f'  {result.reason}')
    except (KeyboardInterrupt, EOFError):
        print('\nSetup cancelled; no configuration was written.', file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f'\nSetup failed; no configuration was written: {exc}', file=sys.stderr)
        raise SystemExit(2) from exc

    devices = {role: _config_entry(result) for role, result in results.items()}
    config_path = save_device_config(devices, args.config)

    try:
        prepared = prepare_devices(
            mode='configured',
            config_path=str(config_path),
            runtime_dir=args.runtime_dir,
            require_imu=True,
            require_lidar=True,
        )
    except Exception as exc:
        print(f'Configuration was saved, but runtime links failed: {exc}', file=sys.stderr)
        raise SystemExit(2) from exc

    print()
    print(f'Configuration saved to: {config_path}')
    print(f'Runtime aliases created in: {args.runtime_dir}')
    for role, result in prepared.items():
        print(
            f'  {role}: {Path(args.runtime_dir) / role} -> '
            f'{result.resolved_device} @ {result.baudrate}'
        )
    print()
    print('Normal launch now uses the saved configuration without scanning all ports:')
    print('  ros2 launch rover_bringup robot.launch.py')
    print()
    print('Optional one-time protocol verification:')
    print(
        '  ros2 run rover_device_manager discover_devices '
        '--mode verify --require-imu --require-lidar'
    )


if __name__ == '__main__':
    main()
