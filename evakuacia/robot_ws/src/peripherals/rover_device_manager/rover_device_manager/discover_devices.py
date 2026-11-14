from __future__ import annotations

import argparse
import sys

from .discovery import (
    DEFAULT_DEVICE_CONFIG,
    DEFAULT_SLLIDAR_BAUDRATES,
    prepare_devices,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Prepare rover serial aliases from saved configuration or run '
            'protocol discovery when the USB topology changed.'
        )
    )
    parser.add_argument(
        '--mode',
        choices=('configured', 'verify', 'full'),
        default='configured',
        help=(
            'configured: fast saved paths with protocol relocation fallback; '
            'verify: configured plus protocol checks; full: scan every serial port'
        ),
    )
    parser.add_argument('--config', default=DEFAULT_DEVICE_CONFIG)
    parser.add_argument('--runtime-dir', default='/tmp/rover_devices')
    parser.add_argument('--require-imu', action='store_true')
    parser.add_argument('--require-lidar', action='store_true')
    parser.add_argument('--motor-device', default=None)
    parser.add_argument('--imu-device', default=None)
    parser.add_argument('--lidar-device', default=None)
    parser.add_argument(
        '--lidar-baudrate',
        type=int,
        action='append',
        dest='lidar_baudrates',
        help='Baudrate to try in full mode; may be repeated',
    )
    args = parser.parse_args()

    try:
        results = prepare_devices(
            mode=args.mode,
            config_path=args.config,
            runtime_dir=args.runtime_dir,
            require_imu=args.require_imu,
            require_lidar=args.require_lidar,
            motor_device=args.motor_device,
            imu_device=args.imu_device,
            lidar_device=args.lidar_device,
            lidar_baudrates=(
                tuple(args.lidar_baudrates)
                if args.lidar_baudrates
                else DEFAULT_SLLIDAR_BAUDRATES
            ),
        )
    except Exception as exc:
        print(f'[rover_device_manager] ERROR: {exc}', file=sys.stderr)
        raise SystemExit(2) from exc

    for name, result in results.items():
        profile = f', profile={result.profile}' if result.profile else ''
        print(
            f'[rover_device_manager] {name}: {result.device} '
            f'-> {result.resolved_device} @ {result.baudrate}{profile}; '
            f'{result.confidence}; {result.reason}'
        )
    print(f'[rover_device_manager] aliases created in {args.runtime_dir}')


if __name__ == '__main__':
    main()
