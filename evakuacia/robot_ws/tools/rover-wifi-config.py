#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys


def run(command: list[str], timeout: float = 15.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def require_root() -> None:
    if hasattr(os, 'geteuid') and os.geteuid() != 0:
        print('This command must run as root', file=sys.stderr)
        sys.exit(1)


def parse_wifi_fields_from_text(text: str, interface: str) -> tuple[str, str]:
    iface_pattern = re.compile(rf'^\s+{re.escape(interface)}:\s*$')
    ap_pattern = re.compile(r'^\s+"?(.*?)"?\s*:\s*$')
    password_pattern = re.compile(r'^\s+password:\s+"?(.*?)"?\s*$')

    in_iface = False
    in_access_points = False
    ssid = ''
    password = ''

    for line in text.splitlines():
        if iface_pattern.match(line):
            in_iface = True
            in_access_points = False
            continue

        if in_iface and line.startswith('    ') and not line.startswith('      '):
            in_iface = False
            in_access_points = False

        if not in_iface:
            continue

        if line.strip() == 'access-points:':
            in_access_points = True
            continue

        if not in_access_points:
            continue

        if not ssid:
            match = ap_pattern.match(line)
            if match and match.group(1) not in {'auth', 'key-management', 'password'}:
                ssid = match.group(1)
                continue

        if ssid and not password:
            match = password_pattern.match(line)
            if match:
                password = match.group(1)
                break

    return ssid, password


def current_config(interface: str, netplan_file: str) -> int:
    require_root()
    config_path = Path(netplan_file)
    if not config_path.exists():
        print(f'Netplan file not found: {config_path}', file=sys.stderr)
        return 1

    try:
        text = config_path.read_text(encoding='utf-8')
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    ssid, password = parse_wifi_fields_from_text(text, interface)
    if not ssid:
        print(f'Could not find SSID for interface {interface} in {config_path}', file=sys.stderr)
        return 1

    print(f'SSID={ssid}')
    print(f'PASSWORD={password}')
    return 0


def replace_first_quoted_value(line: str, new_value: str) -> str:
    return re.sub(r'"[^"]*"', f'"{new_value}"', line, count=1)


def update_wifi_fields_in_text(text: str, interface: str, ssid: str, password: str) -> str:
    lines = text.splitlines(keepends=True)
    iface_pattern = re.compile(rf'^(?P<indent>\s+){re.escape(interface)}:\s*$')
    password_pattern = re.compile(r'^\s*password:\s*"[^"]*"\s*$')

    in_iface = False
    in_access_points = False
    iface_indent = ''
    ap_indent = ''
    ssid_updated = False
    password_updated = False

    for index, line in enumerate(lines):
        stripped = line.strip()

        if not in_iface:
            match = iface_pattern.match(line)
            if match:
                in_iface = True
                iface_indent = match.group('indent')
            continue

        current_indent = line[: len(line) - len(line.lstrip(' '))]
        if stripped and not current_indent.startswith(iface_indent + '    '):
            in_iface = False
            in_access_points = False
            continue

        if stripped == 'access-points:':
            in_access_points = True
            ap_indent = current_indent
            continue

        if not in_access_points:
            continue

        if stripped and len(current_indent) <= len(ap_indent):
            in_access_points = False
            continue

        if not ssid_updated and re.match(r'^\s*"[^"]*":\s*$', line):
            lines[index] = replace_first_quoted_value(line, ssid)
            ssid_updated = True
            continue

        if ssid_updated and not password_updated and password_pattern.match(line):
            lines[index] = replace_first_quoted_value(line, password)
            password_updated = True
            break

    if not ssid_updated or not password_updated:
        raise RuntimeError(
            f'Could not update SSID/password for interface {interface} in the target file'
        )

    return ''.join(lines)


def apply_config(interface: str, netplan_file: str, ssid: str, password: str) -> int:
    require_root()

    if not ssid:
        print('SSID must not be empty', file=sys.stderr)
        return 1
    if len(password) < 8:
        print('Wi-Fi password must be at least 8 characters', file=sys.stderr)
        return 1

    config_path = Path(netplan_file)
    if not config_path.exists():
        print(f'Netplan file not found: {config_path}', file=sys.stderr)
        return 1

    try:
        original_text = config_path.read_text(encoding='utf-8')
        updated_text = update_wifi_fields_in_text(original_text, interface, ssid, password)
        config_path.write_text(updated_text, encoding='utf-8')
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    generated = run(['netplan', 'generate'], timeout=20.0)
    if generated.returncode != 0:
        print(generated.stderr.strip() or generated.stdout.strip() or 'netplan generate failed', file=sys.stderr)
        return 1

    applied = run(['netplan', 'apply'], timeout=30.0)
    if applied.returncode != 0:
        print(applied.stderr.strip() or applied.stdout.strip() or 'netplan apply failed', file=sys.stderr)
        return 1

    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)

    current = subparsers.add_parser('current')
    current.add_argument('interface')
    current.add_argument('netplan_file')

    apply = subparsers.add_parser('apply')
    apply.add_argument('interface')
    apply.add_argument('netplan_file')
    apply.add_argument('ssid')
    apply.add_argument('password')

    args = parser.parse_args()
    if args.command == 'current':
        return current_config(args.interface, args.netplan_file)
    if args.command == 'apply':
        return apply_config(args.interface, args.netplan_file, args.ssid, args.password)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
