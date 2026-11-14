from __future__ import annotations

import json
import math
from typing import Any


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def clamp_int(value: int | float, low: int, high: int) -> int:
    return max(low, min(high, int(round(float(value)))))


def parse_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in {'1', 'true', 'yes', 'y', 'on', 'enable', 'enabled', 'да', 'вкл', 'включи'}:
        return True
    if text in {'0', 'false', 'no', 'n', 'off', 'disable', 'disabled', 'нет', 'выкл', 'выключи'}:
        return False
    return default


def normalize_effect(effect: str | None) -> str:
    aliases = {
        'solid': 'fill',
        'constant': 'fill',
        'on': 'fill',
        'включено': 'fill',
        'постоянный': 'fill',
        'мигание': 'blink',
        'blink_slow': 'blink',
        'fast_blink': 'blink_fast',
        'быстрое_мигание': 'blink_fast',
        'pulse': 'fade',
        'пульсация': 'fade',
        'chase': 'wipe',
        'бегущий': 'wipe',
        'gradient': 'rainbow_fill',
        'радуга': 'rainbow',
    }
    supported = {
        'fill',
        'blink',
        'blink_fast',
        'fade',
        'wipe',
        'flash',
        'rainbow',
        'rainbow_fill',
    }
    raw = str(effect or 'fill').strip().lower().replace('-', '_').replace(' ', '_')
    effect_name = aliases.get(raw, raw)
    if effect_name not in supported:
        return 'fill'
    return effect_name


def parse_color(color: Any, default: str = '#16B8F3') -> tuple[int, int, int]:
    names = {
        'zima_blue': '#16B8F3',
        'zima': '#16B8F3',
        'blue': '#0000FF',
        'голубой': '#16B8F3',
        'синий': '#0000FF',
        'cyan': '#00FFFF',
        'white': '#FFFFFF',
        'белый': '#FFFFFF',
        'red': '#FF0000',
        'красный': '#FF0000',
        'green': '#00FF00',
        'зеленый': '#00FF00',
        'зелёный': '#00FF00',
        'yellow': '#FFFF00',
        'желтый': '#FFFF00',
        'жёлтый': '#FFFF00',
        'orange': '#FF8000',
        'оранжевый': '#FF8000',
        'purple': '#8000FF',
        'фиолетовый': '#8000FF',
        'pink': '#FF40A0',
        'розовый': '#FF40A0',
        'black': '#000000',
        'off': '#000000',
    }
    if isinstance(color, (list, tuple)) and len(color) >= 3:
        return (
            clamp_int(color[0], 0, 255),
            clamp_int(color[1], 0, 255),
            clamp_int(color[2], 0, 255),
        )
    text = str(color or default).strip()
    text = names.get(text.lower(), text)
    if not text.startswith('#'):
        text = names.get(text.lower(), default)
    raw = text.strip().lstrip('#')
    if len(raw) != 6:
        raw = default.lstrip('#')
    try:
        return (int(raw[0:2], 16), int(raw[2:4], 16), int(raw[4:6], 16))
    except ValueError:
        raw = default.lstrip('#')
        return (int(raw[0:2], 16), int(raw[2:4], 16), int(raw[4:6], 16))


def color_to_hex(red: int, green: int, blue: int) -> str:
    return '#%02X%02X%02X' % (
        clamp_int(red, 0, 255),
        clamp_int(green, 0, 255),
        clamp_int(blue, 0, 255),
    )


def yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def json_dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(',', ':'))


def json_dumps_pretty(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2)


def safe_json_loads(text: str) -> Any:
    """Load JSON from a model response.

    Many open models add newlines, markdown fences, or a short prefix even when asked
    for JSON. This helper first tries strict JSON, then extracts the first balanced
    JSON object/array from the response.
    """
    raw = str(text).strip()
    if raw.startswith('```'):
        lines = raw.splitlines()
        if lines and lines[0].strip().lower().startswith('```json'):
            lines = lines[1:]
        elif lines and lines[0].strip().startswith('```'):
            lines = lines[1:]
        if lines and lines[-1].strip().startswith('```'):
            lines = lines[:-1]
        raw = '\n'.join(lines).strip()
    try:
        return json.loads(raw)
    except json.JSONDecodeError as first_exc:
        start_candidates = [idx for idx in (raw.find('{'), raw.find('[')) if idx != -1]
        if not start_candidates:
            raise first_exc
        start = min(start_candidates)
        opener = raw[start]
        closer = '}' if opener == '{' else ']'
        depth = 0
        in_string = False
        escape = False
        for idx in range(start, len(raw)):
            ch = raw[idx]
            if in_string:
                if escape:
                    escape = False
                elif ch == '\\':
                    escape = True
                elif ch == '"':
                    in_string = False
                continue
            if ch == '"':
                in_string = True
            elif ch == opener:
                depth += 1
            elif ch == closer:
                depth -= 1
                if depth == 0:
                    return json.loads(raw[start:idx + 1])
        return json.loads(raw[start:])
