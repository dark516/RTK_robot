from __future__ import annotations

from copy import deepcopy
from typing import Any

LED_EFFECTS = ['fill', 'blink', 'blink_fast', 'fade', 'wipe', 'flash', 'rainbow', 'rainbow_fill']
LED_PRESETS = [
    'off', 'idle', 'zima_blue', 'blue', 'cyan', 'green', 'red', 'white', 'yellow', 'purple',
    'rainbow', 'thinking', 'navigation', 'manual_control', 'warning', 'blink_blue', 'success', 'error',
]

TOOL_SCHEMAS: list[dict[str, Any]] = [
    {
        'name': 'get_available_tools',
        'description': 'Return a categorized list of robot tools and short descriptions. Use when the user asks what you can do.',
        'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False},
    },
    {
        'name': 'wait',
        'description': 'Wait for a specified number of seconds. Useful inside scenarios: подожди 2 секунды, потом продолжи.',
        'inputSchema': {
            'type': 'object',
            'properties': {'duration_s': {'type': 'number', 'minimum': 0.0, 'maximum': 60.0, 'default': 1.0}},
            'additionalProperties': False,
        },
    },
    {
        'name': 'set_led_strip',
        'description': 'Turn the rover LED strip on/off and set color/effect. Use for включи ленту, выключи ленту, включи голубой свет.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'enabled': {'type': 'boolean', 'description': 'True to enable the strip, false to turn it off.'},
                'effect': {'type': 'string', 'description': 'LED effect.', 'enum': LED_EFFECTS, 'default': 'fill'},
                'brightness': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0, 'default': 0.35},
                'color': {'type': 'string', 'description': 'Primary color as #RRGGBB or a common color name.', 'default': '#16B8F3'},
                'secondary_color': {'type': 'string', 'description': 'Secondary color as #RRGGBB.', 'default': '#FFFFFF'},
                'effect_speed_hz': {'type': 'number', 'minimum': 0.05, 'maximum': 20.0, 'default': 1.0},
            },
            'required': ['enabled'],
            'additionalProperties': False,
        },
    },
    {
        'name': 'set_led_preset',
        'description': 'Apply a named LED strip preset. Good for semantic states like thinking, navigation, success, error, warning.',
        'inputSchema': {
            'type': 'object',
            'properties': {'preset': {'type': 'string', 'enum': LED_PRESETS}},
            'required': ['preset'],
            'additionalProperties': False,
        },
    },
    {
        'name': 'blink_led_strip',
        'description': 'Blink the LED strip a finite number of times, then restore it to steady color or off. Use for поморгай светодиодом/лентой.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'color': {'type': 'string', 'default': '#16B8F3'},
                'times': {'type': 'integer', 'minimum': 1, 'maximum': 20, 'default': 3},
                'interval_s': {'type': 'number', 'minimum': 0.05, 'maximum': 5.0, 'default': 0.35},
                'brightness': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0, 'default': 0.35},
                'restore': {'type': 'string', 'enum': ['off', 'steady', 'previous'], 'default': 'steady'},
            },
            'additionalProperties': False,
        },
    },
    {
        'name': 'drive_relative',
        'description': 'Odom-based relative mecanum movement in the robot local frame. forward_m is +base_link X, left_m is +base_link Y. Supports sideways/diagonal movement.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'forward_m': {'type': 'number', 'description': 'Meters forward. Negative means backward.', 'default': 0.30},
                'left_m': {'type': 'number', 'description': 'Meters left/sideways. Negative means right.', 'default': 0.0},
                'speed_mps': {'type': 'number', 'description': 'Translation speed magnitude in m/s.', 'default': 0.12},
                'timeout_s': {'type': 'number', 'description': 'Optional movement timeout.', 'default': 10.0},
            },
            'additionalProperties': False,
        },
    },
    {
        'name': 'turn_relative',
        'description': 'Odom-based rotation in place. Positive angle is left/counter-clockwise, negative angle is right/clockwise.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'angle_deg': {'type': 'number', 'description': 'Relative turn angle in degrees. Positive = left, negative = right.'},
                'angular_speed_degps': {'type': 'number', 'description': 'Angular speed in degrees per second.', 'default': 45.0},
                'timeout_s': {'type': 'number', 'description': 'Optional turn timeout.', 'default': 10.0},
            },
            'required': ['angle_deg'],
            'additionalProperties': False,
        },
    },
    {
        'name': 'run_motion_sequence',
        'description': 'Run several actions in order: mecanum relative drive, turn, wait, LED actions, Nav2 navigation, stop. In sequences, navigate_to_pose waits for the Nav2 action result and then immediately continues.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'steps': {
                    'type': 'array',
                    'minItems': 1,
                    'maxItems': 20,
                    'items': {
                        'type': 'object',
                        'properties': {
                            'type': {
                                'type': 'string',
                                'enum': ['drive_relative', 'drive_forward', 'turn_relative', 'navigate_to_pose', 'set_led_strip', 'set_led_preset', 'blink_led_strip', 'wait', 'stop_motion'],
                            },
                            'forward_m': {'type': 'number'},
                            'left_m': {'type': 'number'},
                            'distance_m': {'type': 'number', 'description': 'Compatibility alias for forward_m on drive_forward steps.'},
                            'speed_mps': {'type': 'number'},
                            'angle_deg': {'type': 'number'},
                            'angular_speed_degps': {'type': 'number'},
                            'timeout_s': {'type': 'number'},
                            'x': {'type': 'number'},
                            'y': {'type': 'number'},
                            'yaw_deg': {'type': 'number'},
                            'frame_id': {'type': 'string'},
                            'wait_until_done': {'type': 'boolean'},
                            'enabled': {'type': 'boolean'},
                            'effect': {'type': 'string'},
                            'brightness': {'type': 'number'},
                            'color': {'type': 'string'},
                            'secondary_color': {'type': 'string'},
                            'effect_speed_hz': {'type': 'number'},
                            'preset': {'type': 'string'},
                            'times': {'type': 'integer'},
                            'interval_s': {'type': 'number'},
                            'restore': {'type': 'string'},
                            'duration_s': {'type': 'number'},
                            'cancel_navigation': {'type': 'boolean'},
                        },
                        'required': ['type'],
                        'additionalProperties': False,
                    },
                },
                'stop_on_error': {'type': 'boolean', 'default': True},
            },
            'required': ['steps'],
            'additionalProperties': False,
        },
    },
    {
        'name': 'drive_forward',
        'description': 'Compatibility alias for drive_relative(forward_m=distance_m, left_m=0). Prefer drive_relative for new plans.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'distance_m': {'type': 'number', 'description': 'Relative distance in meters. Negative means backward.', 'default': 0.30},
                'speed_mps': {'type': 'number', 'default': 0.12},
                'timeout_s': {'type': 'number', 'default': 10.0},
            },
            'required': ['distance_m'],
            'additionalProperties': False,
        },
    },
    {
        'name': 'run_relative_sequence',
        'description': 'Compatibility alias for run_motion_sequence. Prefer run_motion_sequence for new plans.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'steps': {'type': 'array', 'minItems': 1, 'maxItems': 20, 'items': {'type': 'object'}},
                'stop_on_error': {'type': 'boolean', 'default': True},
            },
            'required': ['steps'],
            'additionalProperties': False,
        },
    },
    {
        'name': 'stop_motion',
        'description': 'Immediately publish zero velocity to /cmd_vel_test. Optionally cancel active Nav2 navigation.',
        'inputSchema': {
            'type': 'object',
            'properties': {'cancel_navigation': {'type': 'boolean', 'default': False}},
            'additionalProperties': False,
        },
    },
    {
        'name': 'navigate_to_pose',
        'description': 'Send an absolute Nav2 goal in the map frame. Use for езжай в точку x y, not for simple relative movement. If wait_until_done=true, returns as soon as the Nav2 action succeeds/fails; timeout_s is only a maximum guard.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'x': {'type': 'number', 'description': 'Target x in meters in map frame.'},
                'y': {'type': 'number', 'description': 'Target y in meters in map frame.'},
                'yaw_deg': {'type': 'number', 'description': 'Target yaw in degrees.', 'default': 0.0},
                'frame_id': {'type': 'string', 'description': 'Usually map.', 'default': 'map'},
                'wait_until_done': {'type': 'boolean', 'description': 'If true, wait for Nav2 result before returning.', 'default': False},
                'timeout_s': {'type': 'number', 'description': 'Max wait when wait_until_done=true.', 'default': 60.0},
            },
            'required': ['x', 'y'],
            'additionalProperties': False,
        },
    },
    {
        'name': 'cancel_navigation',
        'description': 'Cancel the current Nav2 navigation goal if one is active.',
        'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False},
    },
    {
        'name': 'get_navigation_status',
        'description': 'Return last known Nav2 status, target, action readiness, and current pose.',
        'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False},
    },
    {
        'name': 'is_navigation_ready',
        'description': 'Check whether Nav2 action server and robot pose are available. Useful before sending Nav2 goals.',
        'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False},
    },
    {
        'name': 'get_robot_pose',
        'description': 'Return latest known robot pose from AMCL/map if available, otherwise odometry. Use for где ты сейчас / текущие координаты.',
        'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False},
    },
    {
        'name': 'get_laser_summary',
        'description': 'Return simple front/left/right/back min-distance summary from /scan.',
        'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False},
    },
    {
        'name': 'get_led_strip_state',
        'description': 'Return the latest LED strip state from /led_strip/state.',
        'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False},
    },
    {
        'name': 'get_system_status',
        'description': 'Check important rover ROS interfaces: LED service, Nav2 action, odom, AMCL pose, scan, LED state. Battery is intentionally not included.',
        'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False},
    },
]


def mcp_tools() -> list[dict[str, Any]]:
    return deepcopy(TOOL_SCHEMAS)


def openai_tools() -> list[dict[str, Any]]:
    converted: list[dict[str, Any]] = []
    for tool in TOOL_SCHEMAS:
        converted.append({
            'type': 'function',
            'function': {
                'name': tool['name'],
                'description': tool.get('description', ''),
                'parameters': deepcopy(tool.get('inputSchema', {'type': 'object', 'properties': {}})),
            },
        })
    return converted


def tool_names() -> set[str]:
    return {tool['name'] for tool in TOOL_SCHEMAS}
