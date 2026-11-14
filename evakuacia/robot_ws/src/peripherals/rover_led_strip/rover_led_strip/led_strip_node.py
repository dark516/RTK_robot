from __future__ import annotations

import colorsys
from glob import glob
import math
import time
from typing import Any

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter as RosParameter
from rover_interfaces.msg import LEDState, LEDStateArray, LedStripState
from rover_interfaces.srv import SetLEDEffect, SetLEDs, SetLedStripState

from rover_led_strip.ws2812_spidev import Color, WS2812SpiDriver


EFFECT_FILL = 'fill'
EFFECT_BLINK = 'blink'
EFFECT_BLINK_FAST = 'blink_fast'
EFFECT_FADE = 'fade'
EFFECT_WIPE = 'wipe'
EFFECT_FLASH = 'flash'
EFFECT_RAINBOW = 'rainbow'
EFFECT_RAINBOW_FILL = 'rainbow_fill'

SUPPORTED_EFFECTS = (
    EFFECT_FILL,
    EFFECT_BLINK,
    EFFECT_BLINK_FAST,
    EFFECT_FADE,
    EFFECT_WIPE,
    EFFECT_FLASH,
    EFFECT_RAINBOW,
    EFFECT_RAINBOW_FILL,
)

LEGACY_EFFECT_ALIASES = {
    'solid': EFFECT_FILL,
    'pulse': EFFECT_FADE,
    'chase': EFFECT_WIPE,
    'gradient': EFFECT_RAINBOW_FILL,
}

READ_ONLY_PARAMETER_NAMES = {
    'state_topic',
    'set_state_service',
    'native_state_topic',
    'set_effect_service',
    'set_leds_service',
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def clamp_byte(value: int | float) -> int:
    return max(0, min(255, int(round(float(value)))))


def parse_color_text(text: str) -> tuple[int, int, int]:
    raw = str(text).strip().lstrip('#')
    if len(raw) != 6:
        raise ValueError('Color must be in #RRGGBB format')
    return (
        int(raw[0:2], 16),
        int(raw[2:4], 16),
        int(raw[4:6], 16),
    )


def color_to_hex(color: tuple[int, int, int]) -> str:
    return '#%02X%02X%02X' % tuple(clamp_byte(channel) for channel in color)


def pack_color(color: tuple[int, int, int]) -> int:
    red, green, blue = (clamp_byte(channel) for channel in color)
    return (red << 16) | (green << 8) | blue


def hsv_to_rgb(hue: float, saturation: float, value: float) -> tuple[int, int, int]:
    red, green, blue = colorsys.hsv_to_rgb(hue % 1.0, saturation, value)
    return (
        clamp_byte(red * 255.0),
        clamp_byte(green * 255.0),
        clamp_byte(blue * 255.0),
    )


def normalize_effect(name: str) -> str:
    effect = str(name).strip().lower()
    if effect in LEGACY_EFFECT_ALIASES:
        return LEGACY_EFFECT_ALIASES[effect]
    if effect not in SUPPORTED_EFFECTS:
        return EFFECT_FILL
    return effect


class LedStripNode(Node):
    def __init__(self) -> None:
        super().__init__('led_strip_node')

        self.declare_parameter('led_transport', 'auto')
        self.declare_parameter('spi_bus', 1)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('led_count', 16)
        self.declare_parameter('frame_id', 'led_strip')
        self.declare_parameter('state_topic', '/led_strip/state')
        self.declare_parameter('set_state_service', '/led_strip/set_state')
        self.declare_parameter('native_state_topic', '/led/state')
        self.declare_parameter('set_effect_service', '/led/set_effect')
        self.declare_parameter('set_leds_service', '/led/set_leds')
        self.declare_parameter('enabled', False)
        self.declare_parameter('brightness', 0.35)
        self.declare_parameter('effect', EFFECT_FILL)
        self.declare_parameter('effect_speed_hz', 1.0)
        self.declare_parameter('primary_color', '#16B8F3')
        self.declare_parameter('secondary_color', '#FFFFFF')
        self.declare_parameter('startup_self_test', True)
        self.declare_parameter('startup_test_color', '#FFFFFF')
        self.declare_parameter('startup_test_step_sec', 0.06)
        self.declare_parameter('startup_test_hold_sec', 0.5)
        self.declare_parameter('animation_rate_hz', 30.0)
        self.declare_parameter('state_publish_hz', 5.0)
        self.declare_parameter('pixel_order', 'GRB')
        self.declare_parameter('gpio_pin', 18)

        self._settings = self._current_settings_from_parameters()
        self._driver: WS2812SpiDriver | None = None
        self._strip = None
        self._backend_name = ''
        self._backend_error = ''
        self._connected = False
        self._effect = normalize_effect(str(self._settings['effect']))
        self._previous_effect = self._effect
        self._effect_rgb = parse_color_text(str(self._settings['primary_color']))
        self._previous_rgb = self._effect_rgb
        self._manual_leds: dict[int, tuple[int, int, int]] = {}
        self._effect_start_time = time.monotonic()
        self._rainbow_hue = 0.0
        self._fade_start_pixels: list[tuple[int, int, int]] | None = None
        self._last_pixels: list[tuple[int, int, int]] = [
            (0, 0, 0) for _ in range(int(self._settings['led_count']))
        ]

        self._compat_state_publisher = self.create_publisher(
            LedStripState,
            str(self.get_parameter('state_topic').value),
            10,
        )
        self._native_state_publisher = self.create_publisher(
            LEDStateArray,
            str(self.get_parameter('native_state_topic').value),
            10,
        )
        self.create_service(
            SetLedStripState,
            str(self.get_parameter('set_state_service').value),
            self._handle_compat_set_state,
        )
        self.create_service(
            SetLEDEffect,
            str(self.get_parameter('set_effect_service').value),
            self._handle_set_effect,
        )
        self.create_service(
            SetLEDs,
            str(self.get_parameter('set_leds_service').value),
            self._handle_set_leds,
        )
        self.add_on_set_parameters_callback(self._on_parameter_set)

        self._animation_timer = self.create_timer(
            1.0 / max(1.0, float(self._settings['animation_rate_hz'])),
            self._animation_tick,
        )
        self._state_timer = self.create_timer(
            1.0 / max(0.5, float(self._settings['state_publish_hz'])),
            self._publish_state,
        )

        self._configure_backend(run_self_test=True)
        self._render_and_show(force=True)
        self.get_logger().info(
            'LED strip node ready via SPI %d.%d with %d LEDs'
            % (
                int(self._settings['spi_bus']),
                int(self._settings['spi_device']),
                int(self._settings['led_count']),
            )
        )

    def _current_settings_from_parameters(self) -> dict[str, object]:
        return self._validate_settings({
            'led_transport': str(self.get_parameter('led_transport').value).strip().lower(),
            'spi_bus': int(self.get_parameter('spi_bus').value),
            'spi_device': int(self.get_parameter('spi_device').value),
            'led_count': int(self.get_parameter('led_count').value),
            'frame_id': str(self.get_parameter('frame_id').value).strip(),
            'state_topic': str(self.get_parameter('state_topic').value).strip(),
            'set_state_service': str(self.get_parameter('set_state_service').value).strip(),
            'native_state_topic': str(self.get_parameter('native_state_topic').value).strip(),
            'set_effect_service': str(self.get_parameter('set_effect_service').value).strip(),
            'set_leds_service': str(self.get_parameter('set_leds_service').value).strip(),
            'enabled': bool(self.get_parameter('enabled').value),
            'brightness': float(self.get_parameter('brightness').value),
            'effect': str(self.get_parameter('effect').value).strip(),
            'effect_speed_hz': float(self.get_parameter('effect_speed_hz').value),
            'primary_color': str(self.get_parameter('primary_color').value).strip(),
            'secondary_color': str(self.get_parameter('secondary_color').value).strip(),
            'startup_self_test': bool(self.get_parameter('startup_self_test').value),
            'startup_test_color': str(self.get_parameter('startup_test_color').value).strip(),
            'startup_test_step_sec': float(self.get_parameter('startup_test_step_sec').value),
            'startup_test_hold_sec': float(self.get_parameter('startup_test_hold_sec').value),
            'animation_rate_hz': float(self.get_parameter('animation_rate_hz').value),
            'state_publish_hz': float(self.get_parameter('state_publish_hz').value),
            'pixel_order': str(self.get_parameter('pixel_order').value).strip().upper(),
            'gpio_pin': int(self.get_parameter('gpio_pin').value),
        })

    def _validate_settings(self, settings: dict[str, object]) -> dict[str, object]:
        validated = dict(settings)
        transport = str(validated['led_transport']).strip().lower() or 'auto'
        if transport not in {'auto', 'spi'}:
            raise ValueError("led_transport must be 'auto' or 'spi'")
        validated['led_transport'] = transport
        validated['spi_bus'] = max(0, int(validated['spi_bus']))
        validated['spi_device'] = max(0, int(validated['spi_device']))
        validated['led_count'] = max(1, min(1024, int(validated['led_count'])))
        validated['frame_id'] = str(validated['frame_id']).strip() or 'led_strip'
        validated['state_topic'] = str(validated['state_topic']).strip() or '/led_strip/state'
        validated['set_state_service'] = (
            str(validated['set_state_service']).strip() or '/led_strip/set_state'
        )
        validated['native_state_topic'] = (
            str(validated['native_state_topic']).strip() or '/led/state'
        )
        validated['set_effect_service'] = (
            str(validated['set_effect_service']).strip() or '/led/set_effect'
        )
        validated['set_leds_service'] = (
            str(validated['set_leds_service']).strip() or '/led/set_leds'
        )
        validated['enabled'] = bool(validated['enabled'])
        validated['brightness'] = clamp(float(validated['brightness']), 0.0, 1.0)
        validated['effect'] = normalize_effect(str(validated['effect']))
        validated['effect_speed_hz'] = clamp(float(validated['effect_speed_hz']), 0.05, 20.0)
        validated['primary_color'] = color_to_hex(parse_color_text(str(validated['primary_color'])))
        validated['secondary_color'] = color_to_hex(
            parse_color_text(str(validated['secondary_color']))
        )
        validated['startup_self_test'] = bool(validated['startup_self_test'])
        validated['startup_test_color'] = color_to_hex(
            parse_color_text(str(validated['startup_test_color']))
        )
        validated['startup_test_step_sec'] = clamp(
            float(validated['startup_test_step_sec']),
            0.01,
            2.0,
        )
        validated['startup_test_hold_sec'] = clamp(
            float(validated['startup_test_hold_sec']),
            0.0,
            5.0,
        )
        validated['animation_rate_hz'] = clamp(float(validated['animation_rate_hz']), 1.0, 120.0)
        validated['state_publish_hz'] = clamp(float(validated['state_publish_hz']), 0.5, 30.0)
        validated['pixel_order'] = str(validated['pixel_order']).strip().upper() or 'GRB'
        validated['gpio_pin'] = int(validated['gpio_pin'])
        return validated

    def _on_parameter_set(self, parameters: list[Any]) -> SetParametersResult:
        try:
            candidate = dict(self._settings)
            previous_animation_rate = float(self._settings['animation_rate_hz'])
            previous_state_publish_rate = float(self._settings['state_publish_hz'])
            for parameter in parameters:
                if parameter.name in READ_ONLY_PARAMETER_NAMES:
                    current_value = self._settings[parameter.name]
                    if parameter.value != current_value:
                        raise ValueError(f'{parameter.name} requires node restart to change')
                    continue
                candidate[parameter.name] = parameter.value
            validated = self._validate_settings(candidate)
            reconfigure_backend = any(
                validated[key] != self._settings.get(key)
                for key in ('led_transport', 'spi_bus', 'spi_device', 'led_count')
            )
            self._settings = validated
            self._effect = normalize_effect(str(self._settings['effect']))
            self._effect_rgb = parse_color_text(str(self._settings['primary_color']))
            self._effect_start_time = time.monotonic()
            self._manual_leds.clear()
            self._fade_start_pixels = None
            if reconfigure_backend:
                self._configure_backend(run_self_test=False)
            if float(self._settings['animation_rate_hz']) != previous_animation_rate:
                self._animation_timer.cancel()
                self._animation_timer = self.create_timer(
                    1.0 / max(1.0, float(self._settings['animation_rate_hz'])),
                    self._animation_tick,
                )
            if float(self._settings['state_publish_hz']) != previous_state_publish_rate:
                self._state_timer.cancel()
                self._state_timer = self.create_timer(
                    1.0 / max(0.5, float(self._settings['state_publish_hz'])),
                    self._publish_state,
                )
            self._resize_last_pixels(int(self._settings['led_count']))
            self._render_and_show(force=True)
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _resize_last_pixels(self, led_count: int) -> None:
        current = list(self._last_pixels)
        if len(current) < led_count:
            current.extend([(0, 0, 0)] * (led_count - len(current)))
        self._last_pixels = current[:led_count]
        self._manual_leds = {
            index: color
            for index, color in self._manual_leds.items()
            if index < led_count
        }

    def _configure_backend(self, *, run_self_test: bool) -> None:
        led_count = int(self._settings['led_count'])
        requested_bus = int(self._settings['spi_bus'])
        requested_device = int(self._settings['spi_device'])
        if self._driver is not None:
            try:
                self._driver.close()
            except Exception:
                pass
        self._driver = None
        self._strip = None
        selected_bus, selected_device = self._resolve_spi_target(
            requested_bus,
            requested_device,
        )
        try:
            self._driver = WS2812SpiDriver(
                spi_bus=selected_bus,
                spi_device=selected_device,
                led_count=led_count,
            )
            self._strip = self._driver.get_strip()
            self._strip.set_brightness(float(self._settings['brightness']))
            self._backend_name = f'ws2812-spidev:{selected_bus}.{selected_device}'
            self._backend_error = ''
            self._connected = True
            self._settings['spi_bus'] = selected_bus
            self._settings['spi_device'] = selected_device
        except Exception as exc:
            available = self._available_spi_targets()
            self._backend_name = 'ws2812-spidev:offline'
            self._backend_error = (
                f'Failed to open /dev/spidev{selected_bus}.{selected_device}: {exc}. '
                f'Available SPI devices: {", ".join(available) if available else "none"}.'
            )
            self._connected = False
        self._resize_last_pixels(led_count)
        if (
            self._strip is not None
            and run_self_test
            and bool(self._settings['startup_self_test'])
        ):
            self._run_startup_self_test()

    def _available_spi_targets(self) -> list[str]:
        return sorted(
            path.rsplit('/dev/', 1)[-1]
            for path in glob('/dev/spidev*')
        )

    def _resolve_spi_target(
        self,
        requested_bus: int,
        requested_device: int,
    ) -> tuple[int, int]:
        requested = f'spidev{requested_bus}.{requested_device}'
        available = self._available_spi_targets()
        if requested in available:
            return requested_bus, requested_device

        transport = str(self._settings.get('led_transport', 'auto')).strip().lower()
        if transport != 'auto':
            return requested_bus, requested_device

        for candidate in ('spidev0.0', 'spidev1.0'):
            if candidate in available:
                bus_text, device_text = candidate.replace('spidev', '').split('.', 1)
                self.get_logger().warn(
                    'Requested SPI device /dev/%s is unavailable, using /dev/%s instead.'
                    % (requested, candidate)
                )
                return int(bus_text), int(device_text)

        if available:
            first = available[0]
            bus_text, device_text = first.replace('spidev', '').split('.', 1)
            self.get_logger().warn(
                'Requested SPI device /dev/%s is unavailable, using /dev/%s instead.'
                % (requested, first)
            )
            return int(bus_text), int(device_text)

        return requested_bus, requested_device

    def _run_startup_self_test(self) -> None:
        if self._strip is None:
            return
        test_color = Color(*parse_color_text(str(self._settings['startup_test_color'])))
        step_sec = float(self._settings['startup_test_step_sec'])
        hold_sec = float(self._settings['startup_test_hold_sec'])
        self._strip.set_brightness(float(self._settings['brightness']))
        for index in range(int(self._settings['led_count'])):
            self._strip.set_pixel_color(index, test_color)
            self._strip.show()
            time.sleep(step_sec)
        time.sleep(hold_sec)
        self._strip.clear()
        self._last_pixels = [(0, 0, 0)] * int(self._settings['led_count'])

    def _handle_compat_set_state(
        self,
        request: SetLedStripState.Request,
        response: SetLedStripState.Response,
    ) -> SetLedStripState.Response:
        try:
            results = self.set_parameters([
                self._parameter('enabled', bool(request.enabled)),
                self._parameter('brightness', clamp(float(request.brightness), 0.0, 1.0)),
                self._parameter('effect', normalize_effect(str(request.effect))),
                self._parameter(
                    'effect_speed_hz',
                    clamp(float(request.effect_speed_hz), 0.05, 20.0),
                ),
                self._parameter(
                    'primary_color',
                    color_to_hex((request.red, request.green, request.blue)),
                ),
                self._parameter(
                    'secondary_color',
                    color_to_hex(
                        (
                            request.secondary_red,
                            request.secondary_green,
                            request.secondary_blue,
                        )
                    ),
                ),
            ])
            failures = [item.reason for item in results if not item.successful]
            response.success = not failures
            response.message = '; '.join(failures) if failures else 'LED strip state updated'
            return response
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response

    def _handle_set_effect(
        self,
        request: SetLEDEffect.Request,
        response: SetLEDEffect.Response,
    ) -> SetLEDEffect.Response:
        results = self.set_parameters([
            self._parameter('effect', normalize_effect(str(request.effect))),
            self._parameter(
                'primary_color',
                color_to_hex((request.r, request.g, request.b)),
            ),
            self._parameter('enabled', True),
        ])
        response.success = all(item.successful for item in results)
        return response

    def _handle_set_leds(
        self,
        request: SetLEDs.Request,
        response: SetLEDs.Response,
    ) -> SetLEDs.Response:
        self._manual_leds.clear()
        led_count = int(self._settings['led_count'])
        for led in request.leds:
            if 0 <= int(led.index) < led_count:
                self._manual_leds[int(led.index)] = (
                    clamp_byte(led.r),
                    clamp_byte(led.g),
                    clamp_byte(led.b),
                )
        self._render_and_show(force=True)
        response.success = True
        return response

    def _render_and_show(self, *, force: bool) -> None:
        if self._strip is None:
            return
        frame = self._compose_frame(force=force)
        self._strip.set_brightness(float(self._settings['brightness']))
        for index, color in enumerate(frame):
            self._strip.set_pixel_color(index, Color(*color))
        self._strip.show()
        self._last_pixels = frame

    def _compose_frame(self, *, force: bool) -> list[tuple[int, int, int]]:
        led_count = int(self._settings['led_count'])
        if not bool(self._settings['enabled']):
            return [(0, 0, 0)] * led_count

        if self._manual_leds:
            frame = [(0, 0, 0)] * led_count
            for index, color in self._manual_leds.items():
                if 0 <= index < led_count:
                    frame[index] = color
            return frame

        primary = parse_color_text(str(self._settings['primary_color']))
        secondary = parse_color_text(str(self._settings['secondary_color']))
        speed = float(self._settings['effect_speed_hz'])
        elapsed = (time.monotonic() - self._effect_start_time) * speed
        effect = self._effect

        if effect == EFFECT_FILL:
            return [primary] * led_count

        if effect == EFFECT_BLINK:
            return ([primary] * led_count) if (int(elapsed * 2) % 2) == 0 else [(0, 0, 0)] * led_count

        if effect == EFFECT_BLINK_FAST:
            return ([primary] * led_count) if (int(elapsed * 6) % 2) == 0 else [(0, 0, 0)] * led_count

        if effect == EFFECT_FADE:
            phase = (math.sin(elapsed * math.tau) + 1.0) * 0.5
            return [
                (
                    clamp_byte(secondary[0] + (primary[0] - secondary[0]) * phase),
                    clamp_byte(secondary[1] + (primary[1] - secondary[1]) * phase),
                    clamp_byte(secondary[2] + (primary[2] - secondary[2]) * phase),
                )
                for _ in range(led_count)
            ]

        if effect == EFFECT_WIPE:
            position = int((elapsed * 2.0) * led_count) % (led_count + 1)
            return [primary if index < position else (0, 0, 0) for index in range(led_count)]

        if effect == EFFECT_FLASH:
            phase = elapsed % 1.0
            if phase < 0.10 or (0.20 <= phase < 0.30):
                return [primary] * led_count
            if phase < 0.40:
                return [(0, 0, 0)] * led_count
            return [secondary] * led_count

        if effect == EFFECT_RAINBOW:
            hue = elapsed * 0.03
            return [
                hsv_to_rgb((hue + index / max(1, led_count)) % 1.0, 1.0, 1.0)
                for index in range(led_count)
            ]

        if effect == EFFECT_RAINBOW_FILL:
            hue = (elapsed * 0.08) % 1.0
            color = hsv_to_rgb(hue, 1.0, 1.0)
            return [color] * led_count

        if force and self._fade_start_pixels is None:
            self._fade_start_pixels = list(self._last_pixels)
        return [primary] * led_count

    def _animation_tick(self) -> None:
        self._render_and_show(force=False)

    def _publish_state(self) -> None:
        now = self.get_clock().now().to_msg()
        native_message = LEDStateArray()
        compat_message = LedStripState()
        compat_message.header.stamp = now
        compat_message.header.frame_id = str(self._settings['frame_id'])
        compat_message.connected = bool(self._connected)
        compat_message.enabled = bool(self._settings['enabled'])
        compat_message.led_count = int(self._settings['led_count'])
        compat_message.lit_count = sum(
            1 for red, green, blue in self._last_pixels if any((red, green, blue))
        )
        compat_message.brightness = float(self._settings['brightness'])
        compat_message.effect = str(self._effect)
        compat_message.effect_speed_hz = float(self._settings['effect_speed_hz'])
        compat_message.gpio_pin = int(self._settings['gpio_pin'])
        compat_message.pixel_order = str(self._settings['pixel_order'])
        compat_message.backend = self._backend_name
        compat_message.status_message = (
            self._backend_error
            if self._backend_error
            else f'SPI transport {self._settings["spi_bus"]}.{self._settings["spi_device"]} active'
        )
        compat_message.transport = str(self._settings['led_transport'])
        compat_message.spi_bus = int(self._settings['spi_bus'])
        compat_message.spi_device = int(self._settings['spi_device'])
        primary = parse_color_text(str(self._settings['primary_color']))
        secondary = parse_color_text(str(self._settings['secondary_color']))
        compat_message.red = primary[0]
        compat_message.green = primary[1]
        compat_message.blue = primary[2]
        compat_message.secondary_red = secondary[0]
        compat_message.secondary_green = secondary[1]
        compat_message.secondary_blue = secondary[2]
        compat_message.preview_colors = [pack_color(color) for color in self._last_pixels]

        for index, color in enumerate(self._last_pixels):
            state = LEDState()
            state.index = index
            state.r = clamp_byte(color[0])
            state.g = clamp_byte(color[1])
            state.b = clamp_byte(color[2])
            native_message.leds.append(state)

        self._native_state_publisher.publish(native_message)
        self._compat_state_publisher.publish(compat_message)

    def _parameter(self, name: str, value: Any) -> RosParameter:
        if isinstance(value, bool):
            return RosParameter(str(name), RosParameter.Type.BOOL, value)
        if isinstance(value, int) and not isinstance(value, bool):
            return RosParameter(str(name), RosParameter.Type.INTEGER, value)
        if isinstance(value, float):
            return RosParameter(str(name), RosParameter.Type.DOUBLE, value)
        return RosParameter(str(name), RosParameter.Type.STRING, str(value))

    def destroy_node(self) -> bool:
        try:
            if self._strip is not None:
                self._strip.clear()
        except Exception:
            pass
        try:
            if self._driver is not None:
                self._driver.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LedStripNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
