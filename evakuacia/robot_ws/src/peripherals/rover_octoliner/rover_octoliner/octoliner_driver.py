from __future__ import annotations

import math
import time


GPIO_EXPANDER_DEFAULT_I2C_ADDRESS = 0x2A
GPIO_EXPANDER_RESET = 0x01
GPIO_EXPANDER_CHANGE_I2C_ADDR = 0x02
GPIO_EXPANDER_SAVE_I2C_ADDR = 0x03
GPIO_EXPANDER_PORT_MODE_INPUT = 0x04
GPIO_EXPANDER_PORT_MODE_PULLUP = 0x05
GPIO_EXPANDER_PORT_MODE_PULLDOWN = 0x06
GPIO_EXPANDER_PORT_MODE_OUTPUT = 0x07
GPIO_EXPANDER_DIGITAL_READ = 0x08
GPIO_EXPANDER_DIGITAL_WRITE_HIGH = 0x09
GPIO_EXPANDER_DIGITAL_WRITE_LOW = 0x0A
GPIO_EXPANDER_ANALOG_WRITE = 0x0B
GPIO_EXPANDER_ANALOG_READ = 0x0C
GPIO_EXPANDER_PWM_FREQ = 0x0D

INPUT = 0
OUTPUT = 1
INPUT_PULLUP = 2
INPUT_PULLDOWN = 3

MIN_SENSITIVITY = 0.47
BLACK_THRESHOLD = 0.39

_PATTERN_TO_LINE = {
    0b00011000: 0.0,
    0b00010000: 0.25,
    0b00111000: 0.25,
    0b00001000: -0.25,
    0b00011100: -0.25,
    0b00110000: 0.375,
    0b00001100: -0.375,
    0b00100000: 0.5,
    0b01110000: 0.5,
    0b00000100: -0.5,
    0b00001110: -0.5,
    0b01100000: 0.625,
    0b11100000: 0.625,
    0b00000110: -0.625,
    0b00000111: -0.625,
    0b01000000: 0.75,
    0b11110000: 0.75,
    0b00000010: -0.75,
    0b00001111: -0.75,
    0b11000000: 0.875,
    0b00000011: -0.875,
    0b10000000: 1.0,
    0b00000001: -1.0,
}


def _swap_u16(value: int) -> int:
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)


class _I2cBackend:
    def __init__(self, bus_number: int, address: int) -> None:
        self._address = address
        self._bus = self._open_bus(bus_number)

    def _open_bus(self, bus_number: int):
        import_error = None
        for module_name, class_name in (('smbus2', 'SMBus'), ('smbus', 'SMBus')):
            try:
                module = __import__(module_name, fromlist=[class_name])
                return getattr(module, class_name)(bus_number)
            except ImportError as exc:
                import_error = exc
        raise RuntimeError(
            'Neither "smbus2" nor "smbus" is installed. '
            'Install one of them, for example: sudo apt install python3-smbus'
        ) from import_error

    def write_reg16(self, register: int, value: int) -> None:
        self._bus.write_word_data(self._address, register, value & 0xFFFF)

    def read_reg16(self, register: int) -> int:
        return int(self._bus.read_word_data(self._address, register))

    def write_byte(self, register: int) -> None:
        self._bus.write_byte_data(self._address, register, 0)


class OctolinerDriver:
    def __init__(
        self,
        i2c_address: int = GPIO_EXPANDER_DEFAULT_I2C_ADDRESS,
        i2c_bus: int = 1,
    ) -> None:
        self._i2c = _I2cBackend(i2c_bus, i2c_address)
        self._ir_leds_pin = 9
        self._sense_pin = 0
        self._sensor_pin_map = (4, 5, 6, 8, 7, 3, 2, 1)
        self._previous_value = 0.0
        self._sensitivity = 0.8

        self.pin_mode(self._ir_leds_pin, OUTPUT)
        self.digital_write(self._ir_leds_pin, 1)
        self.pwm_freq(8000)
        self.set_sensitivity(self._sensitivity)

    def pin_mode(self, pin: int, mode: int) -> None:
        data = _swap_u16(1 << pin)
        if mode == INPUT:
            self._i2c.write_reg16(GPIO_EXPANDER_PORT_MODE_INPUT, data)
        elif mode == INPUT_PULLUP:
            self._i2c.write_reg16(GPIO_EXPANDER_PORT_MODE_PULLUP, data)
        elif mode == INPUT_PULLDOWN:
            self._i2c.write_reg16(GPIO_EXPANDER_PORT_MODE_PULLDOWN, data)
        elif mode == OUTPUT:
            self._i2c.write_reg16(GPIO_EXPANDER_PORT_MODE_OUTPUT, data)

    def digital_write(self, pin: int, value: int) -> None:
        data = _swap_u16(1 << pin)
        register = (
            GPIO_EXPANDER_DIGITAL_WRITE_HIGH
            if value
            else GPIO_EXPANDER_DIGITAL_WRITE_LOW
        )
        self._i2c.write_reg16(register, data)

    def analog_write(self, pin: int, value: float) -> None:
        pwm_value = max(0, min(255, int(round(float(value) * 255.0))))
        data = (pin & 0xFF) | ((pwm_value & 0xFF) << 8)
        self._i2c.write_reg16(GPIO_EXPANDER_ANALOG_WRITE, data)

    def pwm_freq(self, frequency_hz: int) -> None:
        self._i2c.write_reg16(GPIO_EXPANDER_PWM_FREQ, _swap_u16(frequency_hz))

    def analog_read(self, sensor_index: int) -> float:
        sensor_index &= 0x07
        sensor_pin = self._sensor_pin_map[sensor_index]
        self._i2c.write_reg16(GPIO_EXPANDER_ANALOG_READ, sensor_pin)
        raw_value = _swap_u16(self._i2c.read_reg16(GPIO_EXPANDER_ANALOG_READ))
        return raw_value / 4095.0

    def analog_read_all(self) -> list[float]:
        return [self.analog_read(index) for index in range(8)]

    def set_sensitivity(self, sensitivity: float) -> None:
        self._sensitivity = max(0.0, min(1.0, float(sensitivity)))
        self.analog_write(self._sense_pin, self._sensitivity)

    def get_sensitivity(self) -> float:
        return self._sensitivity

    def map_analog_to_pattern(self, analog_values: list[float]) -> int:
        if not analog_values:
            return 0
        min_value = min(analog_values)
        max_value = max(analog_values)
        threshold = min_value + (max_value - min_value) / 2.0
        pattern = 0
        for value in analog_values:
            pattern = (pattern << 1) | (0 if value < threshold else 1)
        return pattern

    def map_pattern_to_line(self, pattern: int) -> float:
        return _PATTERN_TO_LINE.get(pattern, float('nan'))

    def digital_read_all(self) -> int:
        return self.map_analog_to_pattern(self.analog_read_all())

    def track_line(self, values: None | list[float] | int = None) -> float:
        if values is None:
            return self.track_line(self.digital_read_all())
        if isinstance(values, list):
            return self.track_line(self.map_analog_to_pattern(values))
        if isinstance(values, int):
            result = self.map_pattern_to_line(values)
            if math.isnan(result):
                result = self._previous_value
            self._previous_value = result
            return result
        raise TypeError(f'Unsupported track_line input: {type(values)!r}')

    def optimize_sensitivity_on_black(self) -> bool:
        backup = self.get_sensitivity()

        self.set_sensitivity(1.0)
        time.sleep(0.2)

        sensitivity = 1.0
        while sensitivity > MIN_SENSITIVITY:
            self.set_sensitivity(sensitivity)
            time.sleep(0.1)
            if self._count_black() == 8:
                break
            sensitivity -= 0.02

        if sensitivity <= MIN_SENSITIVITY:
            self.set_sensitivity(backup)
            return False

        while sensitivity < 1.0:
            self.set_sensitivity(sensitivity)
            time.sleep(0.05)
            if self._count_black() != 8:
                break
            sensitivity += 0.004

        if sensitivity >= 1.0:
            self.set_sensitivity(backup)
            return False

        sensitivity -= 0.02
        self.set_sensitivity(sensitivity)
        return True

    def _count_black(self) -> int:
        return sum(1 for value in self.analog_read_all() if value > BLACK_THRESHOLD)
