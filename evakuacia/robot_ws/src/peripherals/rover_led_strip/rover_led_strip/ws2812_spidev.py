from __future__ import annotations

import time

import spidev


_SPI_SPEED_HZ = 2_400_000
_BIT0 = 0x80
_BIT1 = 0xF8
_RESET_TIME_S = 80e-6

_BYTE_TO_SPI: list[bytes] = [b''] * 256
for _value in range(256):
    _BYTE_TO_SPI[_value] = bytes(
        _BIT1 if (_value >> _bit) & 1 else _BIT0 for _bit in range(7, -1, -1)
    )


class Color:
    __slots__ = ('r', 'g', 'b')

    def __init__(self, r: int = 0, g: int = 0, b: int = 0) -> None:
        self.r = int(r)
        self.g = int(g)
        self.b = int(b)


class _WS2812Strip:
    def __init__(
        self,
        spi: spidev.SpiDev,
        led_count: int,
        chunk_size: int,
        speed_hz: int,
    ) -> None:
        self._spi = spi
        self._n = int(led_count)
        self._chunk = int(chunk_size)
        self._speed_hz = int(speed_hz)
        self._brightness = 1.0
        self._pixels: list[Color] = [Color() for _ in range(self._n)]

    def set_brightness(self, value: float) -> None:
        self._brightness = max(0.0, min(1.0, float(value)))

    def set_pixel_color(self, index: int, color: Color) -> None:
        if 0 <= int(index) < self._n:
            self._pixels[int(index)] = Color(color.r, color.g, color.b)

    def set_all_pixels(self, color: Color) -> None:
        for index in range(self._n):
            self._pixels[index] = Color(color.r, color.g, color.b)

    def clear(self) -> None:
        self.set_all_pixels(Color(0, 0, 0))
        self.show()

    def show(self) -> None:
        brightness = self._brightness
        buffer = bytearray(self._n * 24)
        offset = 0
        for pixel in self._pixels:
            red = min(255, int(pixel.r * brightness))
            green = min(255, int(pixel.g * brightness))
            blue = min(255, int(pixel.b * brightness))
            for encoded in (_BYTE_TO_SPI[green], _BYTE_TO_SPI[red], _BYTE_TO_SPI[blue]):
                buffer[offset : offset + 8] = encoded
                offset += 8
        self._write(buffer)
        time.sleep(_RESET_TIME_S)

    def _write(self, buffer: bytearray) -> None:
        view = memoryview(buffer)
        offset = 0
        total = len(buffer)
        while offset < total:
            end = min(offset + self._chunk, total)
            self._spi.xfer2(list(view[offset:end]), self._speed_hz)
            offset = end


class WS2812SpiDriver:
    def __init__(
        self,
        spi_bus: int = 1,
        spi_device: int = 0,
        led_count: int = 1,
        spi_chunk_size: int = 4096,
        spi_speed_hz: int = _SPI_SPEED_HZ,
    ) -> None:
        self._dev = spidev.SpiDev()
        self._dev.open(int(spi_bus), int(spi_device))
        self._dev.max_speed_hz = int(spi_speed_hz)
        self._dev.mode = 0b00
        self._dev.bits_per_word = 8
        self._dev.lsbfirst = False
        self._strip = _WS2812Strip(
            self._dev,
            int(led_count),
            int(spi_chunk_size),
            int(spi_speed_hz),
        )

    def get_strip(self) -> _WS2812Strip:
        return self._strip

    def close(self) -> None:
        self._dev.close()
