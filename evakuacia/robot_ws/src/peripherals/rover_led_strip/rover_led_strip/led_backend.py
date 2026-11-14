from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from rover_led_strip.ws2812_spidev import Color, WS2812SpiDriver


class BackendUnavailableError(RuntimeError):
    """Raised when the SPI LED backend cannot be created."""


@dataclass
class BackendInfo:
    name: str
    connected: bool
    status_message: str


class LedStripBackend:
    def __init__(self, spi_bus: int, spi_device: int, led_count: int) -> None:
        try:
            self._driver = WS2812SpiDriver(
                spi_bus=int(spi_bus),
                spi_device=int(spi_device),
                led_count=int(led_count),
            )
        except Exception as exc:  # pragma: no cover - hardware-specific path
            raise BackendUnavailableError(str(exc)) from exc

        self._strip = self._driver.get_strip()
        self.info = BackendInfo(
            name=f'ws2812-spidev:{int(spi_bus)}.{int(spi_device)}',
            connected=True,
            status_message='LED strip backend is ready via Linux spidev.',
        )

    def show(
        self,
        colors: Iterable[tuple[int, int, int]],
        *,
        brightness: float,
    ) -> None:
        self._strip.set_brightness(max(0.0, min(1.0, float(brightness))))
        for index, color in enumerate(colors):
            self._strip.set_pixel_color(index, Color(*tuple(int(channel) for channel in color[:3])))
        self._strip.show()

    def clear(self) -> None:
        self._strip.clear()
