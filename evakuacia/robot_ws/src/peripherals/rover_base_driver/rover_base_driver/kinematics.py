from __future__ import annotations

from typing import Iterable

WheelTuple = tuple[float, float, float, float]


def inverse_mecanum(
    vx: float, vy: float, wz: float, wheelbase: float, track_width: float
) -> WheelTuple:
    k = (wheelbase + track_width) / 2.0
    return (
        vx - vy - k * wz,
        vx + vy + k * wz,
        vx + vy - k * wz,
        vx - vy + k * wz,
    )


def scale_wheels(values: Iterable[float], limit: float) -> WheelTuple:
    wheels = tuple(float(value) for value in values)
    if len(wheels) != 4:
        raise ValueError('Four wheel values are required')
    peak = max(abs(value) for value in wheels)
    if peak <= limit:
        return wheels  # type: ignore[return-value]
    factor = limit / peak
    return tuple(value * factor for value in wheels)  # type: ignore[return-value]
