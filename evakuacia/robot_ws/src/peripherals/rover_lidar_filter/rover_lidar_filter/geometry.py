from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarTransform:
    """Rigid 2D transform from the scan frame into the rover base frame."""

    x: float
    y: float
    yaw: float

    def apply(self, point_x: float, point_y: float) -> tuple[float, float]:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            self.x + cos_yaw * point_x - sin_yaw * point_y,
            self.y + sin_yaw * point_x + cos_yaw * point_y,
        )


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw component of a normalized or non-normalized quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def point_inside_rectangle(
    point_x: float,
    point_y: float,
    *,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    padding: float = 0.0,
) -> bool:
    """Return True when a point lies inside an expanded axis-aligned rectangle."""
    return (
        min_x - padding <= point_x <= max_x + padding
        and min_y - padding <= point_y <= max_y + padding
    )


def filter_ranges_inside_rectangle(
    ranges,
    *,
    angle_min: float,
    angle_increment: float,
    transform: PlanarTransform,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    padding: float = 0.0,
) -> tuple[list[float], list[int]]:
    """Replace endpoints inside the rectangle with infinity and return their indices."""
    filtered = [float(value) for value in ranges]
    removed_indices: list[int] = []
    angle = float(angle_min)
    for index, value in enumerate(filtered):
        if math.isfinite(value) and value > 0.0:
            scan_x = value * math.cos(angle)
            scan_y = value * math.sin(angle)
            base_x, base_y = transform.apply(scan_x, scan_y)
            if point_inside_rectangle(
                base_x,
                base_y,
                min_x=min_x,
                max_x=max_x,
                min_y=min_y,
                max_y=max_y,
                padding=padding,
            ):
                filtered[index] = math.inf
                removed_indices.append(index)
        angle += float(angle_increment)
    return filtered, removed_indices
