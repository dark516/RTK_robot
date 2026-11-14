"""ROS-independent confidence and distance-estimation logic."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from statistics import median
from typing import Deque, Dict, Mapping, Optional


VALID_AXES = {"height", "width", "mean"}


@dataclass(frozen=True)
class SignSpec:
    """Physical sign dimension used by the pinhole-camera calculation."""

    dimension_m: float
    axis: str

    def validate(self, class_name: str) -> None:
        if self.dimension_m <= 0.0:
            raise ValueError(
                f"signs.{class_name}.dimension_m must be greater than zero"
            )
        if self.axis not in VALID_AXES:
            allowed = ", ".join(sorted(VALID_AXES))
            raise ValueError(
                f"signs.{class_name}.axis must be one of: {allowed}"
            )


class DistanceEstimator:
    """Estimate and median-filter sign distance using bounding-box dimensions."""

    def __init__(
        self,
        focal_length_px: float,
        parking_distance_scale: float,
        filter_size: int,
        sign_specs: Mapping[str, SignSpec],
    ) -> None:
        if focal_length_px <= 0.0:
            raise ValueError("focal_length_px must be greater than zero")
        if parking_distance_scale <= 0.0:
            raise ValueError("parking_distance_scale must be greater than zero")
        if filter_size < 1:
            raise ValueError("filter_size must be at least 1")

        self.focal_length_px = float(focal_length_px)
        self.parking_distance_scale = float(parking_distance_scale)
        self.filter_size = int(filter_size)
        self.sign_specs: Dict[str, SignSpec] = dict(sign_specs)

        for class_name, spec in self.sign_specs.items():
            spec.validate(class_name)

        self._distance_history: Dict[str, Deque[float]] = defaultdict(
            lambda: deque(maxlen=self.filter_size)
        )

    def estimate_raw_m(
        self,
        class_name: str,
        box_width_px: float,
        box_height_px: float,
    ) -> Optional[float]:
        """Return unfiltered distance in metres, or None for an unknown class."""
        spec = self.sign_specs.get(class_name)
        if spec is None:
            return None

        if spec.axis == "height":
            pixel_dimension = box_height_px
        elif spec.axis == "width":
            pixel_dimension = box_width_px
        else:
            pixel_dimension = (box_width_px + box_height_px) / 2.0

        if pixel_dimension <= 0.0:
            return None

        distance_m = (
            spec.dimension_m * self.focal_length_px / float(pixel_dimension)
        )

        if class_name == "parking":
            distance_m *= self.parking_distance_scale

        return distance_m

    def estimate_filtered_cm(
        self,
        class_name: str,
        box_width_px: float,
        box_height_px: float,
    ) -> Optional[float]:
        """Return median-filtered distance in centimetres."""
        raw_distance_m = self.estimate_raw_m(
            class_name,
            box_width_px,
            box_height_px,
        )
        if raw_distance_m is None:
            return None

        history = self._distance_history[class_name]
        history.append(raw_distance_m)
        return float(median(history) * 100.0)


def required_confidence(
    class_name: str,
    default_confidence: float,
    bus_stop_confidence: float,
    parking_confidence: float,
) -> float:
    """Return the class-specific acceptance threshold from the original script."""
    if class_name == "bus_stop":
        return bus_stop_confidence
    if class_name == "parking":
        return parking_confidence
    return default_confidence
