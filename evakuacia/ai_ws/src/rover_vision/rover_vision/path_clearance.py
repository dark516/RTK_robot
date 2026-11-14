from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


MODEL_VERSION = 1


@dataclass(frozen=True, slots=True)
class ClearanceMetrics:
    blocked: bool
    anomaly_fraction: float
    largest_component_fraction: float
    max_width_fraction: float


class TemporalDecision:
    def __init__(self, window_size: int = 5, blocked_votes: int = 3, clear_votes: int = 4):
        if window_size <= 0:
            raise ValueError('window_size must be positive')
        self._values: deque[bool] = deque(maxlen=window_size)
        self._blocked_votes = int(blocked_votes)
        self._clear_votes = int(clear_votes)

    def update(self, blocked: bool) -> str:
        self._values.append(bool(blocked))
        blocked_count = sum(self._values)
        clear_count = len(self._values) - blocked_count
        if blocked_count >= self._blocked_votes:
            return 'blocked'
        if clear_count >= self._clear_votes:
            return 'clear'
        return 'unknown'


class RoadAppearanceModel:
    def __init__(
        self,
        roi_points: Iterable[float],
        warp_size: tuple[int, int] = (240, 360),
        color_clusters: int = 6,
        blocked_fraction_threshold: float = 0.10,
        component_fraction_threshold: float = 0.025,
        width_fraction_threshold: float = 0.55,
        use_edge_anomalies: bool = False,
    ) -> None:
        points = np.asarray(list(roi_points), dtype=np.float32)
        if points.shape != (8,):
            raise ValueError('roi_points must contain four normalized x,y pairs')
        if np.any(points < 0.0) or np.any(points > 1.0):
            raise ValueError('roi_points must be normalized to [0, 1]')
        self.roi_points = points.reshape(4, 2)
        self.warp_width, self.warp_height = map(int, warp_size)
        self.color_clusters = int(color_clusters)
        self.blocked_fraction_threshold = float(blocked_fraction_threshold)
        self.component_fraction_threshold = float(component_fraction_threshold)
        self.width_fraction_threshold = float(width_fraction_threshold)
        self.use_edge_anomalies = bool(use_edge_anomalies)
        self.centers: np.ndarray | None = None
        self.color_threshold: float | None = None
        self.gradient_threshold: float | None = None
        self.reference_pixels: np.ndarray | None = None
        self.clear_cell_count = 0

    @property
    def calibrated(self) -> bool:
        return (
            self.centers is not None
            and self.color_threshold is not None
            and self.gradient_threshold is not None
        )

    def source_points(self, frame_shape: tuple[int, ...]) -> np.ndarray:
        height, width = frame_shape[:2]
        scale = np.array([width - 1, height - 1], dtype=np.float32)
        return self.roi_points * scale

    def warp(self, frame: np.ndarray) -> np.ndarray:
        source = self.source_points(frame.shape)
        destination = np.array(
            [
                [0, 0],
                [self.warp_width - 1, 0],
                [self.warp_width - 1, self.warp_height - 1],
                [0, self.warp_height - 1],
            ],
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(source, destination)
        return cv2.warpPerspective(
            frame,
            transform,
            (self.warp_width, self.warp_height),
            flags=cv2.INTER_LINEAR,
        )

    @staticmethod
    def _gradient(image: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        dx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        dy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        return cv2.magnitude(dx, dy)

    @staticmethod
    def _sample_pixels(pixels: np.ndarray, limit: int) -> np.ndarray:
        if len(pixels) <= limit:
            return pixels
        indices = np.linspace(0, len(pixels) - 1, limit, dtype=np.int64)
        return pixels[indices]

    def fit(self, frames: Iterable[np.ndarray], append: bool = False) -> None:
        warped = [self.warp(frame) for frame in frames]
        if not warped:
            raise ValueError('At least one calibration frame is required')

        new_pixels = np.concatenate(
            [cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).reshape(-1, 3) for frame in warped]
        ).astype(np.float32)
        new_pixels = self._sample_pixels(new_pixels, 80_000)
        if append and self.calibrated:
            if self.reference_pixels is not None:
                previous = self.reference_pixels
            else:
                # Models saved before the clear-cell library existed contain
                # only centroids. Give each old centroid enough weight to keep
                # the previously learned palette during the first append.
                previous = np.repeat(self.centers, 3_000, axis=0)
            lab_pixels = np.concatenate([previous, new_pixels]).astype(np.float32)
            self.clear_cell_count = max(1, self.clear_cell_count) + 1
        else:
            lab_pixels = new_pixels
            self.clear_cell_count = 1
        lab_pixels = self._sample_pixels(lab_pixels, 160_000)
        self.reference_pixels = self._sample_pixels(lab_pixels, 80_000).copy()

        self.color_clusters = min(
            12,
            max(self.color_clusters, 4 + self.clear_cell_count * 2),
        )

        cv2.setRNGSeed(42)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.2)
        _compactness, _labels, centers = cv2.kmeans(
            lab_pixels,
            self.color_clusters,
            None,
            criteria,
            5,
            cv2.KMEANS_PP_CENTERS,
        )
        self.centers = centers.astype(np.float32)

        color_distances = self._color_distances(lab_pixels)
        gradients = np.concatenate([self._gradient(frame).reshape(-1) for frame in warped])
        self.color_threshold = max(14.0, float(np.percentile(color_distances, 99.0)) + 4.0)
        self.gradient_threshold = max(55.0, float(np.percentile(gradients, 99.5)) + 12.0)

    def _color_distances(self, lab_pixels: np.ndarray) -> np.ndarray:
        if self.centers is None:
            raise RuntimeError('Model is not calibrated')
        differences = lab_pixels[:, None, :] - self.centers[None, :, :]
        return np.sqrt(np.sum(differences * differences, axis=2)).min(axis=1)

    @staticmethod
    def _road_marking_mask(image: np.ndarray) -> np.ndarray:
        """Return white/yellow painted road areas, including anti-aliased edges."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hue, saturation, value = cv2.split(hsv)
        white = (saturation <= 55) & (value >= 165)
        yellow = (
            (hue >= 20)
            & (hue <= 40)
            & (saturation >= 65)
            & (value >= 120)
        )
        mask = np.logical_or(white, yellow).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        return cv2.dilate(mask, np.ones((11, 11), np.uint8))

    def analyze(self, frame: np.ndarray) -> tuple[ClearanceMetrics, np.ndarray, np.ndarray]:
        if not self.calibrated:
            raise RuntimeError('Model is not calibrated')
        warped = self.warp(frame)
        lab = cv2.cvtColor(warped, cv2.COLOR_BGR2LAB).reshape(-1, 3).astype(np.float32)
        color_distance = self._color_distances(lab).reshape(
            self.warp_height, self.warp_width
        )
        gradient = self._gradient(warped)

        color_mask = color_distance > float(self.color_threshold)
        if self.use_edge_anomalies:
            edge_mask = gradient > float(self.gradient_threshold)
            edge_mask = cv2.dilate(
                edge_mask.astype(np.uint8), np.ones((7, 7), np.uint8)
            ) > 0
        else:
            edge_mask = np.zeros(color_mask.shape, dtype=bool)
        mask = np.logical_or(color_mask, edge_mask).astype(np.uint8) * 255

        # Painted navigation arrows, lane lines and crossings are traversable.
        # Suppress both their filled area and the high-gradient anti-aliased rim.
        marking_mask = self._road_marking_mask(warped)
        mask[marking_mask > 0] = 0

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
        margin = max(2, int(self.warp_width * 0.02))
        mask[:, :margin] = 0
        mask[:, -margin:] = 0

        active_area = float(mask.size)
        anomaly_fraction = float(np.count_nonzero(mask)) / active_area
        largest_fraction = 0.0
        max_width_fraction = 0.0
        count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(mask)
        for index in range(1, count):
            _x, _y, width, height, area = stats[index]
            component_fraction = float(area) / active_area
            height_fraction = float(height) / float(self.warp_height)
            # A floor-panel seam can span the whole corridor but is only a few
            # pixels tall after the bird's-eye warp. Do not treat such a thin
            # line as a blocking object based on width or component area.
            if height_fraction >= 0.10:
                largest_fraction = max(largest_fraction, component_fraction)
            if component_fraction >= 0.006 and height_fraction >= 0.10:
                max_width_fraction = max(
                    max_width_fraction,
                    float(width) / float(self.warp_width),
                )

        blocked = (
            anomaly_fraction >= self.blocked_fraction_threshold
            or largest_fraction >= self.component_fraction_threshold
            or max_width_fraction >= self.width_fraction_threshold
        )
        metrics = ClearanceMetrics(
            blocked=blocked,
            anomaly_fraction=anomaly_fraction,
            largest_component_fraction=largest_fraction,
            max_width_fraction=max_width_fraction,
        )
        return metrics, warped, mask

    def save(self, path: str | Path) -> None:
        if not self.calibrated:
            raise RuntimeError('Cannot save an uncalibrated model')
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + '.tmp')
        with temporary.open('wb') as handle:
            np.savez_compressed(
                handle,
                version=np.array([MODEL_VERSION], dtype=np.int32),
                roi_points=self.roi_points,
                warp_size=np.array([self.warp_width, self.warp_height]),
                centers=self.centers,
                color_threshold=np.array([self.color_threshold]),
                gradient_threshold=np.array([self.gradient_threshold]),
                decision_thresholds=np.array(
                    [
                        self.blocked_fraction_threshold,
                        self.component_fraction_threshold,
                        self.width_fraction_threshold,
                    ]
                ),
                use_edge_anomalies=np.array([self.use_edge_anomalies]),
                reference_pixels=self.reference_pixels,
                clear_cell_count=np.array([self.clear_cell_count], dtype=np.int32),
            )
        temporary.replace(path)

    @classmethod
    def load(cls, path: str | Path) -> 'RoadAppearanceModel':
        with np.load(Path(path).expanduser()) as data:
            version = int(data['version'][0])
            if version != MODEL_VERSION:
                raise ValueError(f'Unsupported model version: {version}')
            width, height = map(int, data['warp_size'])
            thresholds = data['decision_thresholds']
            model = cls(
                data['roi_points'].reshape(-1),
                warp_size=(width, height),
                color_clusters=int(data['centers'].shape[0]),
                blocked_fraction_threshold=float(thresholds[0]),
                component_fraction_threshold=float(thresholds[1]),
                width_fraction_threshold=float(thresholds[2]),
                use_edge_anomalies=(
                    bool(data['use_edge_anomalies'][0])
                    if 'use_edge_anomalies' in data.files
                    else True
                ),
            )
            model.centers = data['centers'].astype(np.float32)
            model.color_threshold = float(data['color_threshold'][0])
            model.gradient_threshold = float(data['gradient_threshold'][0])
            model.reference_pixels = (
                data['reference_pixels'].astype(np.float32)
                if 'reference_pixels' in data.files
                else None
            )
            model.clear_cell_count = (
                int(data['clear_cell_count'][0])
                if 'clear_cell_count' in data.files
                else 1
            )
            return model
