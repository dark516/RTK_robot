"""Allowlisted discovery and decoding of Nav2 occupancy map files."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any, Sequence

import cv2
import yaml


class MapCatalogError(ValueError):
    """Raised when a selected map cannot be loaded safely."""


def _rle(values: list[int]) -> list[list[int]]:
    if not values:
        return []
    result: list[list[int]] = []
    current, count = values[0], 1
    for value in values[1:]:
        if value == current:
            count += 1
        else:
            result.append([current, count])
            current, count = value, 1
    result.append([current, count])
    return result


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapCatalogError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise MapCatalogError(f"{label} must be finite")
    return result


class MapCatalog:
    """Discover map YAML files under a fixed set of filesystem roots."""

    def __init__(self, roots: Sequence[Path | str]):
        self._roots = tuple(Path(root).resolve() for root in roots)

    def list_maps(self) -> list[dict[str, Any]]:
        entries: list[dict[str, Any]] = []
        seen_names: set[str] = set()
        for root_index, root in enumerate(self._roots):
            if not root.is_dir():
                continue
            for yaml_path in sorted(root.glob("*.yaml")):
                resolved = yaml_path.resolve()
                if not resolved.is_file() or not resolved.is_relative_to(root):
                    continue
                name = resolved.stem
                if name in seen_names:
                    continue
                seen_names.add(name)
                map_id = hashlib.sha256(str(resolved).encode("utf-8")).hexdigest()[:20]
                entries.append({
                    "id": map_id,
                    "name": name,
                    "path": str(resolved),
                    "root_index": root_index,
                })
        return entries

    def preview(self, map_id: str) -> dict[str, Any]:
        entry = next((item for item in self.list_maps() if item["id"] == map_id), None)
        if entry is None:
            raise MapCatalogError("unknown map")
        root = self._roots[int(entry["root_index"])]
        yaml_path = Path(entry["path"]).resolve()
        if not yaml_path.is_relative_to(root):
            raise MapCatalogError("map escaped allowlisted root")
        return self._load_preview(yaml_path, root, entry)

    def _load_preview(
        self,
        yaml_path: Path,
        root: Path,
        entry: dict[str, Any],
    ) -> dict[str, Any]:
        try:
            metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            raise MapCatalogError(f"cannot read map YAML: {exc}") from exc
        if not isinstance(metadata, dict):
            raise MapCatalogError("map YAML must contain an object")

        image_value = metadata.get("image")
        if not isinstance(image_value, str) or not image_value.strip():
            raise MapCatalogError("image must be a non-empty path")
        image_path = (yaml_path.parent / image_value).resolve()
        if not image_path.is_relative_to(root):
            raise MapCatalogError("image escaped allowlisted root")
        if image_path.suffix.lower() not in {".pgm", ".png"}:
            raise MapCatalogError("unsupported map image format")
        if not image_path.is_file():
            raise MapCatalogError("map image does not exist")

        resolution = _finite(metadata.get("resolution"), "resolution")
        if resolution <= 0.0:
            raise MapCatalogError("resolution must be positive")
        origin = metadata.get("origin")
        if not isinstance(origin, list) or len(origin) != 3:
            raise MapCatalogError("origin must contain [x, y, yaw]")
        origin_values = [_finite(value, f"origin[{index}]") for index, value in enumerate(origin)]
        negate = metadata.get("negate", 0)
        if negate not in (0, 1, False, True):
            raise MapCatalogError("negate must be 0 or 1")
        occupied_threshold = _finite(metadata.get("occupied_thresh"), "occupied_thresh")
        free_threshold = _finite(metadata.get("free_thresh"), "free_thresh")
        if not (0.0 <= free_threshold < occupied_threshold <= 1.0):
            raise MapCatalogError("map thresholds must satisfy 0 <= free < occupied <= 1")

        pixels = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if pixels is None or pixels.ndim != 2 or pixels.size == 0:
            raise MapCatalogError("cannot decode map image")
        height, width = (int(value) for value in pixels.shape)
        values: list[int] = []
        for row in reversed(pixels.tolist()):
            for raw_pixel in row:
                probability = float(raw_pixel) / 255.0 if bool(negate) else (255.0 - float(raw_pixel)) / 255.0
                if probability > occupied_threshold:
                    values.append(100)
                elif probability < free_threshold:
                    values.append(0)
                else:
                    values.append(-1)

        digest = hashlib.sha256()
        digest.update(yaml_path.read_bytes())
        digest.update(image_path.read_bytes())
        modified_s = max(yaml_path.stat().st_mtime, image_path.stat().st_mtime)
        return {
            "frame_id": "map",
            "stamp_s": modified_s,
            "map_load_time_s": modified_s,
            "width": width,
            "height": height,
            "resolution_m": resolution,
            "origin": {
                "x_m": origin_values[0],
                "y_m": origin_values[1],
                "yaw_rad": origin_values[2],
            },
            "rle": _rle(values),
            "source": {
                "kind": "file",
                "id": entry["id"],
                "name": entry["name"],
                "path": entry["path"],
            },
            "fingerprint": digest.hexdigest(),
        }
