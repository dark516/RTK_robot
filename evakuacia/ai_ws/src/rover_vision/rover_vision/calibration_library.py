from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from threading import RLock
from typing import Iterable
from uuid import uuid4

import cv2
import numpy as np


PROFILES = ('normal', 'bridge')


@dataclass(frozen=True, slots=True)
class BridgePose:
    row: int
    column: int
    direction: str

    def as_dict(self) -> dict:
        return {
            'row': self.row,
            'column': self.column,
            'direction': self.direction,
        }


def cardinal_direction(x: float, y: float) -> str:
    if abs(x) >= abs(y):
        return 'east' if x >= 0 else 'west'
    return 'north' if y >= 0 else 'south'


class CalibrationLibrary:
    """Persistent full-frame calibration samples and per-profile ROI settings."""

    VERSION = 1

    def __init__(self, root: str | Path, default_roi: Iterable[float]) -> None:
        self.root = Path(root).expanduser()
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = RLock()
        roi = [float(value) for value in default_roi]
        self._config_path = self.root / 'library.json'
        self._config = self._load_config(roi)
        for profile in PROFILES:
            self.frames_dir(profile).mkdir(parents=True, exist_ok=True)
        self.save_config()

    def _load_config(self, default_roi: list[float]) -> dict:
        if self._config_path.exists():
            with self._config_path.open(encoding='utf-8') as handle:
                loaded = json.load(handle)
            if int(loaded.get('version', 0)) == self.VERSION:
                return loaded
        return {
            'version': self.VERSION,
            'profiles': {
                name: {'roi_points': list(default_roi)} for name in PROFILES
            },
            'bridge_poses': [],
        }

    def save_config(self) -> None:
        with self._lock:
            temporary = self._config_path.with_suffix('.json.tmp')
            with temporary.open('w', encoding='utf-8') as handle:
                json.dump(self._config, handle, ensure_ascii=False, indent=2)
            temporary.replace(self._config_path)

    def frames_dir(self, profile: str) -> Path:
        self._validate_profile(profile)
        return self.root / 'frames' / profile

    def model_path(self, profile: str) -> Path:
        self._validate_profile(profile)
        return self.root / f'{profile}.npz'

    @staticmethod
    def _validate_profile(profile: str) -> None:
        if profile not in PROFILES:
            raise ValueError(f'Unknown profile: {profile}')

    def roi(self, profile: str) -> list[float]:
        self._validate_profile(profile)
        with self._lock:
            return list(self._config['profiles'][profile]['roi_points'])

    def set_roi(self, profile: str, points: Iterable[float]) -> None:
        self._validate_profile(profile)
        values = [float(value) for value in points]
        if len(values) != 8:
            raise ValueError('ROI must contain eight normalized coordinates')
        with self._lock:
            self._config['profiles'][profile]['roi_points'] = values
            self.save_config()

    def add_frame(self, profile: str, frame: np.ndarray) -> dict:
        self._validate_profile(profile)
        identifier = uuid4().hex
        created_at = datetime.now(timezone.utc).isoformat()
        path = self.frames_dir(profile) / f'{identifier}.jpg'
        if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 94]):
            raise OSError(f'Unable to save calibration frame to {path}')
        return {'id': identifier, 'created_at': created_at, 'path': str(path)}

    def remove_frame(self, profile: str, identifier: str) -> bool:
        self._validate_profile(profile)
        if not identifier or any(ch not in '0123456789abcdef' for ch in identifier):
            raise ValueError('Invalid frame id')
        path = self.frames_dir(profile) / f'{identifier}.jpg'
        if not path.exists():
            return False
        path.unlink()
        return True

    def list_frames(self, profile: str) -> list[dict]:
        self._validate_profile(profile)
        result = []
        for path in sorted(self.frames_dir(profile).glob('*.jpg')):
            stat = path.stat()
            result.append({
                'id': path.stem,
                'created_at': datetime.fromtimestamp(
                    stat.st_mtime, timezone.utc
                ).isoformat(),
                'size': stat.st_size,
            })
        return result

    def load_frames(self, profile: str) -> list[np.ndarray]:
        frames = []
        for item in self.list_frames(profile):
            frame = cv2.imread(str(self.frame_path(profile, item['id'])))
            if frame is not None:
                frames.append(frame)
        return frames

    def frame_path(self, profile: str, identifier: str) -> Path:
        self._validate_profile(profile)
        if not identifier or any(ch not in '0123456789abcdef' for ch in identifier):
            raise ValueError('Invalid frame id')
        return self.frames_dir(profile) / f'{identifier}.jpg'

    def bridge_poses(self) -> list[dict]:
        with self._lock:
            return [dict(item) for item in self._config.get('bridge_poses', [])]

    def add_bridge_pose(self, row: int, column: int, direction: str) -> dict:
        pose = BridgePose(int(row), int(column), str(direction)).as_dict()
        with self._lock:
            poses = self._config.setdefault('bridge_poses', [])
            if pose not in poses:
                poses.append(pose)
                self.save_config()
        return pose

    def remove_bridge_pose(self, row: int, column: int, direction: str) -> bool:
        target = BridgePose(int(row), int(column), str(direction)).as_dict()
        with self._lock:
            poses = self._config.setdefault('bridge_poses', [])
            before = len(poses)
            poses[:] = [item for item in poses if item != target]
            changed = len(poses) != before
            if changed:
                self.save_config()
            return changed

    def profile_for_pose(self, row: int, column: int, direction: str) -> str:
        target = BridgePose(int(row), int(column), str(direction)).as_dict()
        return 'bridge' if target in self.bridge_poses() else 'normal'
