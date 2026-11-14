from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


SUPPORTED_MODEL_FORMATS = {'yolov5', 'yolov8', 'opencv_ssd_tf'}


@dataclass(slots=True)
class ModelManifest:
    identifier: str
    display_name: str
    description: str
    manifest_path: Path
    model_path: Path | None
    config_path: Path | None
    task: str
    model_format: str
    input_width: int
    input_height: int
    swap_rb: bool
    confidence_threshold: float
    nms_threshold: float
    labels: list[str]
    valid: bool
    error: str | None


def default_workspace_root(package_name: str = 'rover_vision') -> Path:
    try:
        share = Path(get_package_share_directory(package_name))
        parents = share.parents
        if len(parents) >= 4 and parents[2].name == 'install':
            return parents[3]
        parts = share.parts
        if 'src' in parts:
            return Path(*parts[:parts.index('src')])
    except Exception:
        pass
    return Path.cwd()


def package_share_directory(package_name: str = 'rover_vision') -> Path | None:
    if get_package_share_directory is None:
        return None
    try:
        return Path(get_package_share_directory(package_name)).resolve()
    except Exception:
        return None


def source_package_root(package_name: str = 'rover_vision') -> Path:
    module_path = Path(__file__).resolve()
    for parent in module_path.parents:
        if parent.name == package_name and (parent / 'package.xml').exists():
            return parent
    return module_path.parents[1]


def resolve_package_uri(value: str) -> Path | None:
    if not value.startswith('package://'):
        return None
    remainder = value[len('package://'):].strip('/')
    if not remainder:
        return None
    package_name, _, relative = remainder.partition('/')
    share = package_share_directory(package_name)
    if share is not None:
        return (share / relative).resolve()
    if package_name == 'rover_vision':
        return (source_package_root(package_name) / relative).resolve()
    return None


def resolve_models_directory(
    value: str | Path | None,
    package_name: str = 'rover_vision',
) -> Path:
    text = str(value or 'models').strip()
    package_uri = resolve_package_uri(text)
    if package_uri is not None:
        return package_uri

    path = Path(text).expanduser()
    if path.is_absolute():
        return path.resolve()

    candidates: list[Path] = []
    share = package_share_directory(package_name)
    if share is not None:
        candidates.append((share / path).resolve())
    candidates.append((source_package_root(package_name) / path).resolve())
    candidates.append((default_workspace_root(package_name) / path).resolve())

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def _parse_input_size(value: Any) -> tuple[int, int]:
    if isinstance(value, int):
        return max(1, value), max(1, value)
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return max(1, int(value[0])), max(1, int(value[1]))
    if isinstance(value, dict):
        width = int(value.get('width', value.get('w', 640)))
        height = int(value.get('height', value.get('h', 640)))
        return max(1, width), max(1, height)
    return 640, 640


def _load_labels(raw: dict[str, Any], manifest_dir: Path) -> list[str]:
    labels = raw.get('labels', [])
    if isinstance(labels, list):
        return [str(item).strip() for item in labels if str(item).strip()]
    labels_file = raw.get('labels_file')
    if labels_file:
        path = (manifest_dir / str(labels_file)).resolve()
        try:
            return [
                line.strip()
                for line in path.read_text(encoding='utf-8').splitlines()
                if line.strip()
            ]
        except OSError:
            return []
    return []


def load_model_manifest(path: str | Path) -> ModelManifest:
    manifest_path = Path(path).expanduser().resolve()
    raw: dict[str, Any]
    try:
        raw_value = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
        raw = raw_value if isinstance(raw_value, dict) else {}
    except Exception as exc:
        return ModelManifest(
            identifier=manifest_path.stem,
            display_name=manifest_path.stem,
            description='',
            manifest_path=manifest_path,
            model_path=None,
            config_path=None,
            task='detection',
            model_format='yolov8',
            input_width=640,
            input_height=640,
            swap_rb=True,
            confidence_threshold=0.25,
            nms_threshold=0.45,
            labels=[],
            valid=False,
            error=str(exc),
        )

    identifier = str(raw.get('id') or manifest_path.stem).strip() or manifest_path.stem
    display_name = str(raw.get('name') or identifier).strip() or identifier
    description = str(raw.get('description') or '').strip()
    task = str(raw.get('task') or 'detection').strip().lower() or 'detection'
    model_format = str(raw.get('format') or 'yolov8').strip().lower() or 'yolov8'
    input_width, input_height = _parse_input_size(raw.get('input_size'))
    model_value = str(raw.get('model') or '').strip()
    model_path = (manifest_path.parent / model_value).resolve() if model_value else None
    config_value = str(raw.get('config') or '').strip()
    config_path = (manifest_path.parent / config_value).resolve() if config_value else None
    labels = _load_labels(raw, manifest_path.parent)
    confidence_threshold = float(raw.get('confidence_threshold', 0.25))
    nms_threshold = float(raw.get('nms_threshold', 0.45))
    swap_rb = bool(raw.get('swap_rb', True))

    error = None
    valid = True
    if task != 'detection':
        valid = False
        error = f'Unsupported task: {task}'
    elif model_format not in SUPPORTED_MODEL_FORMATS:
        valid = False
        error = f'Unsupported format: {model_format}'
    elif model_path is None:
        valid = False
        error = 'Model file is not specified'
    elif not model_path.exists():
        valid = False
        error = f'Model file is missing: {model_path.name}'
    elif model_format == 'opencv_ssd_tf' and config_path is None:
        valid = False
        error = 'Config file is not specified'
    elif model_format == 'opencv_ssd_tf' and config_path is not None and not config_path.exists():
        valid = False
        error = f'Config file is missing: {config_path.name}'

    return ModelManifest(
        identifier=identifier,
        display_name=display_name,
        description=description,
        manifest_path=manifest_path,
        model_path=model_path,
        config_path=config_path,
        task=task,
        model_format=model_format,
        input_width=input_width,
        input_height=input_height,
        swap_rb=swap_rb,
        confidence_threshold=confidence_threshold,
        nms_threshold=nms_threshold,
        labels=labels,
        valid=valid,
        error=error,
    )


def discover_model_manifests(directory: str | Path) -> list[ModelManifest]:
    root = Path(directory).expanduser().resolve()
    if not root.exists() or not root.is_dir():
        return []
    manifests: list[ModelManifest] = []
    for path in sorted(root.iterdir()):
        if path.suffix.lower() not in {'.yaml', '.yml'}:
            continue
        manifests.append(load_model_manifest(path))
    return manifests


def manifest_to_dict(manifest: ModelManifest) -> dict[str, Any]:
    return {
        'id': manifest.identifier,
        'name': manifest.display_name,
        'description': manifest.description,
        'manifest_path': str(manifest.manifest_path),
        'model_path': str(manifest.model_path) if manifest.model_path else '',
        'config_path': str(manifest.config_path) if manifest.config_path else '',
        'task': manifest.task,
        'format': manifest.model_format,
        'input_size': [manifest.input_width, manifest.input_height],
        'swap_rb': manifest.swap_rb,
        'confidence_threshold': manifest.confidence_threshold,
        'nms_threshold': manifest.nms_threshold,
        'labels_count': len(manifest.labels),
        'labels_preview': manifest.labels[:10],
        'valid': manifest.valid,
        'error': manifest.error,
    }
