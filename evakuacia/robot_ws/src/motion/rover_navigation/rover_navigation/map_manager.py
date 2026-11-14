from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Iterable

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph
import yaml


PACKAGE_NAME = 'rover_navigation'
LABEL_RE = re.compile(r'^[A-Za-z0-9_-]+$')


class MapManagerError(RuntimeError):
    pass


def _is_source_package(path: Path) -> bool:
    package_xml = path / 'package.xml'
    setup_py = path / 'setup.py'
    maps_dir = path / 'maps'
    if (
        not package_xml.is_file()
        or not setup_py.is_file()
        or not maps_dir.is_dir()
    ):
        return False
    try:
        return f'<name>{PACKAGE_NAME}</name>' in package_xml.read_text(
            encoding='utf-8'
        )
    except OSError:
        return False


def find_source_package() -> Path:
    """Find the writable source package in a normal colcon workspace."""
    candidates: list[Path] = []

    configured = os.environ.get('ROVER_NAVIGATION_SOURCE_DIR', '').strip()
    if configured:
        candidates.append(Path(configured).expanduser())

    try:
        share = Path(get_package_share_directory(PACKAGE_NAME))
        installed_package_xml = share / 'package.xml'
        if installed_package_xml.exists():
            # With --symlink-install this resolves directly to the source package.
            candidates.append(installed_package_xml.resolve().parent)
    except Exception:
        pass

    try:
        prefix = Path(get_package_prefix(PACKAGE_NAME))
        workspace_candidates = [prefix.parent, prefix.parent.parent]
        for workspace in workspace_candidates:
            candidates.append(workspace / 'src' / 'motion' / PACKAGE_NAME)
            candidates.append(workspace / 'src' / PACKAGE_NAME)
    except Exception:
        pass

    candidates.append(Path.home() / 'ros2_ws' / 'src' / 'motion' / PACKAGE_NAME)
    candidates.append(Path.home() / 'ros2_ws' / 'src' / PACKAGE_NAME)

    current = Path.cwd().resolve()
    for parent in (current, *current.parents):
        candidates.append(parent / 'src' / 'motion' / PACKAGE_NAME)
        candidates.append(parent / 'src' / PACKAGE_NAME)
        if parent.name == PACKAGE_NAME:
            candidates.append(parent)

    seen: set[Path] = set()
    for candidate in candidates:
        candidate = candidate.expanduser().resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        if _is_source_package(candidate):
            if not os.access(candidate, os.W_OK):
                raise MapManagerError(
                    f'Исходный пакет найден, но недоступен для записи: {candidate}'
                )
            return candidate

    raise MapManagerError(
        'Не удалось найти исходный пакет rover_navigation. '
        'Запустите команду из workspace или задайте '
        'ROVER_NAVIGATION_SOURCE_DIR=~/ros2_ws/src/motion/rover_navigation.'
    )


def map_paths() -> tuple[Path, Path, Path, Path]:
    source_package = find_source_package()
    maps_root = source_package / 'maps'
    current = maps_root / 'current'
    archive = maps_root / 'archive'
    return source_package, maps_root, current, archive


def _validate_label(label: str) -> str:
    label = label.strip()
    if not label or not LABEL_RE.fullmatch(label):
        raise MapManagerError(
            'Имя карты может содержать только латинские буквы, цифры, _ и -.'
        )
    return label


def _run(command: Iterable[str], timeout: float) -> subprocess.CompletedProcess[str]:
    command_list = list(command)
    try:
        result = subprocess.run(
            command_list,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise MapManagerError(
            f'Команда не завершилась за {timeout:.0f} с: '
            + ' '.join(command_list)
        ) from exc

    if result.stdout:
        print(result.stdout.rstrip())

    if result.returncode != 0:
        raise MapManagerError(
            f'Команда завершилась с кодом {result.returncode}: '
            + ' '.join(command_list)
        )
    return result


def _wait_for_topic(node: Node, topic: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        names = {name for name, _types in node.get_topic_names_and_types()}
        if topic in names:
            return
        rclpy.spin_once(node, timeout_sec=0.2)
    raise MapManagerError(
        f'Топик {topic} не найден. Убедитесь, что SLAM Toolbox запущен.'
    )


def _serialize_posegraph(node: Node, prefix: Path, timeout: float) -> None:
    client = node.create_client(
        SerializePoseGraph,
        '/slam_toolbox/serialize_map',
    )
    if not client.wait_for_service(timeout_sec=timeout):
        raise MapManagerError(
            'Сервис /slam_toolbox/serialize_map недоступен. '
            'Карту нужно сохранять при запущенном SLAM Toolbox.'
        )

    request = SerializePoseGraph.Request()
    request.filename = str(prefix)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

    if not future.done():
        raise MapManagerError('Истекло время ожидания сериализации pose graph.')
    if future.exception() is not None:
        raise MapManagerError(
            f'Ошибка сервиса serialize_map: {future.exception()}'
        )

    response = future.result()
    if response is None or int(response.result) != 0:
        result_code = None if response is None else int(response.result)
        raise MapManagerError(
            f'SLAM Toolbox не сохранил pose graph, result={result_code}.'
        )


def _read_map_image(yaml_path: Path) -> Path:
    try:
        content = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise MapManagerError(f'Не удалось прочитать {yaml_path}: {exc}') from exc

    if not isinstance(content, dict) or not content.get('image'):
        raise MapManagerError(f'В {yaml_path} отсутствует поле image.')

    image = Path(str(content['image'])).expanduser()
    if not image.is_absolute():
        image = yaml_path.parent / image
    return image.resolve()


def validate_map(directory: Path, require_posegraph: bool = False) -> None:
    yaml_path = directory / 'map.yaml'
    if not yaml_path.is_file() or yaml_path.stat().st_size == 0:
        raise MapManagerError(f'Не найден корректный файл {yaml_path}.')

    image_path = _read_map_image(yaml_path)
    if not image_path.is_file() or image_path.stat().st_size == 0:
        raise MapManagerError(
            f'Не найдено изображение карты из map.yaml: {image_path}'
        )

    if require_posegraph:
        for suffix in ('.posegraph', '.data'):
            path = directory / f'map{suffix}'
            if not path.is_file() or path.stat().st_size == 0:
                raise MapManagerError(f'Не найден файл pose graph: {path}')


def _metadata_label(directory: Path, default: str = 'map') -> str:
    metadata = directory / 'map_info.json'
    if metadata.is_file():
        try:
            data = json.loads(metadata.read_text(encoding='utf-8'))
            label = str(data.get('label', default))
            if LABEL_RE.fullmatch(label):
                return label
        except (OSError, ValueError, TypeError):
            pass
    return default


def _archive_current(current: Path, archive: Path) -> Path | None:
    if not current.exists() or not any(current.iterdir()):
        if current.exists():
            current.rmdir()
        return None

    archive.mkdir(parents=True, exist_ok=True)
    label = _metadata_label(current, 'map')
    stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
    destination = archive / f'{label}_{stamp}'
    counter = 1
    while destination.exists():
        destination = archive / f'{label}_{stamp}_{counter}'
        counter += 1
    current.rename(destination)
    return destination


def _remove_path(path: Path) -> None:
    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.is_dir():
        shutil.rmtree(path)


def sync_installed_current(source_current: Path) -> tuple[bool, str]:
    try:
        share = Path(get_package_share_directory(PACKAGE_NAME))
    except Exception as exc:
        return False, f'установленная копия пакета не найдена: {exc}'

    installed_current = share / 'maps' / 'current'
    try:
        if installed_current.exists():
            try:
                if installed_current.resolve() == source_current.resolve():
                    return True, 'install использует исходную папку через symlink'
            except OSError:
                pass
            _remove_path(installed_current)
        installed_current.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(source_current, installed_current)
        return True, str(installed_current)
    except OSError as exc:
        return False, str(exc)


def save_map(label: str, occupancy_only: bool, timeout: float) -> None:
    label = _validate_label(label)
    _source_package, maps_root, current, archive = map_paths()
    maps_root.mkdir(parents=True, exist_ok=True)
    archive.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
    staging = maps_root / f'.staging_{label}_{stamp}_{os.getpid()}'
    staging.mkdir(parents=False, exist_ok=False)
    prefix = staging / 'map'

    rclpy.init(args=[])
    node = Node('rover_map_manager')
    archived_path: Path | None = None
    try:
        _wait_for_topic(node, '/map', min(timeout, 10.0))

        print(f'Сохраняю occupancy map в {staging} ...')
        _run(
            [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', str(prefix),
                '--ros-args',
                '-p', f'save_map_timeout:={timeout}',
                '-p', 'map_subscribe_transient_local:=true',
            ],
            timeout=timeout + 10.0,
        )

        if not occupancy_only:
            print('Сохраняю pose graph SLAM Toolbox ...')
            _serialize_posegraph(node, prefix, timeout)

        metadata = {
            'label': label,
            'created_at': datetime.now().astimezone().isoformat(timespec='seconds'),
            'occupancy_map': 'map.yaml',
            'posegraph_saved': not occupancy_only,
        }
        (staging / 'map_info.json').write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + '\n',
            encoding='utf-8',
        )

        validate_map(staging, require_posegraph=not occupancy_only)
        archived_path = _archive_current(current, archive)
        staging.rename(current)

        synced, sync_message = sync_installed_current(current)

        print('\nКарта успешно сохранена.')
        print(f'Текущая карта: {current / "map.yaml"}')
        if archived_path is not None:
            print(f'Предыдущая карта: {archived_path}')
        if synced:
            print(f'Установленная копия обновлена: {sync_message}')
        else:
            print('ВНИМАНИЕ: установленную копию обновить не удалось:')
            print(f'  {sync_message}')
            print('Выполните:')
            print(
                '  cd ~/ros2_ws && colcon build '
                '--packages-select rover_navigation --symlink-install'
            )
    except Exception:
        if staging.exists():
            shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


def list_maps() -> None:
    _source_package, _maps_root, current, archive = map_paths()
    print(f'CURRENT  {current}')
    if (current / 'map.yaml').is_file():
        info = _metadata_label(current, 'map')
        posegraph = (current / 'map.posegraph').is_file()
        print(f'         label={info}, posegraph={"yes" if posegraph else "no"}')
    else:
        print('         карта отсутствует')

    print('\nARCHIVE')
    entries = sorted(
        (path for path in archive.iterdir() if path.is_dir()),
        key=lambda path: path.name,
        reverse=True,
    ) if archive.exists() else []
    if not entries:
        print('         архив пуст')
    for path in entries:
        posegraph = (path / 'map.posegraph').is_file()
        print(f'  {path.name}  posegraph={"yes" if posegraph else "no"}')


def activate_map(name: str) -> None:
    _source_package, maps_root, current, archive = map_paths()
    candidate = (archive / name).resolve()
    archive_resolved = archive.resolve()
    if candidate.parent != archive_resolved or not candidate.is_dir():
        raise MapManagerError(f'Архивная карта не найдена: {name}')
    validate_map(candidate, require_posegraph=False)

    stage = maps_root / f'.activate_{name}_{os.getpid()}'
    if stage.exists():
        _remove_path(stage)
    shutil.copytree(candidate, stage)
    archived = _archive_current(current, archive)
    stage.rename(current)
    synced, message = sync_installed_current(current)

    print(f'Активирована карта: {name}')
    print(f'Текущая карта: {current / "map.yaml"}')
    if archived is not None:
        print(f'Предыдущая current сохранена: {archived}')
    if not synced:
        print(f'ВНИМАНИЕ: install не обновлён: {message}')


def show_status() -> None:
    source_package, _maps_root, current, _archive = map_paths()
    print(f'Исходный пакет: {source_package}')
    print(f'Карта Nav2: {current / "map.yaml"}')
    if not (current / 'map.yaml').is_file():
        print('Статус: текущая карта отсутствует')
        return
    try:
        validate_map(current, require_posegraph=False)
        print('Occupancy map: OK')
    except MapManagerError as exc:
        print(f'Occupancy map: ERROR: {exc}')
    posegraph_ok = all(
        (current / f'map{suffix}').is_file()
        for suffix in ('.posegraph', '.data')
    )
    print(f'Pose graph: {"OK" if posegraph_ok else "отсутствует"}')


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='rover_map',
        description='Управление картами rover_navigation.',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    save_parser = subparsers.add_parser(
        'save',
        help='Сохранить /map и pose graph как maps/current.',
    )
    save_parser.add_argument('label', nargs='?', default='map')
    save_parser.add_argument(
        '--occupancy-only',
        action='store_true',
        help='Не сохранять pose graph SLAM Toolbox.',
    )
    save_parser.add_argument('--timeout', type=float, default=30.0)

    subparsers.add_parser('list', help='Показать current и архив карт.')
    subparsers.add_parser('status', help='Проверить текущую карту.')

    use_parser = subparsers.add_parser(
        'use',
        help='Сделать архивную карту текущей.',
    )
    use_parser.add_argument('archive_name')
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == 'save':
            save_map(args.label, args.occupancy_only, args.timeout)
        elif args.command == 'list':
            list_maps()
        elif args.command == 'status':
            show_status()
        elif args.command == 'use':
            activate_map(args.archive_name)
        else:
            parser.error(f'Неизвестная команда: {args.command}')
        return 0
    except (MapManagerError, OSError) as exc:
        print(f'ОШИБКА: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
