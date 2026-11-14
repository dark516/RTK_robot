"""Check installed commands/assets rather than starting ROS or inference."""
import importlib.util
from pathlib import Path
import runpy
from unittest.mock import patch
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


def test_installed_entrypoints_resolve_to_existing_modules(monkeypatch):
    monkeypatch.chdir(PACKAGE)
    with patch('setuptools.setup') as setup:
        runpy.run_path(str(PACKAGE / 'setup.py'), run_name='packaging_check')
    metadata = setup.call_args.kwargs
    for entry in metadata['entry_points']['console_scripts']:
        module = entry.split('=', 1)[1].strip().split(':')[0]
        assert importlib.util.find_spec(module) is not None, entry
    installed = {str(path) for _, files in metadata['data_files'] for path in files}
    assert 'scripts/download_llm_models.sh' in installed
    assert 'launch/vlm_diagnostic.launch.py' in installed
    dependencies = {element.text for element in ET.parse(PACKAGE / 'package.xml').getroot()}
    assert 'rover_configuration' not in dependencies
    assert 'python3-zxing-cpp' not in dependencies


def test_launch_arguments_resolve_without_starting_nodes(monkeypatch):
    import pytest
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node

    monkeypatch.setattr('ament_index_python.packages.get_package_share_directory', lambda _: str(PACKAGE))
    expected = {
        'local_llm_gateway': {'config_file', 'llama_server', 'models_dir', 'default_model', 'device', 'autostart'},
        'path_clearance': {'config_file', 'camera_url'},
        'vlm_diagnostic': {'robot_url', 'vlm_server_url', 'mode'},
    }
    for name, arguments in expected.items():
        namespace = runpy.run_path(str(PACKAGE / 'launch' / f'{name}.launch.py'))
        description = namespace['generate_launch_description']()
        context = LaunchContext()
        declared = set()
        nodes = []
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
                declared.add(entity.name)
            elif isinstance(entity, Node):
                nodes.append(entity)
        assert declared == arguments
        assert len(nodes) == 1
        assert perform_substitutions(context, normalize_to_list_of_substitutions(nodes[0].node_executable)) == name + '_node'
