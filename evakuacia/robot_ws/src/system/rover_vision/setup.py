from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_vision'


def data_files_from_tree(source_dir: str, install_dir: str):
    data_files = []
    for root, _dirs, files in os.walk(source_dir):
        selected = [
            os.path.join(root, name)
            for name in files
            if not name.startswith('.')
        ]
        if not selected:
            continue
        relative = os.path.relpath(root, source_dir)
        target = install_dir if relative == '.' else os.path.join(install_dir, relative)
        data_files.append((target, selected))
    return data_files

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + data_files_from_tree('models', os.path.join('share', package_name, 'models')),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='YOLO-like camera processing node for the rover.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_detector_node = rover_vision.camera_detector_node:main',
        ],
    },
)
