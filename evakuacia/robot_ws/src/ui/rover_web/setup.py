from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_web'


def recursive_data_files(source_dir: str, install_dir: str):
    entries = []
    for root, _, files in os.walk(source_dir):
        if not files:
            continue
        relative = os.path.relpath(root, source_dir)
        destination = install_dir if relative == '.' else os.path.join(
            install_dir, relative
        )
        entries.append(
            (destination, [os.path.join(root, name) for name in files])
        )
    return entries


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]
data_files.extend(
    recursive_data_files('web', os.path.join('share', package_name, 'web'))
)
data_files.extend(
    recursive_data_files('tools', os.path.join('share', package_name, 'tools'))
)
data_files.extend(
    recursive_data_files('plans', os.path.join('share', package_name, 'plans'))
)


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='Local web interface, gateway and route executor for the rover.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_gateway_node = rover_web.web_gateway_node:main',
        ],
    },
)
