from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_navigation'

setup(
    name=package_name,
    version='0.4.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.md')),
        (
            os.path.join('share', package_name, 'maps', 'current'),
            glob('maps/current/*'),
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='Nav2, SLAM and project-local map management for the mecanum rover.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'rover_map = rover_navigation.map_manager:main',
        ],
    },
)
