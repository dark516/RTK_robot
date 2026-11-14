from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_device_manager'

setup(
    name=package_name,
    version='0.4.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='Automatic serial device discovery for rover hardware.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'discover_devices = rover_device_manager.discover_devices:main',
            'setup_devices = rover_device_manager.setup_devices:main',
        ],
    },
)
