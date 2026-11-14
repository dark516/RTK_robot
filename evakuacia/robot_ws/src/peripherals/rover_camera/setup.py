from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='USB camera ROS 2 node for the rover.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'usb_camera_node = rover_camera.usb_camera_node:main',
        ],
    },
)
