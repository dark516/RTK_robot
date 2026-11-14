from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_led_strip'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='ROS 2 driver and controller for an addressable LED strip.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'led_strip_node = rover_led_strip.led_strip_node:main',
        ],
    },
)
