from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'rover_base_driver'
setup(
    name=package_name,
    version='0.2.0',
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
    description='Vendor-neutral ROS 2 base driver for the rover motor controller.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'base_driver_node = rover_base_driver.base_driver_node:main',
        'configure_motor_board = rover_base_driver.configure_motor_board:main',
    ]},
)
