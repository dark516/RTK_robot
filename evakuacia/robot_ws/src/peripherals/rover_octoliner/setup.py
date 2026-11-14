from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_octoliner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='ROS 2 node for the Amperka Octoliner module.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'octoliner_node = rover_octoliner.octoliner_node:main',
        ],
    },
)
