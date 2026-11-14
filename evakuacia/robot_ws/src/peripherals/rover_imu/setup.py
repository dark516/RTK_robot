from glob import glob
import os

from setuptools import find_packages, setup
package_name = 'rover_imu'
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
    maintainer='Rover Team', maintainer_email='maintainer@example.com',
    description='Native Yahboom/YB-MRA02 IMU serial driver for ROS 2.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'imu_normalizer_node = rover_imu.imu_normalizer_node:main',
        'yahboom_imu_node = rover_imu.yahboom_imu_node:main',
    ]},
)
