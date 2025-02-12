from setuptools import setup
import os
from glob import glob

package_name = 'image_display'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'images'), glob('images/*.jpg')),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'image_display = image_display.image_display_node:main',
        ],
    },
)
