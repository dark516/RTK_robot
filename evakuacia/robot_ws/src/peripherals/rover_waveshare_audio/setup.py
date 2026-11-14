from glob import glob
import os
from pathlib import Path

from setuptools import find_packages, setup


package_name = 'rover_waveshare_audio'


def package_files(directory: str):
    files = []
    root = Path(directory)
    if not root.exists():
        return files
    for path in root.rglob('*'):
        if path.is_file():
            install_dir = Path('share') / package_name / path.parent
            files.append((str(install_dir), [str(path)]))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'tools'), glob('tools/*')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ] + package_files('firmware'),
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='Waveshare ESP32-S3-AUDIO-Board Whisper STT/TTS bridge for ROS 2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waveshare_audio_node = rover_waveshare_audio.waveshare_audio_node:main',
        ],
    },
)
