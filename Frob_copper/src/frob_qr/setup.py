from setuptools import find_packages, setup

package_name = 'frob_qr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/qr.launch.py']),
        ('share/' + package_name + '/launch', ['launch/navigate.launch.py']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Kulagin',
    maintainer_email='sashakulagin2007@gmail.com',
    description='QR code detection and PID tracking control',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'qr_detector = frob_qr.qr_detector:main',
            'qr_controller = frob_qr.qr_controller:main',
            'qr_navigator = frob_qr.qr_navigator:main',
        ],
    },
)
