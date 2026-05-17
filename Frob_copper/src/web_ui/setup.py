from setuptools import find_packages, setup

package_name = 'web_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/web_ui.launch.py']),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='Alex Kulagin',
    maintainer_email='sashakulagin2007@gmail.com',
    description='Web UI for Frob robot — distance from /dist topic',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'web_server = web_ui.web_server:main',
        ],
    },
)
