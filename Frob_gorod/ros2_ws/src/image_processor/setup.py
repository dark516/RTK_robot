from setuptools import setup

package_name = 'image_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kulagin Alex',
    maintainer_email='sashakulagin2007@gmail.com',
    description='image processor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = image_processor.image_processor:main'
        ],
    },
)
