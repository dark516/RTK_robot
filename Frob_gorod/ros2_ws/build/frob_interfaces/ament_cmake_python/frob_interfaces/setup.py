from setuptools import find_packages
from setuptools import setup

setup(
    name='frob_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('frob_interfaces', 'frob_interfaces.*')),
)
