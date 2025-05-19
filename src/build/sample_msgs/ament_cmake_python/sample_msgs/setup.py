from setuptools import find_packages
from setuptools import setup

setup(
    name='sample_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('sample_msgs', 'sample_msgs.*')),
)
