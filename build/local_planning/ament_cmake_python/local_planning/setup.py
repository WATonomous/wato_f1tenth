from setuptools import find_packages
from setuptools import setup

setup(
    name='local_planning',
    version='0.0.0',
    packages=find_packages(
        include=('local_planning', 'local_planning.*')),
)
