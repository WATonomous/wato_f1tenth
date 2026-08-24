import os
from glob import glob
from setuptools import setup

package_name = 'region_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            [f for f in glob('launch/*.py') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cedric',
    maintainer_email='cedric.hollande25@gmail.com',
    description='Bridge region state machine: publishes /region/active based on PF pose + bubble triggers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'region_manager = region_manager.region_manager_node:main',
        ],
    },
)
