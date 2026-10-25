import os
import sys

from setuptools import find_packages, setup

package_name = 'mppi_example'

def _strip_arg(argv, flag, takes_value=False):
    while flag in argv:
        idx = argv.index(flag)
        del argv[idx]
        if takes_value and idx < len(argv):
            del argv[idx]


# Older setuptools builds on the Jetson can choke on the legacy colcon develop
# flags used for --symlink-install. Strip only the compatibility flags that are
# safe to ignore so the package still builds across both environments.
_strip_arg(sys.argv, '--editable')
_strip_arg(sys.argv, '--build-directory', takes_value=True)

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, [
        'mppi_example/config.yaml',
        'mppi_example/README.md',
        'mppi_example/LICENSE',
    ]),
]

for root, _, files in os.walk('mppi_example/waypoints'):
    if files:
        destination = os.path.join('share', package_name, root.replace('mppi_example/', ''))
        data_files.append((destination, [os.path.join(root, name) for name in files]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cedric',
    maintainer_email='cedrich@seas.upenn.edu',
    description='JAX MPPI controller for F1TENTH/Roboracer.',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "mppi_node = mppi_example.mppi_node:main",
        ],
    },
)
