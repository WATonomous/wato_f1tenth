from setuptools import find_packages, setup

package_name = 'rosbag_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/rosbag_recorder']),
    ('share/rosbag_recorder', ['package.xml']),
    ('share/rosbag_recorder/launch', ['launch/rosbag_recorder.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bolty',
    maintainer_email='kjscan1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['rosbag_recorder_node = rosbag_recorder.rosbag_recorder_node:main',
        ],
    },
)
