from setuptools import find_packages, setup

package_name = 'vehicle_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bolty',
    maintainer_email='rodneydong156@gmail.com',
    description='Vehicle odometry with EKF',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_bicycle_odometry = vehicle_odometry.odometry_publisher:main',
        ],
    },
)
