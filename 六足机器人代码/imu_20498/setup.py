import os
from setuptools import setup

package_name = 'imu_20498'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/imu_20498.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycy',
    maintainer_email='ycy@todo.todo',
    description='IMU20498 9-axis IMU driver via USB1',
    license='MIT',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_20498_driver = imu_20498:main',
        ],
    },
)