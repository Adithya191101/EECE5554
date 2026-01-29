from setuptools import setup
import os
from glob import glob

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))  # This line includes the launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adithya',
    maintainer_email='rajendran.ad@northeastern.edu',
    description='IMU Driver for VN-100',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = imu_driver.driver:main',
        ],
    },
)
