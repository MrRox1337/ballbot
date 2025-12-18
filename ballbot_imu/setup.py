import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ballbot_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aman Mishra',
    maintainer_email='amanrox97@gmail.com',
    description='IMU sensor integration for Ballbot (simulated BNO055 absolute orientation sensor)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_processor = ballbot_imu.imu_processor:main',
            'imu_filter = ballbot_imu.imu_filter:main',
        ],
    },
)