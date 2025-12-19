from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ballbot_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Navigation package for ballbot using Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = ballbot_nav2.cmd_vel_relay:main',
            'simple_commander = ballbot_nav2.simple_commander:main',
            'waypoint_commander = ballbot_nav2.waypoint_commander:main',
        ],
    },
)