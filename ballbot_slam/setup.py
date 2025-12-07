import os
from glob import glob
from setuptools import setup

package_name = 'ballbot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Standard installation files
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        
        # Install config files (the parameter file)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aman Mishra', # Replace with your name
    maintainer_email='amanrox97@gmail.com', # Replace with your email
    description='SLAM package for Ballbot using slam_toolbox in Gazebo',
    license='Apache-2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)