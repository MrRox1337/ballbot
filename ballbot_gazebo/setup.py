import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ballbot_gazebo'

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
        
        # (Optional) Install world files if you add them later
        # (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        
        # (Optional) Install model files if you add them later
        # (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aman Mishra',
    maintainer_email='amanrox97@gmail.com',
    description='Gazebo simulation package for Ballbot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)