import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ballbot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install the package resource marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install the package.xml
        ('share/' + package_name, ['package.xml']),
        
        # --- Custom installation rules for URDF files, meshes, and launch files ---
        # Install URDF files to share/ballbot_description/urdf
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Install Meshes to share/ballbot_description/meshes (if you have them)
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),

        # Install Launch files to share/ballbot_description/launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),

        # Install RViz configuration files to share/ballbot_description/rviz
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Contains the URDF and meshes for the Ballbot robot.',
    license='TODO: License declaration', # Update this!
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)