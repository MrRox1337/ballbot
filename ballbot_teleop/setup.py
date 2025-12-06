from setuptools import find_packages, setup

package_name = 'ballbot_teleop'

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
    maintainer='Aman Mishra',
    maintainer_email='amanrox97@gmail.com',
    description='Teleoperation node for Ballbot driving and flap control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop_flap_node = ballbot_teleop.teleop_flap_node:main",
        ],
    },
)