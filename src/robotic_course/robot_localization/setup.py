from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
   
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='narjes',
    maintainer_email='ghdnfrynrjs2@gmail.com',
    description='ROS 2 package for map publishing, localization and path planning',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'map_pub_node = robot_localization.map_pub_node:main',
            'astar_node = robot_localization.astar_node:main',
            'frame_id_converter = robot_localization.frame_id_converter:main',
            'motor_command_node = robot_localization.motor_command:main',
            'ekf_node = robot_localization.ekf_node:main',
            'particlefilter_node = robot_localization.particlefilter_node:main',
        ],
    },
)
