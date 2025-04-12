#!/usr/bin/env python3
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'urdf_and_meshes_of_the_robot'

setup(
    name=package_name,
    version='0.0.0',
    # Be explicit about which packages to include
    packages=[package_name, 'scripts'], # Include the scripts directory as a package 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mickyas Tamiru Asfaw',
    author_email='mickyastamiru92@gmail.com',
    maintainer='Mickyas Tamiru Asfaw',
    maintainer_email='mickyastamiru92@gmail.com',
    description='A package for robot simulation and reinforcement learning',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_nodes_control = scripts.robot_nodes_control:main',
            'train_rl_ppo = scripts.train_rl_ppo:main',
        ],
    },
)