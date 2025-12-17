from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_arm_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nouraldeenm00',
    maintainer_email='nouraldeenm00@gmail.com',
    description='Kinematics solver and controller for 4-DOF robot arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_kinematics_node = robot_arm_kinematics.forward_kinematics_node:main',
            'inverse_kinematics_node = robot_arm_kinematics.inverse_kinematics_node:main',
            'trajectory_planner_node = robot_arm_kinematics.trajectory_planner_node:main',
        ],
    },
)
