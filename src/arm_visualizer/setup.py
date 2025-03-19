from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_visualizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abulhasnat-abdullah',
    maintainer_email='abdkalam22@gmail.com',
    description='A ROS2 package for visualizing and controlling a robot arm using Tkinter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_visualizer = arm_visualizer.joint_visualizer:main',
            'three_d_visualizer = arm_visualizer.three_d_visualizer:main'
        ],
    },
)