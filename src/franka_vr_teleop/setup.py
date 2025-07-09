from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'franka_vr_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhengyang Kris Weng',
    maintainer_email='wengmister@gmail.com',
    description='Teleoperating franka FER with quest 3s',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_to_robot_converter = franka_vr_teleop.vr_to_robot_converter:main',
        ],
    },
)