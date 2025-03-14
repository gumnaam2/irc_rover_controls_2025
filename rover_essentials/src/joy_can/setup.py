from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'joy_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.*')), #include all files in the launch folder
        #(os.path.join('share',package_name),glob('launch/DriveLaunch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi',
    maintainer_email='rpi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'master = joy_can.CANIn:main',
        	'drive_master = joy_can.DriveCAN:main',
        	'arm_master = joy_can.ArmCAN:main',
        	'gripper = joy_can.gripper:main',
        ],
    },
)
