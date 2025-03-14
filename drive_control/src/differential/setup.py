from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'differential'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'joystick_launcher.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kushaal',
    maintainer_email='kushaalmatam1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_receiverrr = codes.drive_receiver:main',
            'cam_receiverrr = codes.cam_receiver:main',
            'arm_end_receiverrr = codes.arm_end_receiver:main',
            'joystick_innerrr_arm = codes.joystick_inputter_arm:main',
            'joystick_innerrr = codes.joystick_inputter:main',
            'motor_six_receiverrr = codes.motor_six_receiver:main',
            'rtk_demo = codes.serial_pub:main',
            'byte_array = codes.byte_array:main',
            'drive_arm_concatenate = codes.drive_arm_concatenate:main',
            'base_tuning = codes.base_tuning:main',
            'node1 = codes.base_tuning_node1:main',
            'node2 = codes.base_tuning_node2:main',
            'freshies_attempt = codes.freshies_attempt:main'
        ],
    },
)
