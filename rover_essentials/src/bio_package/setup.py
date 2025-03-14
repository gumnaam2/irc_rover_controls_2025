from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bio_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/bio_launch.py')),
    ],
    install_requires=['setuptools','requests'],
    zip_safe=True,
    maintainer='rpi',
    maintainer_email='arjoe.basak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pid=bio_package.pid:main',
        'cameras=bio_package.cameras:main',
        'relay=bio_package.relay:main',
        'bio_controller=bio_package.bio_controller:main',
        'bio_cache=bio_package.node_mcu_interface:main',
        'joining_node=bio_package.joining_node:main',
        'node_mcu_interface=bio_package.node_mcu_interface:main'
        ],
    },
)
