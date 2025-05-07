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
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'joystick_innerrr = differential.joystick_inputter:main',
            'freshies_attempt = differential.freshies_attempt:main'
        ],
    },
)
