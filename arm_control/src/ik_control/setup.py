from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ik_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share',package_2,'urdf'),glob('urdf/*.py'))
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sudhindra',
    maintainer_email='154395800+Sudhindra2005@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pos_ik = ik_control.pos_ik:main',
            'ik_pos_pid = ik_control.pos_ik_pid:main',
            'shoulder_hold_ik = wrist_testing.shoulder_hold_ik:main'
        ],
    },
)
