from setuptools import find_packages, setup

package_name = 'joint_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'full_command = joint_control.full_command:main',
            'pos_control_shoulder = joint_control.pos_control_shoulder:main',
            'pos_control_elbow = joint_control.pos_control_elbow:main'
        ],
    },
)
