from setuptools import find_packages, setup

package_name = 'bio_drill'

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
    maintainer='rpi',
    maintainer_email='arjoe.basak@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'cache_box=bio_drill.cache:main',
            'drill = bio_drill.drill:main',
            'auto_drill = bio_drill.drill_auto:main'
        ],
    },
)
