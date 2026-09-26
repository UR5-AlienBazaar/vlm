import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bartender_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ami',
    maintainer_email='azmozgame@gmail.com',
    description='Top-level launch files bringing up sim, control, and MoveIt together',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twin_mirror = bartender_bringup.twin_mirror:main',
        ],
    },
)
