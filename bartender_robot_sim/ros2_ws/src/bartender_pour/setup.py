from setuptools import find_packages, setup

package_name = 'bartender_pour'

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
    maintainer='ami',
    maintainer_email='azmozgame@gmail.com',
    description='Pour-drink ROS2 action server and pick-tilt-pour state machine',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pour_action_server = bartender_pour.pour_action_server:main',
        ],
    },
)
