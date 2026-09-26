from setuptools import find_packages, setup

package_name = 'bartender_open'

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
    description=('Two-armed bottle opening: one arm holds the beer, '
                 'the other pushes the opener down onto its cap'),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'open_action_server = bartender_open.open_action_server:main',
        ],
    },
)
