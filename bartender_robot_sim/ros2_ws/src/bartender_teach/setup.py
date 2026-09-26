from setuptools import find_packages, setup

package_name = 'bartender_teach'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/taught_points.yaml', 'config/workcell_points.yaml',
          'config/workcell_menu.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ami',
    maintainer_email='azmozgame@gmail.com',
    description='Teach pendant for recording named arm configurations',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teach = bartender_teach.teach_points:main',
            'teach_gui = bartender_teach.teach_gui:main',
        ],
    },
)
