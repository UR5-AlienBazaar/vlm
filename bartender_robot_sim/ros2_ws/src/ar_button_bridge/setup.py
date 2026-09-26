from setuptools import find_packages, setup

package_name = 'ar_button_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alessandro Sica',
    maintainer_email='alessandrosica500@gmail.com',
    description='Publishes the Spectacles AR button presses on /ar/button',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ar_button_bridge = ar_button_bridge.bridge:main',
            'ar_button_listener = ar_button_bridge.listener:main',
        ],
    },
)
