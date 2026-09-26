from setuptools import find_packages, setup

package_name = 'bartender_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/stand_camera.yaml', 'config/bottles.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ami',
    maintainer_email='azmozgame@gmail.com',
    description='Read-only HTTP/JSON control API (docs/CONTROL_API.md, Phase B)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
        # Optional: label OCR, and Gemini as its fallback.
        'drink': [
            'paddlepaddle',
            'paddleocr',
            'google-genai',
        ],
    },
    entry_points={
        'console_scripts': [
            'server = bartender_api.server:main',
        ],
    },
)
