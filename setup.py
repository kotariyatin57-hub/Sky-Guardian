import os
from glob import glob
from setuptools import setup

package_name = 'drone_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),          # <-- this line registers launch files
    ],
    entry_points={
        'console_scripts': [
            'hello_node = drone_core.hello_node:main',
            'altitude_publisher = drone_core.altitude_publisher:main',
            'altitude_subscriber = drone_core.altitude_subscriber:main',
            'arm_service = drone_core.arm_service:main',
            'arm_client = drone_core.arm_client:main',
        ],
    },
)
