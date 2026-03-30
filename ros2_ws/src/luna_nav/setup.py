from setuptools import setup
import os
from glob import glob

package_name = 'luna_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunabot',
    maintainer_email='the.sam.rooney@gmail.com',
    description='Nav2 integration, zone publisher, frontier explorer, and mission supervisor for WPI Lunabotics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'zone_publisher = luna_nav.zone_publisher:main',
            'frontier_explorer = luna_nav.frontier_explorer:main',
            'mission_supervisor = luna_nav.mission_supervisor:main',
        ],
    },
)
