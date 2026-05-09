from setuptools import setup
import os
from glob import glob

package_name = 'luna_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'config', 'experimental'),
         glob(os.path.join('config', 'experimental', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunabot',
    maintainer_email='the.sam.rooney@gmail.com',
    description='Visual SLAM mapping stack (D455 + RTAB-Map) for WPI Lunabotics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_echo = luna_mapping.joy_echo:main',
            'camera_info_fixer = luna_mapping.camera_info_fixer:main',
            'tf_relay = luna_mapping.tf_relay:main',
            'image_frame_fixer = luna_mapping.image_frame_fixer:main',
            'depth_fov_filter = luna_mapping.depth_fov_filter:main',
            'depth_to_rgb = luna_mapping.depth_to_rgb:main',
            'height_filter_pointcloud = luna_mapping.height_filter_pointcloud:main',
            'depth_round_obstacle_detector = '
            'luna_mapping.depth_round_obstacle_detector:main',
            'depth_obstacle_tf_node = luna_mapping.depth_obstacle_tf_node:main',
            'depth_crater_detector = luna_mapping.depth_crater_detector:main',
            'odom_relay = luna_mapping.odom_relay:main',
        ],
    },
)
