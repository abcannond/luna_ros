from setuptools import setup

package_name = 'fiducial_localizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fiducial_localizer.launch.py']),
        ('share/' + package_name + '/params', ['params/jetson.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moon Tracks',
    maintainer_email='you@example.com',
    description='Robot pose from ArUco marker.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'marker_localizer = fiducial_localizer.marker_localizer:main',
        ],
    },
)
