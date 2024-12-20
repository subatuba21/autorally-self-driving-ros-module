import os
from glob import glob
from setuptools import setup

package_name = 'team1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processing = team1.lidar_processing:main',
            'automatic_braking = team1.automatic_braking:main',
            'pure_pursuit_controller = team1.pure_pursuit_controller:main',
            'ackermann_publisher = team1.ackermann_publisher:main'
        ],
    },
)
