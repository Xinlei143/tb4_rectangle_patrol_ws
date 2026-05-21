from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'tb4_rectangle_patrol'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xinlei',
    maintainer_email='xinlei@example.com',
    description='TurtleBot4 rectangle patrol application using Nav2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rectangle_patrol = tb4_rectangle_patrol.rectangle_patrol_node:main',
            'initial_pose_publisher = tb4_rectangle_patrol.initial_pose_publisher:main',
        ],
    },
)
