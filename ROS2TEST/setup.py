from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament package index marker
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # URDF
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # rviz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='Robot arm IK with ROS2, RViz2, and Meshcat',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node        = robot_arm_ros2.ik_node:main',
            'meshcat_bridge = robot_arm_ros2.meshcat_bridge:main',
            'sweep_node     = robot_arm_ros2.sweep_node:main',
        ],
    },
)
