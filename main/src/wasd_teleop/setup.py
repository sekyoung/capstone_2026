from setuptools import find_packages, setup

package_name = 'wasd_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wsl_sek',
    maintainer_email='131950672+sekyoung@users.noreply.github.com',
    description='WASD keyboard teleop for /target_hand_pose',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = wasd_teleop.teleop_node:main',
        ],
    },
)
