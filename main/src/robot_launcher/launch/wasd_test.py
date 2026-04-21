import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = get_package_share_directory('ik_solver')
    rviz_path = get_package_share_directory('rviz_viewer')
    urdf_path = os.path.join(urdf_path, 'urdf', 'arm.urdf')
    rviz_cfg  = os.path.join(rviz_path, 'rviz', 'arm.rviz')

    with open(urdf_path, 'r') as fh:
        robot_description = fh.read()

    declare_ik_rate = DeclareLaunchArgument(
        'ik_rate_hz', default_value='100.0',
        description='IK solve + joint-state publish rate (Hz)',
    )
    declare_sweep_radius = DeclareLaunchArgument(
        'sweep_radius', default_value='0.08',
        description='Sweep circle radius (metres)',
    )
    declare_sweep_speed = DeclareLaunchArgument(
        'sweep_speed', default_value='1.0',
        description='Sweep angular speed multiplier',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    teleop_node = Node(
        package='wasd_teleop',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
    )

    ik_node = Node(
        package='ik_solver',
        executable='ik_node',
        name='ik_node',
        output='screen',
        parameters=[{
            'urdf_path':  urdf_path,
            'ik_rate_hz': LaunchConfiguration('ik_rate_hz'),
        }],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
    )

    return LaunchDescription([
        declare_ik_rate,
        declare_sweep_radius,
        declare_sweep_speed,
        robot_state_publisher,
        teleop_node,
        ik_node,
        rviz2,
    ])
