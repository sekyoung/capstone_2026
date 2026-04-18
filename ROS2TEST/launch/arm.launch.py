"""
arm.launch.py — launch the full robot arm software stack.

Nodes started
─────────────
  robot_state_publisher   reads URDF + /joint_states  → publishes /tf
  sweep_node              publishes simulated target poses
  ik_node                 QP-IK solver   → /joint_states
  meshcat_bridge          /joint_states  → Meshcat web viewer
  rviz2                   visualises the robot from /tf

Usage
─────
  # build first (once):
  cd ~/Desktop/QP/ROS2TEST
  colcon build --symlink-install
  source install/setup.bash

  # run:
  ros2 launch robot_arm_ros2 arm.launch.py

  # optional overrides:
  ros2 launch robot_arm_ros2 arm.launch.py sweep_radius:=0.12 ik_rate_hz:=50.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm_ros2')
    urdf_path = os.path.join(pkg_share, 'urdf', 'arm.urdf')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'arm.rviz')

    # Read URDF as a string for robot_state_publisher
    with open(urdf_path, 'r') as fh:
        robot_description = fh.read()

    # ── Launch arguments (overridable on CLI) ─────────────────────────
    declare_ik_rate = DeclareLaunchArgument(
        'ik_rate_hz', default_value='100.0',
        description='IK solver and joint-state publish rate (Hz)',
    )
    declare_sweep_radius = DeclareLaunchArgument(
        'sweep_radius', default_value='0.08',
        description='Sweep circle radius (metres)',
    )
    declare_sweep_speed = DeclareLaunchArgument(
        'sweep_speed', default_value='1.0',
        description='Sweep angular speed multiplier',
    )

    # ── Nodes ─────────────────────────────────────────────────────────

    # 1. robot_state_publisher: URDF → /tf (used by RViz2)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 2. Simulated tracking source (replace with real tracking later)
    sweep_node = Node(
        package='robot_arm_ros2',
        executable='sweep_node',
        name='sweep_node',
        output='screen',
        parameters=[{
            'radius':  LaunchConfiguration('sweep_radius'),
            'speed':   LaunchConfiguration('sweep_speed'),
        }],
    )

    # 3. IK solver
    ik_node = Node(
        package='robot_arm_ros2',
        executable='ik_node',
        name='ik_node',
        output='screen',
        parameters=[{
            'urdf_path':  urdf_path,
            'ik_rate_hz': LaunchConfiguration('ik_rate_hz'),
        }],
    )

    # 4. Meshcat web viewer bridge
    meshcat_bridge = Node(
        package='robot_arm_ros2',
        executable='meshcat_bridge',
        name='meshcat_bridge',
        output='screen',
        parameters=[{'urdf_path': urdf_path}],
    )

    # 5. RViz2  (–d loads our preset config; remove the arg to use a blank panel)
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
        sweep_node,
        ik_node,
        meshcat_bridge,
        rviz2,
    ])
