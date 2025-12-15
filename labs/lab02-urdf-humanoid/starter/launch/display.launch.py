#!/usr/bin/env python3
"""
Launch file for Lab 2: Humanoid URDF Visualization

This launch file:
1. Loads the URDF file
2. Publishes robot state (robot_state_publisher)
3. Publishes joint states with GUI (joint_state_publisher_gui)
4. Opens RViz for visualization

Usage:
  ros2 launch humanoid_description display.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for humanoid visualization."""

    # Declare launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    # Get package share directory
    pkg_share = FindPackageShare('humanoid_description').find('humanoid_description')

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid.urdf')

    # Path to RViz config
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'humanoid.rviz')

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher node (publishes TF based on URDF and joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # Joint State Publisher GUI node (provides sliders to control joints)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # RViz node (visualization)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
