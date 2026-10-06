#!/usr/bin/env python3
"""Issue 3: Robot 1's Nav2 stack - slam_toolbox in mapping mode.

slam_toolbox is the fleet's ONLY map server: it publishes the global /map
that Robot 2's AMCL subscribes to, and the TF chain

    map -> odom_robot1 -> robot1/base_footprint

The lifecycle manager (managed the same way slam_toolbox's own launch files
support via use_lifecycle_manager) configures and activates the node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg = get_package_share_directory("ms05_auto_mapping")
    params_file = os.path.join(pkg, "params", "robot1_nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation time")

    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "use_lifecycle_manager": True,
        }],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_robot1",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["slam_toolbox"],
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription([declare_use_sim_time, slam_toolbox, lifecycle])
