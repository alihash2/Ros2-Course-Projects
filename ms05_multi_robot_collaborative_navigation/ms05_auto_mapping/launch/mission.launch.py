#!/usr/bin/env python3
"""Issue 3: one-command bringup of the whole mission.

    ros2 launch ms05_auto_mapping mission.launch.py

Starts, in order:
  1. ms05_gazebo_worlds/launch/simulation.launch.py
     - multi_room world, both Waffles (waffle_r1 / waffle_r2),
       per-robot bridges, cmd_vel relays, robot_state_publishers
  2. nav2_robot1.launch.py  - slam_toolbox mapping (publishes global /map)
  3. nav2_robot2.launch.py  - AMCL + Nav2 server stack in /robot2
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_auto = get_package_share_directory("ms05_auto_mapping")
    pkg_worlds = get_package_share_directory("ms05_gazebo_worlds")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation time")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, "launch", "simulation.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    nav2_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_auto, "launch", "nav2_robot1.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    nav2_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_auto, "launch", "nav2_robot2.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        simulation,
        nav2_robot1,
        nav2_robot2,
    ])
