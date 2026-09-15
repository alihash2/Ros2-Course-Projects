#!/usr/bin/env python3
"""Launch file for autonomous SLAM mapping in the Warehouse environment."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_ms04 = get_package_share_directory('ms04_autonomous_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(pkg_ms04, 'params', 'slam_toolbox_params.yaml')
    )
    map_save_path = LaunchConfiguration(
        'map_save_path',
        default=os.path.join(pkg_ms04, 'maps', 'warehouse_map')
    )
    max_exploration_time = LaunchConfiguration('max_exploration_time', default='300.0')

    # Declare launch arguments
    declare_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_ms04, 'params', 'slam_toolbox_params.yaml'),
        description='Full path to slam_toolbox parameters file'
    )
    declare_map_save_path = DeclareLaunchArgument(
        'map_save_path',
        default_value=os.path.join(pkg_ms04, 'maps', 'warehouse_map'),
        description='Base path (no extension) to save generated map'
    )
    declare_max_time = DeclareLaunchArgument(
        'max_exploration_time',
        default_value='300.0',
        description='Max seconds for autonomous exploration before saving map'
    )

    # 1. Launch the warehouse Gazebo simulation
    warehouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ms04, 'launch', 'warehouse_simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': '-6.0',
            'y_pose': '0.0'
        }.items()
    )

    # 2. SLAM Toolbox (online async mapping)
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # 3. Autonomous frontier explorer + auto map saver
    auto_explorer_node = Node(
        package='ms04_autonomous_navigation',
        executable='auto_slam_explorer.py',
        name='auto_slam_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_save_path': map_save_path,
            'max_exploration_time': ParameterValue(max_exploration_time, value_type=float),
            'linear_speed': 0.22,
            'angular_speed': 0.6,
            'min_frontier_size': 6,
            'obstacle_distance': 0.45,
            'auto_save': True
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(declare_slam_params)
    ld.add_action(declare_map_save_path)
    ld.add_action(declare_max_time)
    ld.add_action(warehouse_sim)
    ld.add_action(slam_toolbox_node)
    ld.add_action(auto_explorer_node)
    return ld
