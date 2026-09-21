#!/usr/bin/env python3
"""Launch file for autonomous SLAM mapping in the Office environment.

Architecture (industry-standard Nav2 + explore-lite frontier exploration):
  1. office_simulation      -> Gazebo office world + TurtleBot3 (12 m lidar) + relay
  2. slam_toolbox (async)   -> /map + map->odom TF
3. nav2_servers           -> shared Nav2 stack (planner, DWB controller, smoother,
                                behaviors, BT navigator, velocity smoother,
                                collision monitor) — owns ALL robot motion
  4. auto_slam_explorer     -> frontier detection + NavigateToPose goals + auto-save
  5. rviz2                  -> live visualization
"""
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
        default=os.path.expanduser('~/ros2_ws/src/ms04_autonomous_navigation/maps/office_map')
    )
    max_exploration_time = LaunchConfiguration('max_exploration_time', default='900.0')

    # Declare launch arguments
    declare_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_ms04, 'params', 'slam_toolbox_params.yaml'),
        description='Full path to slam_toolbox parameters file'
    )
    declare_map_save_path = DeclareLaunchArgument(
        'map_save_path',
        default_value=os.path.expanduser('~/ros2_ws/src/ms04_autonomous_navigation/maps/office_map'),
        description='Base path (no extension) to save generated map'
    )
    declare_max_time = DeclareLaunchArgument(
        'max_exploration_time',
        default_value='900.0',
        description='Max seconds for autonomous exploration before saving map'
    )

    # 1. Launch the office Gazebo simulation
    office_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ms04, 'launch', 'office_simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': '0.0',
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
            'slam_params_file': slam_params_file,
            'use_map_saver': 'false'  # save only on demand via SaveMap service
        }.items()
    )

    # 3. Nav2 navigation stack — owns ALL motion (planner + DWB + recoveries)
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ms04, 'launch', 'nav2_servers.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    # 4. Autonomous frontier explorer + auto map saver (drives Nav2, no cmd_vel)
    auto_explorer_node = Node(
        package='ms04_autonomous_navigation',
        executable='auto_slam_explorer.py',
        name='auto_slam_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_save_path': map_save_path,
            'max_exploration_time': ParameterValue(max_exploration_time, value_type=float),
            'min_frontier_size': 6,
            'auto_save': True,
            'gain_scale': 1.0,          # explore-lite: size dominates
            'potential_scale': 0.001,   # slight preference for closer goals
            'orientation_scale': 0.0,   # ignore orientation
            'progress_timeout': 300.0   # abort a Nav2 goal that never succeeds
        }]
    )

    # 5. RViz2 for live map + robot visualization
    rviz_config = os.path.join(pkg_ms04, 'rviz', 'mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_sim_time,
        declare_slam_params,
        declare_map_save_path,
        declare_max_time,
        office_sim,
        slam_toolbox_node,
        nav2_stack,
        auto_explorer_node,
        rviz_node,
    ])