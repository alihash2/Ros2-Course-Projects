#!/usr/bin/env python3
"""Launch AMCL-based navigation in the WAREHOUSE environment.

Architecture:
  1. warehouse_simulation    -> Gazebo warehouse world + TurtleBot3 + relay
  2. nav2_servers            -> shared Nav2 stack with warehouse_tuned params
  3. map_server              -> static map from maps/warehouse_map.yaml
  4. amcl                    -> particle-filter localization (map -> odom TF)
  5. navigation_coordinator  -> unified mission/action control (issue 4)
  6. rviz2 (nav2_gui_view)   -> robot model, costmaps, paths, waypoint markers
Localization lifecycle (map_server + amcl) is managed separately from the
navigation lifecycle inside nav2_servers.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ms04 = get_package_share_directory('ms04_autonomous_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    gui = LaunchConfiguration('gui', default='true')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_ms04, 'params', 'warehouse_nav2_params.yaml')
    )
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_ms04, 'maps', 'warehouse_map.yaml')
    )

    declare_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')
    declare_gui = DeclareLaunchArgument('gui', default_value='true',
                                        description='Set false to run Gazebo headless')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_ms04, 'params', 'warehouse_nav2_params.yaml')
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_ms04, 'maps', 'warehouse_map.yaml')
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    warehouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ms04, 'launch', 'warehouse_simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': '-6.0',
            'y_pose': '0.0',
            'gui': gui
        }.items()
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ms04, 'launch', 'nav2_servers.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file
        }.items()
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml}],
        remappings=remappings,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        remappings=remappings,
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
            'use_sim_time': use_sim_time,
        }],
    )

    coordinator = Node(
        package='ms04_autonomous_navigation',
        executable='navigation_coordinator_node',
        name='navigation_coordinator_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_config = os.path.join(pkg_ms04, 'rviz', 'nav2_gui_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_sim_time,
        declare_autostart,
        declare_gui,
        declare_params,
        declare_map,
        warehouse_sim,
        nav2_stack,
        map_server,
        amcl,
        localization_lifecycle,
        coordinator,
        rviz_node,
    ])