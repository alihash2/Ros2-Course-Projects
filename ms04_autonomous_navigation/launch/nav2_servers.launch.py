#!/usr/bin/env python3
"""Launch the shared Nav2 navigation server stack (localization-agnostic).

Starts the servers needed for NavigateToPose and FollowWaypoints:
  - planner_server (NavFN, allow_unknown so it can plan toward frontier goals)
  - controller_server (DWB local planner)
  - smoother_server
  - behavior_server (recoveries)
  - bt_navigator (NavigateToPose action server)
  - waypoint_follower (FollowWaypoints action server)
  - velocity_smoother (caps cmd_vel to modest limits)
  - collision_monitor (last-chance stop before /cmd_vel_relay)
Wrapped in a LifecycleManager with autostart.

This is a shared constant: the mapping launches and the AMCL-based
navigation launches (office/warehouse) all include it, each supplying the
params_file appropriate to their phase/environment. It is deliberately
localization-agnostic — /map and map->odom TF may come from slam_toolbox
(mapping) or map_server + AMCL (navigation).

cmd_vel chain:
  controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
    -> collision_monitor -> cmd_vel_raw -> relay -> /cmd_vel (gz)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ms04 = get_package_share_directory('ms04_autonomous_navigation')
    pkg_bt_nav = get_package_share_directory('nav2_bt_navigator')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_ms04, 'params', 'nav2_exploration_params.yaml')
    )
    default_bt_xml = LaunchConfiguration(
        'default_bt_xml',
        default=os.path.join(
            pkg_bt_nav,
            'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml'
        )
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor',
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'default_bt_xml_filename': default_bt_xml}],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'default_bt_xml_filename': default_bt_xml}],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'default_bt_xml_filename': default_bt_xml}],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'default_bt_xml_filename': default_bt_xml}],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'default_bt_xml_filename': default_bt_xml}],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'default_bt_xml_filename': default_bt_xml}],
        remappings=remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'default_bt_xml_filename': default_bt_xml}],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file, {'default_bt_xml_filename': default_bt_xml}],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('default_bt_xml', default_value=default_bt_xml),
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager,
    ])