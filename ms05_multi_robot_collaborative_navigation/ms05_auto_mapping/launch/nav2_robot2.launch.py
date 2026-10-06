#!/usr/bin/env python3
"""Issue 3: Robot 2's Nav2 stack - AMCL bound to Robot 1's live map.

Runs inside namespace /robot2:

    amcl                localisation against the GLOBAL /map
                        (('map', '/map') remap + latched transient-local QoS
                        from slam_toolbox/map QoS); publishes
                        map -> odom_robot2 -> robot2/base_footprint
    planner/controller/behaviors/bt/waypoint/smoother servers
    velocity_smoother   cmd_vel chain: controller -> cmd_vel_nav ->
                        smoother -> cmd_vel -> (simulation.launch.py relay)

TF note: unlike upstream nav2_bringup this launch does NOT remap
('/tf', 'tf') - both robots share one global /tf so view_frames shows a
single connected tree (frames are unique per robot instead).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory("ms05_auto_mapping")
    params_file = os.path.join(pkg, "params", "robot2_nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation time")

    # Wrap the whole file under the /robot2 namespace key so the parameters
    # match the fully qualified node names (nav2's standard RewrittenYaml
    # root_key pattern).
    configured_params = RewrittenYaml(
        source_file=params_file, root_key="robot2",
        param_rewrites={}, convert_types=True)

    lifecycle_nodes = [
        "amcl", "controller_server", "planner_server", "smoother_server",
        "behavior_server", "bt_navigator", "waypoint_follower",
        "velocity_smoother",
    ]

    # NOTE: no ('/tf', 'tf') remap - global /tf on purpose (see header).
    remap_map = [("map", "/map")]
    remap_cmd_vel = [("cmd_vel", "cmd_vel_nav")]

    nodes = [
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            remappings=remap_map,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            remappings=remap_cmd_vel,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            remappings=remap_cmd_vel,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
            remappings=remap_cmd_vel + [("cmd_vel_smoothed", "cmd_vel")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_robot2",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": lifecycle_nodes,
                "use_sim_time": use_sim_time,
            }],
        ),
    ]

    return LaunchDescription([
        declare_use_sim_time,
        GroupAction([PushROSNamespace("/robot2")] + nodes),
    ])
