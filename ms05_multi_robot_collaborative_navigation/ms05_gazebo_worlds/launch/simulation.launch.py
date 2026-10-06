#!/usr/bin/env python3
"""Issue 3: multi_room world + two namespaced TurtleBot3 Waffles.

Brings up, in one command:
  * Gazebo headless (add ``gui:=true`` for the client)
  * both robots spawned from models/waffle/waffle.template.sdf, rendered
    per robot (model names waffle_r1 / waffle_r2, gz topics robotN/...,
    frames odom_robotN + robotN/...)
  * per-robot ros_gz_bridge instances (namespace /robotN,
    expand_gz_topic_names) for scan/odom/cmd_vel/imu/joint_states/camera
  * one simulator-wide bridge for /clock, the merged global /tf and the
    ground-truth PoseArray
  * one cmd_vel_relay per robot (Twist -> TwistStamped)
  * one namespaced robot_state_publisher per robot (frame_prefix robotN/,
    URDF from turtlebot3_description)

Per-issue spawn plan: robot1 starts in the north hall (the mapper),
robot2 starts in the south hall (the localizer).
"""
import os
import tempfile
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOTS = (
    # (namespace, model name, odom frame, spawn x, y, yaw)
    ("robot1", "waffle_r1", "odom_robot1", "0.0", "4.5", "0.0"),
    ("robot2", "waffle_r2", "odom_robot2", "0.0", "-4.5", "3.14159"),
)


def _render_template(template_path, ns, odom_frame, model_name):
    """Render the Waffle SDF template for one robot into a temp file."""
    with open(template_path) as fh:
        sdf = fh.read()
    sdf = (sdf.replace("__MODEL_NAME__", model_name)
              .replace("__ODOM_FRAME__", odom_frame)
              .replace("__NS__", ns))
    leftover = [t for t in ("__MODEL_NAME__", "__ODOM_FRAME__", "__NS__") if t in sdf]
    if leftover:
        raise RuntimeError(f"template render left tokens for {model_name}: {leftover}")
    fd, path = tempfile.mkstemp(prefix=f"{model_name}_", suffix=".sdf")
    with os.fdopen(fd, "w") as fh:
        fh.write(sdf)
    return path


def generate_launch_description():
    pkg = get_package_share_directory("ms05_gazebo_worlds")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    pkg_turtlebot3_description = get_package_share_directory("turtlebot3_description")

    world = os.path.join(pkg, "worlds", "multi_room.world")
    template = os.path.join(pkg, "models", "waffle", "waffle.template.sdf")
    robot_bridge = os.path.join(pkg, "config", "robot_bridge.yaml")
    sim_bridge = os.path.join(pkg, "config", "sim_bridge.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation time on all ROS nodes")
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="false",
        description="Set true to launch the Gazebo GUI client")

    # model://turtlebot3_common/... mesh URIs in the Waffle SDF resolve here
    # (same mechanism ms04 uses); the world's ../markers QR textures resolve
    # relative to the world file itself.
    set_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_turtlebot3_gazebo, "models"))

    # Plain TB3 URDF (namespace arg empty); per-robot frame names come from
    # robot_state_publisher's frame_prefix below.
    urdf = xacro.process_file(
        os.path.join(pkg_turtlebot3_description, "urdf", "turtlebot3_waffle.urdf"),
        mappings={"namespace": ""},
    ).toxml()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s -v2 ", world],
                          "on_exit_shutdown": "true"}.items(),
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        condition=IfCondition(gui),
        launch_arguments={"gz_args": "-g -v2 ",
                          "on_exit_shutdown": "true"}.items(),
    )

    actions = [declare_use_sim_time, declare_gui, set_resource_path,
               gz_sim, gz_client]

    for ns, model_name, odom_frame, x, y, yaw in ROBOTS:
        sdf_path = _render_template(template, ns, odom_frame, model_name)
        actions.append(Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{model_name}",
            output="screen",
            arguments=["-name", model_name, "-file", sdf_path,
                       "-x", x, "-y", y, "-z", "0.01", "-Y", yaw],
        ))
        actions.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="parameter_bridge",
            namespace=f"/{ns}",
            output="screen",
            arguments=["--ros-args",
                       "-p", f"config_file:={robot_bridge}",
                       "-p", "expand_gz_topic_names:=true"],
            parameters=[{"use_sim_time": use_sim_time}],
        ))
        actions.append(Node(
            package="ms05_gazebo_worlds",
            executable="cmd_vel_relay",
            name=f"cmd_vel_relay_{ns}",
            output="screen",
            parameters=[{
                "input_topic": f"/{ns}/cmd_vel",
                "output_topic": f"/{ns}/cmd_vel_stamped",
                "frame_id": f"{ns}/base_footprint",
                "use_sim_time": use_sim_time,
            }],
        ))
        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=f"/{ns}",
            output="screen",
            parameters=[{
                "robot_description": urdf,
                "frame_prefix": f"{ns}/",
                "use_sim_time": use_sim_time,
            }],
        ))

    # One bridge for the shared simulator topics: /clock, the merged global
    # /tf (both robots' odom->base transforms) and Gazebo's ground-truth
    # pose array used by issue 7.
    actions.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sim_bridge",
        output="screen",
        arguments=["--ros-args", "-p", f"config_file:={sim_bridge}"],
        parameters=[{"use_sim_time": use_sim_time}],
    ))

    return LaunchDescription(actions)
