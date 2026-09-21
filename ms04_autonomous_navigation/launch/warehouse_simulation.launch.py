#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ms04 = get_package_share_directory('ms04_autonomous_navigation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default='waffle')
    x_pose = LaunchConfiguration('x_pose', default='-6.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    gui = LaunchConfiguration('gui', default='true')

    world_file = os.path.join(pkg_ms04, 'worlds', 'warehouse.world')

    # Environment variables
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', model)
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_turtlebot3_gazebo, 'models')
    )

    # Declare arguments
    declare_model = DeclareLaunchArgument('model', default_value='waffle', description='TurtleBot3 model (waffle, burger)')
    declare_x = DeclareLaunchArgument('x_pose', default_value='-6.0', description='Initial X position in warehouse')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial Y position in warehouse')
    declare_gui = DeclareLaunchArgument('gui', default_value='true', description='Set true to launch Gazebo GUI')
    declare_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')

    # Gazebo server & client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(gui),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Gazebo ↔ ROS 2 bridge (clock, odom, tf, scan, cmd_vel, imu, joint_states)
    bridge_params = os.path.join(
        pkg_turtlebot3_gazebo, 'params', 'turtlebot3_waffle_bridge.yaml'
    )
    ros_gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    # Spawn TurtleBot3 — custom model with the lidar max range extended to
    # 8.0 m (RPLIDAR A2M8-class, the standard replacement lidar for this class
    # of indoor SLAM robot; stock LDS-02 only reaches 3.5 m).
    our_model = os.path.join(pkg_ms04, 'models', 'turtlebot3_waffle', 'model.sdf')
    spawn_turtlebot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'waffle', '-file', our_model,
                   '-x', x_pose, '-y', y_pose, '-z', '0.01'],
        output='screen'
    )

    # Relay for cmd_vel: bridges standard geometry_msgs/Twist on /cmd_vel_raw to TwistStamped on /cmd_vel
    cmd_vel_relay_node = Node(
        package='ms04_autonomous_navigation',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel_raw',
            'output_topic': '/cmd_vel',
            'frame_id': 'base_footprint',
            'use_sim_time': use_sim_time
        }]
    )

    ld = LaunchDescription()
    ld.add_action(set_tb3_model)
    ld.add_action(set_gz_resource_path)
    ld.add_action(declare_model)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_gui)
    ld.add_action(declare_sim_time)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(ros_gz_bridge_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(cmd_vel_relay_node)

    return ld
