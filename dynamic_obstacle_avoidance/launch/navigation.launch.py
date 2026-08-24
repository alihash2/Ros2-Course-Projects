import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_my_planner = get_package_share_directory('dynamic_obstacle_avoidance')

    # Args
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model = LaunchConfiguration('model', default='waffle')
    map_yaml = LaunchConfiguration('map', default='/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml')
    enable_custom_follower = LaunchConfiguration('enable_custom_follower', default='false')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', model),
        
        DeclareLaunchArgument('params_file', 
                             default_value=os.path.join(pkg_my_planner, 'params', 'nav2_params_astar.yaml'),
                             description='Full path to the ROS2 parameters file to use for all launched nodes'),
        
        DeclareLaunchArgument('map',
                             default_value=map_yaml,
                             description='Full path to map yaml file to load'),

        DeclareLaunchArgument('enable_custom_follower',
                             default_value='false',
                             description='Set true to run custom PID/LQR path_follower instead of Nav2 controller'),

        # 1. Gazebo + Robot Spawning
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py'))
        ),

        # 2. Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'autostart': 'true'
            }.items()
        ),

        # 3. RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz_config': os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
            }.items()
        ),

        # 4. Twist to TwistStamped relay for Gazebo Bridge
        Node(
            package='dynamic_obstacle_avoidance',
            executable='cmd_vel_relay',
            name='cmd_vel_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 5. Optional Custom path follower
        Node(
            condition=IfCondition(enable_custom_follower),
            package='dynamic_obstacle_avoidance',
            executable='path_follower_node',
            name='path_follower',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
