import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to packages
    pkg_turtlebot3_control = get_package_share_directory('ms03_turtlebot3_control')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # 2. Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 3. Include TurtleBot3 World Launch (Simulation)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 4. Our Custom Nodes
    # We set output='screen' so logs appear in the terminal.
    # To see specific node logs, you can look for the [node_name] prefix in the terminal.
    
    odom_node = Node(
        package='ms03_turtlebot3_control',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    scan_node = Node(
        package='ms03_turtlebot3_control',
        executable='scan_node',
        name='scan_node',
        output='screen', # Ensures scan results are visible
        parameters=[{'use_sim_time': use_sim_time}]
    )

    bt_executor_node = Node(
        package='ms03_turtlebot3_control',
        executable='bt_executor_node',
        name='bt_executor_node',
        output='screen', # Ensures BT logs are visible
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. Build and return LaunchDescription
    return LaunchDescription([
        gazebo_launch,
        odom_node,
        scan_node,
        bt_executor_node
    ])
