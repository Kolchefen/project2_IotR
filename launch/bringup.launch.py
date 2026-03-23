"""
Launch file for TurtleBot 4 reactive controller with SLAM and Nav2.

Launches three components:
  1. reactive_controller  - our subsumption-architecture node
  2. SLAM Toolbox          - online async mapping (best for RPi4)
  3. Nav2 stack            - full navigation (costmaps, planner, controller, recoveries)

To run on robot over network:
  ros2 launch turtlebot4_reactive_controller bringup.launch.py

Optional args:
  use_sim_time:=true       - set True if running in Gazebo
  slam_params:=/path.yaml  - custom SLAM Toolbox config
  nav2_params:=/path.yaml  - custom Nav2 params file
  sync:=true               - use synchronous SLAM (better maps, heavier CPU)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # == Declare launch arguments ==
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    sync_arg = DeclareLaunchArgument(
        'sync',
        default_value='false',
        description='Use synchronous SLAM',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    sync = LaunchConfiguration('sync')

    # == 1. Reactive controller node ==
    reactive_controller_node = Node(
        package='turtlebot4_reactive_controller',
        executable='reactive_controller',
        name='reactive_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # == 2. SLAM Toolbox (via turtlebot4_navigation) ==
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'slam.launch.py',
            ])
        ),
        launch_arguments={
            'sync': sync,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    # == 3. Rviz2 bringup ==
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_viz'),
                'launch',
                'view_navigation.launch.py',
            ])
        ),
    )

    # == 4. Return Launch Description ==
    return LaunchDescription([
        use_sim_time_arg,
        sync_arg,
        reactive_controller_node,
        slam_launch,
        rviz_launch,
    ])