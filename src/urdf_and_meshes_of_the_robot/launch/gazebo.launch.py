#!/usr/bin/env python3
# Description: This file is used to launch Gazebo with the robot model for reinforcement learning
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('urdf_and_meshes_of_the_robot')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='')
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'config', 'urdf.rviz')
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='',
        description='/home/mickyas/ros2_ws/src/urdf_and_meshes_of_the_robot/worlds/empty.world'
    )
    
    # Get the path to the URDF file
    urdf_file_name = 'urdf_and_meshes_of_the_robot.urdf'
    urdf_file_path = os.path.join(pkg_dir, 'urdf', urdf_file_name)
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file_path).read(),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'gui': 'true'
        }.items()
    )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'urdf_and_meshes_of_the_robot',
            '-file', urdf_file_path,
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch the robot control node
    robot_control_node = Node(
        package='urdf_and_meshes_of_the_robot',
        executable='robot_nodes_control.py',
        name='robot_control_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('urdf_and_meshes_of_the_robot'), 'config', 'urdf.rviz')]
    )

    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_world_file,
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        joint_state_publisher,
        robot_control_node,
        rviz_node,
    ])
