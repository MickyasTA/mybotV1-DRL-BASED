#!/usr/bin/env python3
# Description: This file is used to launch the robot_state_publisher, joint_state_publisher_gui and rviz2 to display the robot model in RViz2.
#              The robot model is loaded from the urdf file located in the urdf folder of the urdf_and_meshes_of_the_robot package.
#              The RViz2 configuration file is located in the config folder of the urdf_and_meshes_of_the_robot package.
import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('urdf_and_meshes_of_the_robot')
    # Get the path to the urdf file
    urdf_file_name = 'urdf_and_meshes_of_the_robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('urdf_and_meshes_of_the_robot'),
        'urdf',
        urdf_file_name)
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'config', 'urdf.rviz')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf).read()}]
        ),

        # Conditional joint state publisher GUI
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ) if os.environ.get('DISPLAY') else None,
        
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('urdf_and_meshes_of_the_robot'), 'config', 'urdf.rviz')]
        ),

        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 
                            'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={
                # 'world': 'true',  # Empty world
                'verbose': 'true'
            }.items()
        ),

        # Node to spawn the robot model in Gazebo
        Node(
            package='gazebo_ros',
            executable='/usr/bin/python3',  # Force system Python
            arguments=['/opt/ros/humble/lib/gazebo_ros/spawn_entity.py', 
                    '-entity', 'urdf_and_meshes_of_the_robot', 
                    '-file', urdf, '-x', '0', '-y', '0', '-z', '0.5'],
            output='screen'
        ),
        # Node to control the robot model
        # The robot model is controlled by the robot_nodes_control.py file
        # The robot_nodes_control.py file is used to control the robot model
        Node(
            package='urdf_and_meshes_of_the_robot',
            executable='robot_nodes_control.py',
            name='robot_control_node',
            output='screen'
        ),

    ])
    
