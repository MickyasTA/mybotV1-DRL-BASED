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

def generate_launch_description():
    urdf_file_name = 'urdf_and_meshes_of_the_robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('urdf_and_meshes_of_the_robot'),
        'urdf',
        urdf_file_name)

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
        # Gazebo simulation of the robot model with the urdf file and meshes 
        ExecuteProcess(
            cmd=['/usr/bin/gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='gazebo_ros',
            executable='/usr/bin/python3',  # Force system Python
            arguments=['/opt/ros/humble/lib/gazebo_ros/spawn_entity.py', 
                    '-entity', 'urdf_and_meshes_of_the_robot', 
                    '-file', urdf, '-x', '0', '-y', '0', '-z', '0.5'],
            output='screen'
        ),

    ])
    
