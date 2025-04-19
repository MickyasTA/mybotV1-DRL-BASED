import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, execute_process, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('urdf_and_meshes_of_the_robot')

    # launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    train_rl = LaunchConfiguration('train_rl', default='false')
    # world_file_name = LaunchConfiguration('world_file_name', default='empty')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true'
    )

    declare_train_rl = DeclareLaunchArgument(
        'train_rl',
        default_value='false',
        description='Launch RL training if true'
    )

    # include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')
        ]),
    )

    # launch RViz if conditionally
    rviz_node = Node(
        condition=launch.conditions.IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'robot.rviz')],
        parameters= [{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # launch RL training if conditionally
    rl_training_node = Node(
        condition=launch.conditions.IfCondition(train_rl),
        package='urdf_and_meshes_of_the_robot',
        executable='train_rl_ppo.py',
        name='rl_training_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_train_rl,
        gazebo_launch,
        rviz_node,
        rl_training_node,
    ])