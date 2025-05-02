import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, execute_process, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

"""
conda activate ros2_rl_env
tensorboard --logdir=./PPO_4
"""

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('urdf_and_meshes_of_the_robot')
    
    # Path to the Gazebo world file
    world_file = os.path.join(pkg_dir,'worlds', 'world1.world')
    # launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # use_rviz = LaunchConfiguration('use_rviz', default='true')
    train_rl = LaunchConfiguration('train_rl', default='false')
    # world_file = LaunchConfiguration('world_file', default='world1.world')
    
    # Physics parameters
    physics_params = {'physics': 'ode',
                     'max_step_size': '0.001',
                     'real_time_factor': '1.0',
                     'real_time_update_rate': '1000.0'}
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # declare_use_rviz = DeclareLaunchArgument(
    #     'use_rviz',
    #     default_value='true',
    #     description='Launch RViz if true'
    # )

    declare_train_rl = DeclareLaunchArgument(
        'train_rl',
        default_value='false',
        description='Launch RL training if true'
    )

    # Get the path to the world file
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='',
        description='/home/mickyas/ros2_ws/src/urdf_and_meshes_of_the_robot/worlds/world1.world'
    )


    # include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
                'physics': physics_params['physics'],
                'max_step_size': physics_params['max_step_size'],
                'real_time_factor': physics_params['real_time_factor'],
                'real_time_update_rate': physics_params['real_time_update_rate'],
                'verbose': 'false'
        }.items()
    )


    # launch RViz if conditionally
    # rviz_node = Node(
    #     condition=launch.conditions.IfCondition(use_rviz),
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(pkg_dir, 'config', 'robot.rviz')],
    #     parameters= [{'use_sim_time': use_sim_time}],
    #     output='screen',
    # )

    # launch RL training if conditionally
    rl_training_node = Node(
        condition=launch.conditions.IfCondition(train_rl),
        package='urdf_and_meshes_of_the_robot',
        executable='/home/mickyas/miniconda3/envs/ros2_rl_env/bin/python',
        arguments=[
            '/home/mickyas/ros2_ws/install/urdf_and_meshes_of_the_robot/lib/urdf_and_meshes_of_the_robot/train_rl_ppo.py',
            # Add any arguments your script expects
            # '--timesteps', '1000000',
            # '--save-freq', '10000',
        ],
        name='rl_training_node',
        output='screen',
        # Remove 'parameters' as they're converted to ROS args that your script doesn't understand
        # parameters=[{'use_sim_time': use_sim_time}],
    )

    # return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_world_file,
        # declare_use_rviz,
        declare_train_rl,
        gazebo_launch,
        # rviz_node,
        rl_training_node,
    ])