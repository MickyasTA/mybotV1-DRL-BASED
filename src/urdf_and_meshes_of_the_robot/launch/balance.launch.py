#!/usr/bin/env python3
"""
Bring-up for the self-balance milestone.

Launches (headless by default):
  * Gazebo Classic (gzserver) with worlds/balance.world  (init+factory services on)
  * robot_state_publisher with the ros2_control balance description
  * spawn of the robot
  * controller spawners: joint_state_broadcaster -> wheel_effort_controller + leg_position_controller

Then, in a SEPARATE terminal (conda env with torch), run the trainer:
    python scripts/ppo_balance.py

Usage:
    ros2 launch urdf_and_meshes_of_the_robot balance.launch.py            # headless (training)
    ros2 launch urdf_and_meshes_of_the_robot balance.launch.py gui:=true  # watch in Gazebo GUI
"""
import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('urdf_and_meshes_of_the_robot')

    xacro_file = os.path.join(pkg, 'urdf', 'mybot_balance.urdf.xacro')
    controllers_file = os.path.join(pkg, 'config', 'controllers.yaml')
    gazebo_params_file = os.path.join(pkg, 'config', 'gazebo_params.yaml')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    spawn_z = LaunchConfiguration('spawn_z')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # world1.world is the real room (walls, doors, staircase, cabinet) — gives the
    # periodic_recorder a real environment to film. balance.world is an empty flat
    # plane (faster, no scenery) if you only care about raw balance throughput.
    world_file = PathJoinSubstitution([pkg, 'worlds', world])

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Run the Gazebo GUI (gzclient). Headless (false) is faster for training.')
    declare_world = DeclareLaunchArgument(
        'world', default_value='world1.world',
        description='World file in worlds/: world1.world (real room+stairs) or balance.world (flat).')
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z', default_value='0.53',
        description='Initial spawn height (m); FK puts wheel contact ~0.52 m below base_link.')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use /clock from Gazebo.')

    # robot_description from xacro, with the controllers path injected.
    # IMPORTANT: gazebo_ros2_control forwards robot_description to the controller_manager
    # via a --param CLI rule, and current Humble's rcl rejects MULTI-LINE param values
    # ("Couldn't parse parameter override rule"). So we strip the <?xml?> decl + comments
    # and collapse all whitespace to a single line. XML is whitespace-insensitive between
    # tags, so this is safe and makes the forwarded --param parse cleanly.
    # lock_legs='false' (default) => legs are REVOLUTE + effort-controlled; balance_env.py
    # holds them at the 0 stance with a per-joint PD->effort, so they're separately
    # controlled and rigid enough for balancing, and can be RL-driven later for walking.
    # 'true' => legs FIXED (kept only as a fallback; wheels alone hold the body).
    lock_legs = os.environ.get('LOCK_LEGS', 'false')
    _doc = xacro.process_file(xacro_file, mappings={'controllers_file': controllers_file,
                                                    'lock_legs': lock_legs})
    _urdf = _doc.toxml()
    _urdf = re.sub(r'<\?xml[^>]*\?>', '', _urdf)
    _urdf = re.sub(r'<!--.*?-->', '', _urdf, flags=re.S)
    # Resolve mesh URIs to absolute file:// paths so BOTH gzserver and gzclient find
    # them even when GAZEBO_MODEL_PATH/RESOURCE_PATH are empty (otherwise only the
    # primitive shapes render and the mesh links are invisible).
    _urdf = _urdf.replace('package://urdf_and_meshes_of_the_robot', 'file://' + pkg)
    robot_description = ' '.join(_urdf.split())

    # Gazebo: include gzserver.launch.py DIRECTLY (not the gazebo.launch.py wrapper)
    # so we can hand it a params_file. gzserver.launch.py loads init+factory plugins
    # by default -> pause/unpause/reset/spawn services.
    #
    # The params_file sets the `gazebo` node's publish_rate=200 Hz (sim time). The
    # gazebo_ros default is 10 Hz -> 0.1 s /clock ticks, far coarser than our 0.02 s
    # (50 Hz) control step; that made the *effective* control rate 10 Hz (the robot
    # tips faster than that) and quantized stepping. A string passed via
    # extra_gazebo_args is NOT shell-split by ExecuteProcess, so a params_file is the
    # reliable way to pass it.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'pause': 'false',
            'params_file': gazebo_params_file,
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'urdf_and_meshes_of_the_robot',
            '-z', spawn_z,
            '-timeout', '120.0',
        ],
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    wheel_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['wheel_effort_controller', '--controller-manager', '/controller_manager'])
    leg_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['leg_effort_controller', '--controller-manager', '/controller_manager'])

    # Sequence: spawn robot -> joint_state_broadcaster -> wheel_effort_controller -> leg_position_controller
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[jsb_spawner]))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[wheel_spawner]))
    after_wheel = RegisterEventHandler(
        OnProcessExit(target_action=wheel_spawner, on_exit=[leg_spawner]))

    # world1.world is large; give gzserver time to finish loading it before spawning the
    # robot, else spawn_entity races a half-loaded world and dies ("Spawn service failed.
    # Exiting."), which leaves no controllers and the trainer wedges.
    delayed_spawn = TimerAction(period=10.0, actions=[spawn_entity])

    _actions = [
        declare_gui,
        declare_world,
        declare_spawn_z,
        declare_sim_time,
        gzserver,
        gzclient,
        robot_state_publisher,
        delayed_spawn,
        after_spawn,
        after_jsb,
    ]
    # Spawn the leg controller ONLY when the legs are revolute (walking stage). With
    # lock_legs=true the legs are fixed + absent from ros2_control, so the controller
    # would fail to load.
    if lock_legs != 'true':
        _actions.append(after_wheel)
    return LaunchDescription(_actions)
