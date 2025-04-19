cmake_minimum_required(VERSION 3.8)
project(urdf_and_meshes_of_the_robot)

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


find_package(robot_state_publisher REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# ament_python_install_package(scripts)

# Create a symlink for the Python scripts with correct naming

# Install Python scripts
install(PROGRAMS
  scripts/robot_nodes_control.py
  scripts/train_rl_ppo.py
  scripts/env.py
  DESTINATION lib/${PROJECT_NAME}
)

# Also install them without the .py extension
# install(PROGRAMS
#   scripts/robot_nodes_control.py 
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME robot_nodes_control
# )

# install(PROGRAMS
#   scripts/train_rl_ppo.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME train_rl_ppo
# )

# Install launch files
install(DIRECTORY
  launch
  urdf
  config
  meshes
  worlds
  resources
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()