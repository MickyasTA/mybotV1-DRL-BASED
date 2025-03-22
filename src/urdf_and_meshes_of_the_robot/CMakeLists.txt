cmake_minimum_required(VERSION 3.5)
project(urdf_and_meshes_of_the_robot)

# Use ament_cmake (ROS 2) instead of catkin (ROS 1)
find_package(ament_cmake REQUIRED)

# Install URDF, Meshes, and Launch Files
install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Required for ROS 2 package building
ament_package()
