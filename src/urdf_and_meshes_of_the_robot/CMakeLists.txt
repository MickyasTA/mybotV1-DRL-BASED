cmake_minimum_required(VERSION 3.8)
project(urdf_and_meshes_of_the_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find core build dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Only core message packages need to be REQUIRED during build
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Handle runtime dependencies differently
# These might not have CMake modules but are still needed at runtime
# They're declared in package.xml and will be verified during package installation

# Install Python scripts directly without using ament_python_install_package
install(PROGRAMS
  scripts/robot_nodes_control.py
  scripts/train_rl_ppo.py
  scripts/env.py
  scripts/robot_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create executable symlinks
foreach(script robot_nodes_control train_rl_ppo env)
  install(CODE
    "execute_process(
      COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${script}.py
      ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${script}
    )"
  )
endforeach()

# Check which directories exist and install them conditionally
foreach(dir launch urdf config meshes worlds)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME}/)
  endif()
endforeach()

# Handle resource directory specially (not resources)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/resource)
  install(DIRECTORY resource DESTINATION share/${PROJECT_NAME}/)
endif()

# Create a marker file for the package index
# First ensure the resource directory exists
install(CODE "file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/resource)")
install(CODE "file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/resource/${PROJECT_NAME} \"\")")
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# List runtime dependencies here for clarity
# These are verified through package.xml
ament_package()