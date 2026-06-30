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
  # --- self-balance / wheel-legged RL stack (active) ---
  scripts/balance_env.py          # Gymnasium env (IMU tilt + wheel odometry)
  scripts/ppo_balance.py          # from-scratch PPO trainer + eval/play
  scripts/record_episode.py       # FK MP4 render of an episode
  scripts/periodic_recorder.py    # real headless-Gazebo video via spawned camera
  scripts/tb_plot_writer.py       # TensorBoard + live PNG graphs
  scripts/metrics_logger.py       # per-episode CSV + summary JSON
  scripts/plot_metrics.py         # overview-grid plotter
  scripts/camera_recorder.py      # (optional) onboard-camera MP4 helper
  # --- original baseline (kept for reference) ---
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

# NOTE: the package-index marker and package.xml install are handled
# automatically by ament_package() below. The previous custom install(CODE ...)
# resource-marker block was redundant and broke `colcon build --symlink-install`.
ament_package()