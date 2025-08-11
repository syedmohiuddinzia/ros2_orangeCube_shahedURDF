cmake_minimum_required(VERSION 3.10)
project(shahed)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/imu_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF and meshes
install(DIRECTORY
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
