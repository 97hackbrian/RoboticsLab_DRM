cmake_minimum_required(VERSION 3.8)

project(palletizer_v1_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python scripts as executables
install(
  PROGRAMS scripts/odom_tf_corrector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch meshes urdf worlds models textures
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()

