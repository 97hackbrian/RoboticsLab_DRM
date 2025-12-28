cmake_minimum_required(VERSION 3.8)

project(palletizer_v1_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf worlds models textures
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
