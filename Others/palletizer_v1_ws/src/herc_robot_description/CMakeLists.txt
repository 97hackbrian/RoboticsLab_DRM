cmake_minimum_required(VERSION 3.5)
project(herc_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes config worlds textures
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

