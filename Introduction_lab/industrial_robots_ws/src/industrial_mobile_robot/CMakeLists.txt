cmake_minimum_required(VERSION 3.5)
project(industrial_mobile_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes config worlds textures models
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

