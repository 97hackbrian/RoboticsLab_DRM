cmake_minimum_required(VERSION 3.8)

project(palletizer_v1_description)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# Build visual_odometry_publisher executable
add_executable(visual_odometry_publisher
  src/visual_odometry_publisher.cpp
)

ament_target_dependencies(visual_odometry_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
)

# Build stereo_camera_info_publisher executable
add_executable(stereo_camera_info_publisher
  src/stereo_camera_info_publisher.cpp
)

ament_target_dependencies(stereo_camera_info_publisher
  rclcpp
  sensor_msgs
)

# Build pointcloud_transform_publisher executable
add_executable(pointcloud_transform_publisher
  src/pointcloud_transform_publisher.cpp
)

ament_target_dependencies(pointcloud_transform_publisher
  rclcpp
  sensor_msgs
)

# Install executables
install(TARGETS
  visual_odometry_publisher
  stereo_camera_info_publisher
  pointcloud_transform_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource directories
install(
  DIRECTORY config launch meshes urdf worlds models textures rviz2
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
