cmake_minimum_required(VERSION 3.8)
project(slam_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(rplidar_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz2 REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_bringup REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
