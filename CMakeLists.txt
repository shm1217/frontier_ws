cmake_minimum_required(VERSION 3.8)
project(frontier_ws)

# 컴파일 옵션
if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


add_executable(frontier_multi
  src/frontier_multi.cpp
)

ament_target_dependencies(frontier_multi
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  nav2_msgs
)

install(TARGETS frontier_multi
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config lua
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/cloud_merge_to_octomap.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

