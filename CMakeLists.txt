cmake_minimum_required(VERSION 3.8)
project(frontier_ws)

# 컴파일 옵션
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



add_executable(frontier_node
  src/frontier_db.cpp
)

ament_target_dependencies(frontier_node
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

install(TARGETS frontier_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

