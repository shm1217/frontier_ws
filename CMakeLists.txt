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
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BboxImg.msg"
  "msg/EmbArray.msg"
  "msg/DynamicObstacle.msg"
  DEPENDENCIES std_msgs sensor_msgs
)



add_executable(frontier_multi
  src/frontier_multi.cpp
  src/frontier_multi_main.cpp
  src/controller.cpp
)

rosidl_target_interfaces(frontier_multi
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 헤더 파일 경로 포함 (include 폴더에 .hpp가 있는 경우)
target_include_directories(frontier_multi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${rclcpp_EXPORTED_TARGETS}
)

add_executable(detect_node
        src/detect_node.cpp)
ament_target_dependencies(detect_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  yolo_msgs
  tf2_eigen
  cv_bridge
  image_geometry
  OpenCV
)

rosidl_target_interfaces(detect_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(detect_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${rclcpp_EXPORTED_TARGETS}
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

install(TARGETS detect_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config lua
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
