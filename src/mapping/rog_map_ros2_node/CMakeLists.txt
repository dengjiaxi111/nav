cmake_minimum_required(VERSION 3.8)
project(rog_map_ros2_node)

# 开启 Release 模式和 O3 优化
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rog_map REQUIRED)
find_package(robots_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StairTarget.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# ROG-Map node (standalone executable，保留用于独立运行)
add_executable(rog_map_node 
  src/rog_map_node.cpp
  src/map_2d_projector.cpp
)

# Define necessary macros for rog_map
target_compile_definitions(rog_map_node PRIVATE 
  USE_ROS2
  ORIGIN_AT_CORNER
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

target_link_libraries(rog_map_node
  ${PCL_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(rog_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  rog_map
  robots_msgs
)

# ROG-Map Component (可组合节点，支持进程内零拷贝)
add_library(rog_map_component SHARED
  src/rog_map_component.cpp
)

target_compile_definitions(rog_map_component PRIVATE
  USE_ROS2
  ORIGIN_AT_CORNER
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

ament_target_dependencies(rog_map_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  rog_map
)

rclcpp_components_register_node(rog_map_component
  PLUGIN "rog_map_ros2_node::ROGMapComponent"
  EXECUTABLE rog_map_component_node
)

# Map2DProjector Component (可组合节点)
add_library(map_2d_projector_component SHARED
  src/map_2d_projector.cpp
)

target_compile_definitions(map_2d_projector_component PRIVATE
  USE_ROS2
  ORIGIN_AT_CORNER
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

ament_target_dependencies(map_2d_projector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  rog_map
  robots_msgs
)

target_link_libraries(map_2d_projector_component
  ${PCL_LIBRARIES}
  yaml-cpp
)

rclcpp_components_register_node(map_2d_projector_component
  PLUGIN "map_2d_projector::Map2DProjector"
  EXECUTABLE map_2d_projector_node
)

# Stair Detector Component (可组合节点)
add_library(stair_detector_component SHARED
  src/stair_detector.cpp
)

target_compile_definitions(stair_detector_component PRIVATE
  USE_ROS2
  ORIGIN_AT_CORNER
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

ament_target_dependencies(stair_detector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  rog_map
)

target_link_libraries(stair_detector_component
  ${PCL_LIBRARIES}
)

# Link with generated message library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stair_detector_component "${cpp_typesupport_target}")

rclcpp_components_register_node(stair_detector_component
  PLUGIN "stair_detector::StairDetector"
  EXECUTABLE stair_detector_node
)

# Integration Component (集成组件，管理所有子组件)
add_library(integration_component SHARED
  src/integration_component.cpp
)

target_compile_definitions(integration_component PRIVATE
  USE_ROS2
  ORIGIN_AT_CORNER
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

ament_target_dependencies(integration_component
  rclcpp
  rclcpp_components
  rog_map
)

target_link_libraries(integration_component
  rog_map_component
  map_2d_projector_component
  stair_detector_component
)

rclcpp_components_register_node(integration_component
  PLUGIN "rog_map_ros2_node::IntegrationComponent"
  EXECUTABLE integration_node
)

add_executable(lidar_roll_calibrator
  src/lidar_roll_calibrator.cpp
)

target_link_libraries(lidar_roll_calibrator
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lidar_roll_calibrator
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  livox_ros_driver2
)

install(TARGETS 
  rog_map_node 
  lidar_roll_calibrator
  rog_map_component 
  map_2d_projector_component
  stair_detector_component
  integration_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
