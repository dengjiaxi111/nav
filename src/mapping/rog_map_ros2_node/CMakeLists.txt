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

# Stair Detector Node
add_executable(stair_detector_node
  src/stair_detector.cpp
)

target_link_libraries(stair_detector_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(stair_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# Link with generated message library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stair_detector_node "${cpp_typesupport_target}")

install(TARGETS rog_map_node rog_map_component stair_detector_node
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
