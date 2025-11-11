cmake_minimum_required(VERSION 3.8)
project(rog_map_ros2_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rog_map REQUIRED)

add_executable(rog_map_node src/rog_map_node.cpp)

# Define necessary macros for rog_map
target_compile_definitions(rog_map_node PRIVATE 
  USE_ROS2
  ORIGIN_AT_CORNER
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

ament_target_dependencies(rog_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  rog_map
)

install(TARGETS rog_map_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
