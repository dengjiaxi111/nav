cmake_minimum_required(VERSION 3.10)
project(pointcloud_obstacle_layer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

set(PCL_INCLUDE_DIR /usr/include/pcl-1.12)
if(NOT EXISTS ${PCL_INCLUDE_DIR})
  message(FATAL_ERROR "Expected PCL include directory not found: ${PCL_INCLUDE_DIR}")
endif()

set(PCL_LINK_LIBS
  pcl_common
  pcl_filters
  pcl_features
  pcl_search
  pcl_kdtree
)

add_executable(pointcloud_segmentation_node
  src/pointcloud_segmentation_node.cpp
)
ament_target_dependencies(pointcloud_segmentation_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(pointcloud_segmentation_node
  ${PCL_LINK_LIBS}
)
target_include_directories(pointcloud_segmentation_node
  SYSTEM PRIVATE
    ${PCL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(local_obstacle_grid_node
  src/local_obstacle_grid_node.cpp
)
ament_target_dependencies(local_obstacle_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(local_obstacle_grid_node
  ${PCL_LINK_LIBS}
)
target_include_directories(local_obstacle_grid_node
  SYSTEM PRIVATE
    ${PCL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS
  pointcloud_segmentation_node
  local_obstacle_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
