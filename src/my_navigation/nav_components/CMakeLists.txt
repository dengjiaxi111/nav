cmake_minimum_required(VERSION 3.8)
project(nav_components)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(nav_core REQUIRED)
find_package(yaml-cpp REQUIRED)

set(DEPS
  rclcpp rclcpp_action geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs nav_interfaces nav_core
)

# 头文件目录
include_directories(include)

# 导航服务器节点
add_executable(nav_server
  src/nav_server.cpp
  src/simple_planner.cpp
  src/pure_pursuit.cpp
  src/backup_recovery.cpp
  src/spin_recovery.cpp
  src/recovery_manager.cpp
)
ament_target_dependencies(nav_server ${DEPS})
target_link_libraries(nav_server yaml-cpp)

install(TARGETS nav_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
