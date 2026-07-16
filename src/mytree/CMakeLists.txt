cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
project(mytree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robots_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(mytree
  src/mytree.cpp
)

target_include_directories(mytree PUBLIC include)

ament_target_dependencies(mytree
  rclcpp
  behaviortree_cpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  robots_msgs
  nav_msgs
  nav2_msgs
  visualization_msgs
)

target_link_libraries(mytree
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)

install(TARGETS mytree
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config tree launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
