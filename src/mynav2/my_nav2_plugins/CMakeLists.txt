cmake_minimum_required(VERSION 3.8)
project(my_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)


# === 1. 编译行为插件 ===
add_library(my_nav2_bt SHARED
  src/back_to_freespace.cpp
)
target_include_directories(my_nav2_bt PUBLIC include)

ament_target_dependencies(my_nav2_bt
  rclcpp
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  nav2_costmap_2d
  pluginlib
)

# === 安装插件 ===
install(
  TARGETS my_nav2_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装插件 XML 配置文件
install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
# !!注意！！ 下面这行不可少
target_compile_definitions(my_nav2_bt PRIVATE BT_PLUGIN_EXPORT)
ament_export_libraries(my_nav2_bt)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
