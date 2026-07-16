cmake_minimum_required(VERSION 3.8)
project(mylog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(robots_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ========== C++ 节点 ==========
add_executable(recordlog src/recordlog.cpp)

ament_target_dependencies(recordlog
  rclcpp
  robots_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_msgs
  Boost
  spdlog
  ament_index_cpp
)

# 安装 C++ 节点
install(TARGETS
  recordlog
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Python 节点 ==========
install(PROGRAMS
  src/mylog.py
  DESTINATION lib/${PROJECT_NAME}
)


# ========== 测试相关 ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
