cmake_minimum_required(VERSION 3.8)
project(localization_initializer)

# ==================== 编译优化选项 ====================
# ⚠️ 临时使用 Debug 模式排查 -O3 导致的 crash
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Release 模式: 保守优化 (-O2)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
# Debug 模式: 禁用优化，启用调试符号
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# 编译选项 (移除激进优化)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic  # 警告选项
    # 不使用 -O3/-march=native/-ffast-math (避免栈溢出和内存对齐问题)
  )
endif()

message(STATUS "🔧 Localization Initializer 保守优化模式:")
message(STATUS "   - 构建类型: ${CMAKE_BUILD_TYPE}")
message(STATUS "   - 优化级别: -O2 (移除 -O3/-march=native/-ffast-math)")
message(STATUS "   - 原因: 避免大点云处理时的栈溢出和内存对齐问题")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== 依赖包 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration segmentation features search kdtree)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# pclomp (自建包)
find_package(PCLOMP REQUIRED)


# ==================== 可执行文件 ====================
add_executable(localization_initializer_node
  src/localization_initializer_node.cpp
)

target_include_directories(localization_initializer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(localization_initializer_node
  ${PCL_LIBRARIES}
  pclomp::registration
)

ament_target_dependencies(localization_initializer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
)

# ==================== 安装 ====================
install(TARGETS
  localization_initializer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ==================== Ament 打包 ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
