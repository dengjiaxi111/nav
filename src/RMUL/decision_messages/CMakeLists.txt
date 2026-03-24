cmake_minimum_required(VERSION 3.8)
project(decision_messages)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OurRobotState.msg"
  "msg/GameState.msg"
  DEPENDENCIES std_msgs
)

# 移除 ament_export_libraries，改为只导出依赖
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(std_msgs)

# 导出包含目录
ament_export_include_directories(include)

ament_package()
