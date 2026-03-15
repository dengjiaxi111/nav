cmake_minimum_required(VERSION 3.8)
project(sentry_decision)

# =========================
# C++ 标准
# =========================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# 查找依赖
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 消息生成
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# 外部消息包
find_package(decision_messages REQUIRED)

# =========================
# 生成自定义消息
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SentryControl.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# =========================
# include 目录
# =========================
include_directories(
  include
)

# =========================
# 决策库
# =========================
add_library(sentry_decision_lib
  src/Blackboard.cpp
  src/Models.cpp
  src/RegionManager.cpp
  src/DecisionManager.cpp
  src/ActionNodes.cpp
  src/ConditionNodes.cpp
  src/TreeBuilder.cpp
)

target_include_directories(sentry_decision_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sentry_decision_lib
  rclcpp
  geometry_msgs
  std_msgs
  decision_messages
  rosidl_default_runtime
)

# 正确方式：让库使用本包 msg
target_link_libraries(sentry_decision_lib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# =========================
# 主节点
# =========================
add_executable(sentry_decision_node
  src/main.cpp
)

target_include_directories(sentry_decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sentry_decision_node
  rclcpp
  geometry_msgs
  std_msgs
  decision_messages
  rosidl_default_runtime
)

target_link_libraries(sentry_decision_node
  sentry_decision_lib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# =========================
# 安装
# =========================
install(TARGETS
  sentry_decision_node
  sentry_decision_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(sentry_decision_lib)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_msgs
  decision_messages
  rosidl_default_runtime
)

ament_package()
