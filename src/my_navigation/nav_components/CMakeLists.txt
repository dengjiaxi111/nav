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

# ========== NMPC acados solver 集成 ==========
set(ACADOS_SOLVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/nmpc_solver)

if(EXISTS ${ACADOS_SOLVER_DIR}/libacados_ocp_solver_wheelleg_nmpc.so)
    message(STATUS "✓ 找到 NMPC solver: ${ACADOS_SOLVER_DIR}")
    
    # 添加 acados include 路径
    include_directories(${ACADOS_SOLVER_DIR})
    
    # 链接 solver 库
    add_library(acados_nmpc_solver SHARED IMPORTED)
    set_target_properties(acados_nmpc_solver PROPERTIES
        IMPORTED_LOCATION ${ACADOS_SOLVER_DIR}/libacados_ocp_solver_wheelleg_nmpc.so
    )
    
    set(HAS_NMPC TRUE)
else()
    message(WARNING "⚠️  未找到 NMPC solver, 跳过 NMPC 控制器编译")
    message(WARNING "    请先运行: cd model_ocp && python3 export_ocp.py")
    set(HAS_NMPC FALSE)
endif()

# 导航服务器节点
set(NAV_SERVER_SOURCES
  src/nav_server.cpp
  src/simple_planner.cpp
  src/pure_pursuit.cpp
  src/backup_recovery.cpp
  src/spin_recovery.cpp
  src/recovery_manager.cpp
  src/layered_map_manager.cpp
)

# 如果有 NMPC solver, 添加 NMPC 源文件
if(HAS_NMPC)
    list(APPEND NAV_SERVER_SOURCES src/nmpc.cpp)
    add_compile_definitions(HAS_NMPC_CONTROLLER)
endif()



add_executable(nav_server ${NAV_SERVER_SOURCES})
ament_target_dependencies(nav_server ${DEPS})
target_link_libraries(nav_server yaml-cpp)

target_include_directories(nav_server
  PRIVATE
    /usr/local/include
    /usr/local/include/blasfeo/include
    /usr/local/include/hpipm/include
)

# 链接 acados solver
if(HAS_NMPC)
    target_link_libraries(nav_server 
        acados_nmpc_solver
        /usr/local/lib/libacados.so
        /usr/local/lib/libhpipm.so
        /usr/local/lib/libblasfeo.so
    )
endif()

install(TARGETS nav_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# 安装 NMPC 配置文件
if(HAS_NMPC)
    install(DIRECTORY model_ocp/
        DESTINATION share/${PROJECT_NAME}/nmpc_config
        FILES_MATCHING PATTERN "*.yaml"
    )
endif()

ament_package()
