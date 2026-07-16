cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED) #并行计算

add_executable(relocalization src/relocalization.cpp)
add_executable(converge src/converge.cpp)

# 如果找到了 OpenMP，启用它
if(OpenMP_CXX_FOUND)
    message(WARNING "OpenMP found!")
    target_compile_options(relocalization PRIVATE ${OpenMP_CXX_FLAGS})  
    target_link_libraries(relocalization  ${OpenMP_CXX_LIBRARIES}) 
else()
    message(WARNING "\n!!!!!OpenMP not found!!!!!\n")
endif()

target_include_directories(relocalization 
PUBLIC
include
${PCL_INCLUDE_DIRS}
)

target_include_directories(converge 
PUBLIC
include
${PCL_INCLUDE_DIRS}
)

target_link_libraries(relocalization ${PCL_LIBRARIES})
target_link_libraries(converge ${PCL_LIBRARIES})

ament_target_dependencies(converge rclcpp sensor_msgs pcl_ros pcl_conversions tf2_ros Eigen3 geometry_msgs tf2_geometry_msgs) 
ament_target_dependencies(relocalization rclcpp sensor_msgs std_srvs pcl_ros pcl_conversions tf2_ros Eigen3 geometry_msgs tf2_geometry_msgs) 

install(TARGETS
relocalization
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS converge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


