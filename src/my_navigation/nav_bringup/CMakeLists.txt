cmake_minimum_required(VERSION 3.8)
project(nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/static_relocator.py
  scripts/odom_base_debug_rebaser.py
  scripts/omni_cmd_adapter.py
  scripts/omni_stage3_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
