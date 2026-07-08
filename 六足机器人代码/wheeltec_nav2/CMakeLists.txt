cmake_minimum_required(VERSION 3.8)
project(wheeltec_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)

install(DIRECTORY launch map param rviz DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(nav2_bringup)
ament_package()
