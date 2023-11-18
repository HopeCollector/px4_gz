cmake_minimum_required(VERSION 3.5)

project(px4_gz_application)

add_compile_options( -std=c++2a -O3)

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

ament_auto_add_library(visualization_helper_component SHARED src/visualization_helper.cc)
rclcpp_components_register_nodes(visualization_helper_component
  "px4_gz::visualization_helper"
)

ament_auto_package()
