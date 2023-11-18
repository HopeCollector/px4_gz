cmake_minimum_required(VERSION 3.5)

project(px4_gz_application)

add_compile_options( -std=c++2a -O3)

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

ament_auto_add_library(rviz_mesh_loader_component SHARED src/rviz_mesh_loader.cc)
rclcpp_components_register_nodes(rviz_mesh_loader_component
  "px4_gz::rivz_mesh_loader"
)

ament_auto_package()
