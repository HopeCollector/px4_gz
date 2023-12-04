cmake_minimum_required(VERSION 3.5)

project(px4_gz_application)

add_compile_options( -std=c++2a -O3)

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(visualization_helper_component SHARED src/visualization_helper.cc)
target_include_directories(visualization_helper_component PRIVATE
  ${CMAKE_INSTALL_PREFIX}/../yaLanTingLibs/include
  ${CMAKE_INSTALL_PREFIX}/../yaLanTingLibs/include/ylt/thirdparty
)
rclcpp_components_register_nodes(visualization_helper_component
  "px4_gz::visualization_helper"
)

ament_auto_add_library(odometry_component SHARED src/odometry.cc)
target_link_libraries(odometry_component Eigen3::Eigen)
rclcpp_components_register_nodes(odometry_component
  "px4_gz::odometry"
)

ament_auto_package()
