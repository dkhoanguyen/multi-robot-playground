cmake_minimum_required(VERSION 3.5)
project(mrp_behavior_tree)

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

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(mrp_local_server_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(mrp_common REQUIRED)
find_package(mrp_orca REQUIRED)
find_package(mrp_common_msgs REQUIRED)
find_package(mrp_motion_planner_msgs REQUIRED)
find_package(mrp_comms_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_action
  mrp_local_server_core
  pluginlib
  geometry_msgs
  nav_msgs
  nav2_msgs
  mrp_common
  mrp_common_msgs
  mrp_orca
  mrp_motion_planner_msgs
  mrp_comms_msgs
  behaviortree_cpp_v3
)

set(library_name behavior_engine)
add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

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
