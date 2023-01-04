cmake_minimum_required(VERSION 3.5)
project(mrp_pure_pursuit)

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
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(mrp_local_server_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mrp_common REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  mrp_local_server_core
  pluginlib
  geometry_msgs
  Eigen3
  mrp_common
  orocos_kdl
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/pure_pursuit_controller.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugins.xml
  DESTINATION share
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # Automatic inclusion of lint-related tests
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  # Test ORCA
  # ament_add_gtest(test_orca test/test_orca.cpp) 
  # target_link_libraries(test_orca ${library_name})

  # Test Linear solver
  # ament_add_gtest(test_solver test/test_solver.cpp)
  # target_link_libraries(test_solver ${library_name})

  # Test planner
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(mrp_local_server_core plugins.xml)
ament_package()