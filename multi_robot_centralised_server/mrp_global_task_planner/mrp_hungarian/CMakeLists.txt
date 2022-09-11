cmake_minimum_required(VERSION 3.5)
project(mrp_hungarian)

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
find_package(mrp_centralised_server_core REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  mrp_centralised_server_core
  pluginlib
)


set(library_name mrp_hungarian)
add_library(${library_name} SHARED
  src/mrp_hungarian.cpp
)

set(executable_name mrp_hungarian_node)
add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${library_name} ${dependencies})


add_library(hungarian_algorithm SHARED hungarian_algorithm/hungarian.cpp)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} 
  ${library_name}
  hungarian_algorithm
)

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugins.xml
  DESTINATION share
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name}
  hungarian_algorithm)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(mrp_centralised_server_core plugins.xml)
ament_package()
