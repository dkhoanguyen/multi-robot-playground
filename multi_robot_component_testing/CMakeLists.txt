cmake_minimum_required(VERSION 3.5)
project(multi_robot_component_testing)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(multi_robot_playground_common REQUIRED)

# include_directories(
#   include
# )

set(dependencies
  rclcpp
  rclcpp_components
  std_srvs
  multi_robot_playground_common
)

set(library_name multi_robot_component_testing)
add_library(${library_name} SHARED
  src/minimal_service_server.cpp
)

target_include_directories(${library_name} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${library_name} ${dependencies})

set(executable_name minimal_service_server)
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} 
  ${library_name}
  # multi_robot_playground_common::multi_robot_playground_common
)

target_include_directories(${executable_name} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${executable_name} ${dependencies})

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

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
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
