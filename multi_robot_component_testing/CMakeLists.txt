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
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mrp_common REQUIRED)
find_package(mrp_common_msgs REQUIRED)

# include_directories(
#   include
# )

#############################
# COMPONENT TESTING LIBRARY #
#############################

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rclcpp_action
  std_srvs
  turtlesim
  mrp_common
  mrp_common_msgs
)

set(library_name multi_robot_component_testing)
add_library(${library_name} SHARED
  src/minimal_service_server.cpp
  # src/minimal_service_client.cpp
  src/minimal_action_server.cpp
  src/minimal_action_client.cpp
  src/lifecycle_node.cpp
)

target_include_directories(${library_name} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${library_name} ${dependencies})

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

##########################
# MINIMAL SERVICE SERVER #
##########################

set(minimal_service_server minimal_service_server)
add_executable(${minimal_service_server}
  exec/minimal_service_server.cpp
)

target_link_libraries(${minimal_service_server} 
  ${library_name}
)

target_include_directories(${minimal_service_server} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${minimal_service_server} ${dependencies})

install(TARGETS ${minimal_service_server}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

##########################
# MINIMAL SERVICE CLIENT #
##########################

# set(minimal_service_client minimal_service_client)
# add_executable(${minimal_service_client}
#   exec/minimal_service_client.cpp
# )

# target_link_libraries(${minimal_service_client} 
#   ${library_name}
# )

# target_include_directories(${minimal_service_client} PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(${minimal_service_client} ${dependencies})

# install(TARGETS ${minimal_service_client}
#   RUNTIME DESTINATION lib/${PROJECT_NAME})

##########################
# MINIMAL ACTION SERVER #
##########################

set(minimal_action_server minimal_action_server)
add_executable(${minimal_action_server}
  exec/minimal_action_server.cpp
)

target_link_libraries(${minimal_action_server} 
  ${library_name}
)

target_include_directories(${minimal_action_server} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${minimal_action_server} ${dependencies})

install(TARGETS ${minimal_action_server}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

##########################
# MINIMAL ACTION CLIENT #
##########################

set(minimal_action_client minimal_action_client)
add_executable(${minimal_action_client}
  exec/minimal_action_client.cpp
)

target_link_libraries(${minimal_action_client} 
  ${library_name}
)

target_include_directories(${minimal_action_client} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${minimal_action_client} ${dependencies})

install(TARGETS ${minimal_action_client}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

###################
# LIFE CYCLE NODE #
###################

set(lifecycle_node lifecycle_node)
add_executable(${lifecycle_node}
  exec/lifecycle_node.cpp
)

target_link_libraries(${lifecycle_node} 
  ${library_name}
)

target_include_directories(${lifecycle_node} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${lifecycle_node} ${dependencies})

install(TARGETS ${lifecycle_node}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

################
# UNIT TESTING #
################

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
