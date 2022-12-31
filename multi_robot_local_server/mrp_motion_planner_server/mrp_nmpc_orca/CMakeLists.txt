cmake_minimum_required(VERSION 3.5)
project(mrp_nmpc_orca)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(ifopt REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(fmt REQUIRED)
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
  ifopt
)

set(
  CGMRES_SOLVER_SRC
  src/cgmres_solver/continuation_gmres.cpp
  src/cgmres_solver/single_shooting_continuation.cpp
  src/cgmres_solver/single_shooting_ocp.cpp
  src/cgmres_solver/time_varying_smooth_horizon.cpp
  src/cgmres_solver/cgmres_initializer.cpp
  src/cgmres_solver/zero_horizon_ocp.cpp
  src/cgmres_solver/optimal_control_problem.cpp
  src/cgmres_solver/linear_algebra.cpp
)

set(cgmres_solver_lib_name cgmres_solver_lib)
add_library(${cgmres_solver_lib_name} SHARED
  ${CGMRES_SOLVER_SRC}
)

set(nmpc_related_lib_name ${PROJECT_NAME})
add_library(${nmpc_related_lib_name} SHARED
  src/nmpc/course_manager.cpp
  src/nmpc/frenet_serret_converter.cpp
  src/nmpc/frenet_state_filter.cpp
  src/nmpc/mpc_simulator.cpp
  src/nmpc/nmpc_formulation.cpp
  src/nmpc/nmpc_path_tracker.cpp
)

ament_target_dependencies(${nmpc_related_lib_name} ${dependencies})
target_link_libraries(${nmpc_related_lib_name}
${cgmres_solver_lib_name}
)

install(
  TARGETS ${cgmres_solver_lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${nmpc_related_lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  ${cgmres_solver_lib_name}
  ${nmpc_related_lib_name}
)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(mrp_local_server_core plugins.xml)
ament_package()
