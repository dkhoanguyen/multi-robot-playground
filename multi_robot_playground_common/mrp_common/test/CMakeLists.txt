find_package(std_srvs REQUIRED)
find_package(test_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

ament_add_gtest(test_action_server test_action_server.cpp)
ament_target_dependencies(test_action_server std_srvs test_msgs rclcpp_action)
target_link_libraries(test_action_server ${library_name})

ament_add_gtest(test_action_client test_action_client.cpp)
ament_target_dependencies(test_action_client std_srvs test_msgs rclcpp_action)
target_link_libraries(test_action_client ${library_name})

