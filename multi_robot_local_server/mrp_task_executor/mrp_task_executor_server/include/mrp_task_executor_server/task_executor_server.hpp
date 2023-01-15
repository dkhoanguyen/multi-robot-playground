#ifndef MRP_TASK_EXECUTOR__TASK_EXECUTOR_SERVER_HPP_
#define MRP_TASK_EXECUTOR__TASK_EXECUTOR_SERVER_HPP_

#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <pluginlib/class_loader.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "mrp_common/lifecycle_node.hpp"
#include "mrp_common/service_client.hpp"
#include "mrp_common/action_server.hpp"
#include "mrp_common/action_client.hpp"
#include "mrp_common/logging.hpp"

#include "mrp_comms_msgs/msg/member_state.hpp"
#include "mrp_comms_msgs/srv/get_all_teams.hpp"
#include "mrp_comms_msgs/srv/get_members_in_team.hpp"

#include "mrp_task_msgs/msg/task.hpp"
#include "mrp_task_msgs/action/execute_task.hpp"

namespace mrp_task_executor
{
  class TaskExecutorServer : public mrp_common::LifecycleNode
  {
  public:
    TaskExecutorServer();
    ~TaskExecutorServer();

    bool initialise();
    bool start();
    bool stop();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state) override;

  protected:
    struct RobotState
    {
      mrp_comms_msgs::msg::MemberState member_state;
      std::recursive_mutex mtx;
      std::atomic<bool> ready{false};
    };

    // Robot related
    std::string robot_name_;

    std::string member_state_topic_name_;
    rclcpp_lifecycle::LifecyclePublisher<mrp_comms_msgs::msg::MemberState>::SharedPtr member_state_pub_;
    void createMemberStatePublisher();

    // Publisher for cmd_vel
    std::string robot_cmd_vel_topic_name_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr robot_cmd_vel_pub_;
    void createCmdVelPublisher();

    // Subscriber for getting current pose of the robot
    std::string robot_odom_topic_name_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub_;
    void createOdomSubscriber();

    // Subscriber for getting laser scan of the robot
    std::string robot_scan_topic_name_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr robot_scan_sub_;
    void createLaserScanSubscriber();

    std::shared_ptr<mrp_common::ActionClient<nav2_msgs::action::FollowPath>> follow_path_client_;
    void createFollowPathActionClient();
  };
}

#endif