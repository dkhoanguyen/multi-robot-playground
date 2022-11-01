#ifndef MRP_MOTION_PLANNER_SERVER__MOTION_PLANNER_SERVER_HPP_
#define MRP_MOTION_PLANNER_SERVER__MOTION_PLANNER_SERVER_HPP_

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

#include "mrp_common/lifecycle_node.hpp"
#include "mrp_common/service_server.hpp"
#include "mrp_common/action_server.hpp"
#include "mrp_common/logging.hpp"

#include "mrp_local_server_core/local_motion_planner.hpp"

#include "mrp_motion_planner_msgs/srv/waypoints.hpp"

namespace mrp_motion_planner
{
  class MotionPlannerServer : public mrp_common::LifecycleNode
  {
  public:
    MotionPlannerServer(const std::string &default_planner_name);
    virtual ~MotionPlannerServer();

    void initialise();
    void start();
    bool loadPlanner(const std::string &planner_name);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state) override;

    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    // on_deactivate(const rclcpp_lifecycle::State &state) override;

    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    // on_cleanup(const rclcpp_lifecycle::State &state) override;

    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    // on_shutdown(const rclcpp_lifecycle::State &state) override;

    bool setWaypoints(const std::vector<geometry_msgs::msg::Pose> waypoints);
    bool getRobotCurrentPose(geometry_msgs::msg::Pose &pose) const;

    void setOtherRobots(const std::string &robot_names);

  protected:
    struct RobotOdom
    {
      RobotOdom() : ready(false) {};
      nav_msgs::msg::Odometry current_odom;
      std::recursive_mutex mtx;
      std::atomic<bool> ready;
    };

    std::shared_ptr<pluginlib::ClassLoader<mrp_local_server_core::MotionPlannerInterface>> loader_ptr_;

    // Robot related
    std::string robot_name_;
    std::string planner_name_;
    std::map<std::string, std::string> planner_name_map_;

    // Robot Odom
    std::atomic<bool> robot_odom_ready_{false};
    std::recursive_mutex robot_odom_mtx_;
    nav_msgs::msg::Odometry robot_current_odom_;

    // Publisher for cmd_vel
    std::string robot_cmd_vel_topic_name_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr robot_cmd_vel_pub_;
    // Subscriber for getting current pose of the robot
    std::string robot_odom_topic_name_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub_;
    std::shared_ptr<mrp_common::ActionServer<nav2_msgs::action::FollowPath>> follow_path_action_server_;

    // Other robots related
    std::vector<std::string> other_robots_names_;
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> other_robots_odom_sub_map_;
    std::map<std::string, std::shared_ptr<RobotOdom>> other_robots_odom_data_map_;

    void resgisterRobotOdomSub(const std::string &robot_name);
    void registerRobotsOdomSubs(const std::vector<std::string> &robot_names);

    void followPath();
  };
}

#endif