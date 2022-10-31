#ifndef MRP_MOTION_PLANNER_SERVER__MOTION_PLANNER_SERVER_HPP_
#define MRP_MOTION_PLANNER_SERVER__MOTION_PLANNER_SERVER_HPP_

#include <chrono>
#include <iostream>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "mrp_common/lifecycle_node.hpp"
#include "mrp_common/service_server.hpp"

#include <pluginlib/class_loader.hpp>
#include "mrp_local_server_core/local_motion_planner.hpp"

#include "mrp_motion_planner_msgs/srv/waypoints.hpp"

namespace mrp_motion_planner
{
  class MotionPlannerServer : public mrp_common::LifecycleNode
  {
  public:
    MotionPlannerServer();
    virtual ~MotionPlannerServer();

    void initialise();
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
    std::shared_ptr<pluginlib::ClassLoader<mrp_local_server_core::MotionPlannerInterface>> loader_ptr_;

    std::string robot_names_;
    // std::vector<nav_msgs::msg::Odometry>

    void resgisterRobotOdomSub(const std::string &robot_name);
    void registerRobotsOdomSubs(const std::vector<std::string> &robot_names);
  };
}

#endif