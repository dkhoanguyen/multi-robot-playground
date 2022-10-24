#ifndef CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
#define CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_

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
#include "mrp_local_server_core/local_controller.hpp"
#include "spotturn_controller/spotturn_controller.hpp"

#include "controller_server/srv/waypoints.hpp"

namespace mrp_motion_planner_server
{
  namespace controller_server
  {
    class ControllerServer : mrp_common::LifecycleNode
    {
    public:
      ControllerServer();
      virtual ~ControllerServer();

      void initialise();
      bool loadController(const std::string &controller_name);
      void start();

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

    protected:
      std::string robot_name_;

      std::string cmd_vel_topic_;
      std::string controller_name_;
      std::string cmd_topic_vel_;

      std::shared_ptr<local_server_core::LocalController> controller_ptr_;
      std::map<std::string, std::string> controller_name_map_;

      std::recursive_mutex odom_mtx_;
      nav_msgs::msg::Odometry current_odom_;

      // Publisher for cmd_vel
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
      // Timer callback for velocity publisher
      rclcpp::TimerBase::SharedPtr cmd_vel_pub_timer_;

      // Subscriber for getting current pose of the robot
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

      // Services for waypoints and controller modifier
      // std::shared_ptr<mrp_common::ServiceServer<>> ;

      void followWaypoints(); 
      void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    };
  }
}

#endif