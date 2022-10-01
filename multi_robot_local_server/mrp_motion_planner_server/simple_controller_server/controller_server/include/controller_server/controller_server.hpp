#ifndef CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
#define CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mrp_common/lifecycle_node.hpp"

namespace mrp_motion_planner_server
{
  namespace controller_server
  {
    class ControllerServer : mrp_common::LifecycleNode
    {
    public:
      ControllerServer();
      virtual ~ControllerServer();

    protected:
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

      bool loadController(const std::string &controller_name);

      bool setWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> waypoints);
      bool getRobotCurrentPose(geometry_msgs::msg::PoseStamped &pose) const;
    };
  }
}

#endif