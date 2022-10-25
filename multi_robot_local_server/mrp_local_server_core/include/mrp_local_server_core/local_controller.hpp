#ifndef MRP_LOCAL_SERVER_CORE_LOCAL_CONTROLLER_SERVER_HPP_
#define MRP_LOCAL_SERVER_CORE_LOCAL_CONTROLLER_SERVER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace local_server_core
{
  class LocalController
  {
  public:
    virtual ~LocalController(){};
    virtual void initialise() = 0;
    virtual void setWaypoints(const std::vector<geometry_msgs::msg::Pose> waypoints) = 0;
    virtual void calculateVelocityCommand(
      const geometry_msgs::msg::Pose &current_pose,
      geometry_msgs::msg::Twist &vel_cmd) = 0;
  };
} // namespace name

#endif