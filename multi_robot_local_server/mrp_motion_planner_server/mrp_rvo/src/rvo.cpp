#include "mrp_rvo/rvo.hpp"

namespace mrp_motion_planner
{
  RVO::RVO()
  {
  }

  RVO::~RVO()
  {
  }

  void RVO::initialise()
  {
  }

  void RVO::start()
  {
  }

  void RVO::stop()
  {
  }

  void RVO::setWaypoints(const std::vector<geometry_msgs::msg::Pose> &waypoints)
  {
  }
  void RVO::calculateVelocityCommand(
      const geometry_msgs::msg::Pose &current_pose,
      geometry_msgs::msg::Twist &vel_cmd)
  {
  }

  void RVO::setOthersOdom(const std::vector<nav_msgs::msg::Odometry> &others_odom)
  {
  }

  void RVO::setLaserScan(const sensor_msgs::msg::LaserScan &scan)
  {
  }
}