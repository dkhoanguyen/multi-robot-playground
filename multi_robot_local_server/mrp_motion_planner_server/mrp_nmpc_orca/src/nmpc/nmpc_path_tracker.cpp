#include "mrp_nmpc_orca/nmpc_path_tracker.hpp"

namespace mrp_nmpc_orca
{
  NMPCPathTracker::NMPCPathTracker()
  {
  }

  NMPCPathTracker::~NMPCPathTracker()
  {
  }

  void NMPCPathTracker::start()
  {
  }
  void NMPCPathTracker::stop()
  {
  }

  void NMPCPathTracker::setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    path_ = path;
    current_waypoint_indx_ = 0;
    at_position_ = false;
    reach_goal_ = false;
  }

  void NMPCPathTracker::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      const sensor_msgs::msg::LaserScan &scan,
      geometry_msgs::msg::Twist &vel_cmd)
  {
    if (current_waypoint_indx_ == path_.size())
    {
      return;
    }
  }

} // namespace mrp