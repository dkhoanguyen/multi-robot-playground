#ifndef MRP_RVO__RVO_HPP_
#define MRP_RVO__RVO_HPP_

#include "mrp_local_server_core/local_motion_planner.hpp"

namespace mrp_motion_planner
{
  class RVO : public mrp_local_server_core::MotionPlannerInterface
  {
  public:
    RVO();
    virtual ~RVO();

    void initialise();
    void start();
    void stop();

    void setWaypoints(const std::vector<geometry_msgs::msg::Pose> &waypoints);
    void calculateVelocityCommand(
        const geometry_msgs::msg::Pose &current_pose,
        geometry_msgs::msg::Twist &vel_cmd);

    // Consider the intension of other robots
    void setOthersOdom(const std::vector<nav_msgs::msg::Odometry> &others_odom);

    // Consider what the robot sees (laser scan)
    void setLaserScan(const sensor_msgs::msg::LaserScan &scan);
  };
}

#endif