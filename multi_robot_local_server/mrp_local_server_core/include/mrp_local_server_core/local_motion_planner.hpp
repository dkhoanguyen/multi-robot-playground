#ifndef MRP_LOCAL_SERVER_CORE_LOCAL_CONTROLLER_SERVER_HPP_
#define MRP_LOCAL_SERVER_CORE_LOCAL_CONTROLLER_SERVER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mrp_local_server_core
{
  class MotionPlannerInterface
  {
  public:
    virtual ~MotionPlannerInterface(){};
    virtual void initialise() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    // Target
    virtual void setWaypoints(const std::vector<geometry_msgs::msg::Pose> &waypoints) = 0;
    virtual void calculateVelocityCommand(
        const geometry_msgs::msg::Pose &current_pose,
        geometry_msgs::msg::Twist &vel_cmd) = 0;

    // Consider the intension of other robots
    virtual void setOthersOdom(const std::vector<nav_msgs::msg::Odometry> &others_odom) = 0;

    // Consider what the robot sees (laser scan)
    virtual void setLaserScan(const sensor_msgs::msg::LaserScan &scan) = 0;
  };
};

#endif