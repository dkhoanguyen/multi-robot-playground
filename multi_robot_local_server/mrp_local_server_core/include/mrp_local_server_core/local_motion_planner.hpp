#ifndef MRP_LOCAL_SERVER_CORE_LOCAL_MOTION_PLANNER_HPP_
#define MRP_LOCAL_SERVER_CORE_LOCAL_MOTION_PLANNER_HPP_

#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "mrp_common/parameter_interface.hpp"

#include "mrp_comms_msgs/msg/member_state.hpp"

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
    // We should change laser scan to cost map in the future
    virtual void setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path) = 0;
    virtual void calculateVelocityCommand(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<nav_msgs::msg::Odometry> &members_odom,
        const sensor_msgs::msg::LaserScan &scan,
        const double &current_time,
        geometry_msgs::msg::Twist &vel_cmd) = 0;

    virtual void step(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<mrp_comms_msgs::msg::MemberState> &members_state,
        geometry_msgs::msg::Twist &vel_cmd) = 0;

    // For feedback
    virtual double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose) = 0;

    // For accessing
    virtual bool reachGoal() = 0;

    // For settingg parameters
    virtual void setParameter(const std::unordered_map<std::string, double> &param_map) = 0;
  };
}

#endif