#ifndef MRP_LOCAL_SERVER_CORE__LOCAL_MOTION_PLANNER_HPP_
#define MRP_LOCAL_SERVER_CORE__LOCAL_MOTION_PLANNER_HPP_

#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

    virtual void step(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<mrp_comms_msgs::msg::MemberState> &members_state,
        const sensor_msgs::msg::LaserScan &scan,
        geometry_msgs::msg::Twist &vel_cmd) = 0;

    // For feedback
    virtual double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose) = 0;

    // For accessing
    virtual bool reachGoal() = 0;
  };
}

#endif