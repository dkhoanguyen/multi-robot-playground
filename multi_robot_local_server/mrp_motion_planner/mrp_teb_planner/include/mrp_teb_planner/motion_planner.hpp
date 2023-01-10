#ifndef MRP_TEB_PLANNER__MOTION_PLANNER_HPP_
#define MRP_TEB_PLANNER__MOTION_PLANNER_HPP_

#include <atomic>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include <kdl/frames.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include "mrp_local_server_core/local_motion_planner.hpp"
#include "mrp_common/parameter_interface.hpp"

#include "mrp_common/utils.hpp"
#include "mrp_comms_msgs/msg/member_state.hpp"

namespace mrp_teb_planner
{
  class MotionPlanner : public mrp_local_server_core::MotionPlannerInterface
  {
  public:
    MotionPlanner();
    ~MotionPlanner();

    void initialise();
    void start();
    void stop();

    void setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path);

    void step(
        const nav_msgs::msg::Odometry &current_odom,
        const std::vector<mrp_comms_msgs::msg::MemberState> &members_state,
        const sensor_msgs::msg::LaserScan &scan,
        geometry_msgs::msg::Twist &vel_cmd);

    // For feedback
    double getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose);

    // For accessing
    bool reachGoal();
  };
}
#endif