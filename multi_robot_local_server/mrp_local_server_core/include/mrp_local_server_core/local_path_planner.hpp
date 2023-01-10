#ifndef MRP_LOCAL_SERVER_CORE__LOCAL_PATH_PLANNER_HPP_
#define MRP_LOCAL_SERVER_CORE__LOCAL_PATH_PLANNER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "mrp_comms_msgs/msg/member_state.hpp"

namespace mrp_local_server_core
{
  class PathPlannerInterface
  {
  public:
    virtual ~PathPlannerInterface(){};
    virtual void initialise() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual bool createPath(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::vector<geometry_msgs::msg::PoseStamped> &path) = 0;
  };
}

#endif