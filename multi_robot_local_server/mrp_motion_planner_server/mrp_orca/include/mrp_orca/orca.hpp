#ifndef MRP_ORCA__ORCA_HPP_
#define MRP_ORCA__ORCA_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "mrp_common/utils.hpp"
#include "mrp_orca/geometry.hpp"

namespace mrp_orca
{
  class ORCA
  {
  public:
    enum class Result
    {
      COLLISION,
      NO_IMMEDIATE_COLLISION,
      NO_COLLISION
    };

    static Result localConstruct(
        mrp_orca::geometry::HalfPlane &output_orca,
        const Eigen::Vector2d &desired_vel_A,
        const nav_msgs::msg::Odometry &odom_B,
        const double &radius_A, 
        const double &radius_B,
        const double &delta_tau,
        const double &weight);

  protected:
  };
}

#endif