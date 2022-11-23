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
    // Construct ORCA half plane of B induced by A, given
    // A current position and velocity vector
    // B current position and velocity vector
    static bool construct(
        mrp_orca::geometry::HalfPlane &orca_plane,
        const nav_msgs::msg::Odometry &odom_A,
        const nav_msgs::msg::Odometry &odom_B,
        const double &radius_A, const double &radius_B,
        const double &delta_tau,
        const double &weight);

  protected:
  };
}

#endif