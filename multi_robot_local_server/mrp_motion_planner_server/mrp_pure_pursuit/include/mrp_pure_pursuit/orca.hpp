#ifndef MRP_ORCA__ORCA_HPP_
#define MRP_ORCA__ORCA_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "mrp_common/utils.hpp"
#include "mrp_pure_pursuit/geometry.hpp"
#include "tf2/utils.h"

namespace mrp_pure_pursuit
{
  class ORCA
  {
  public:
    enum class Result
    {
      COLLISION,
      ON_COLLISION_COURSE,
      NO_IMMEDIATE_COLLISION,
      NO_COLLISION
    };

    static Result localConstruct(
        mrp_pure_pursuit::geometry::HalfPlane &output_orca,
        const Eigen::Vector2d &desired_vel_A,
        const nav_msgs::msg::Odometry &odom_B,
        const double &radius_A,
        const double &radius_B,
        const double &delta_tau,
        const double &weight);

  protected:
  };

  struct RVO
  {
  public:
    Eigen::Vector2d rvo;
    double upper_angle;
    double lower_angle;
    double distance_to_other;
    double angle_to_other;

    enum class Result
    {
      COLLISION,
      NO_COLLISION
    };

    static Result construct(
        mrp_pure_pursuit::RVO &output_rvo,
        const nav_msgs::msg::Odometry &robot_odom,
        const nav_msgs::msg::Odometry &other_odom,
        const double &radius_A,
        const double &radius_B,
        const double &weight);

    static bool checkCollision(
        const nav_msgs::msg::Odometry &robot_odom,
        const RVO &target_rvo);

    static Eigen::Vector2d pickNewVelocity(
        const geometry_msgs::msg::Pose &robot_pose,
        const Eigen::Vector2d &desired_vel,
        const std::vector<RVO> &rvo_list);
  };

}

#endif