#include "mrp_pure_pursuit/orca.hpp"

namespace mrp_pure_pursuit
{
  ORCA::Result ORCA::localConstruct(
      mrp_pure_pursuit::geometry::HalfPlane &output_orca,
      const Eigen::Vector2d &desired_vel_A,
      const nav_msgs::msg::Odometry &odom_B,
      const double &radius_A,
      const double &radius_B,
      const double &delta_tau,
      const double &weight)
  {
    // Position vector for robot B in robot A frame
    Eigen::Vector2d B_2d_pose{
        odom_B.pose.pose.position.x,
        odom_B.pose.pose.position.y};
    double theta_B = mrp_common::GeometryUtils::yawFromPose(odom_B.pose.pose);

    // Velocity vector for robot B
    Eigen::Vector2d B_vel_vector = mrp_common::GeometryUtils::projectToXY(
        odom_B.twist.twist.linear.x, theta_B);

    double distance_A_to_B = B_2d_pose.norm();
    double angle_A_to_B = atan2(B_2d_pose(1), B_2d_pose(0));

    // Sum of radius
    double sum_r = radius_A + radius_B;
    double inner_angle = asin(sum_r / distance_A_to_B);
    double side_edge = sqrt(std::pow(distance_A_to_B, 2) - std::pow(sum_r, 2)) + sum_r;

    // Relative position between AB
    Eigen::Vector2d relative_pos = B_2d_pose;

    // Truncated cone bottom
    Eigen::Vector2d bottom_circle = (1 / delta_tau) * relative_pos;
    double bottom_radius = sum_r / delta_tau;

    double upper_angle = angle_A_to_B + inner_angle;
    double lower_angle = angle_A_to_B - inner_angle;

    // Relative velocity
    Eigen::Vector2d relative_v = desired_vel_A - B_vel_vector;

    // std::cout << "relative_v: " << relative_v.transpose() << std::endl;

    // Ok so before we carry on let's check if there is a collision
    // If relative vector is not bounded by the truncated VO
    if (!mrp_common::GeometryUtils::vectorIsInside(
            relative_v,
            mrp_common::GeometryUtils::projectToXY(side_edge, lower_angle),
            mrp_common::GeometryUtils::projectToXY(side_edge, upper_angle)))
    {
      return ORCA::Result::NO_COLLISION;
    }

    Eigen::Vector2d u(0, 0);
    // Check if relative v can be projected onto the circle
    if (bottom_circle.dot(relative_v - bottom_circle) < 0)
    {
      // Relative v lies in front of the circle
      // Check if relative v is inside the circle
      if ((relative_v - bottom_circle).norm() > bottom_radius)
      {
        return ORCA::Result::NO_IMMEDIATE_COLLISION;
      }
    }

    double angle_to_relative_v = atan2(relative_v(1), relative_v(0));
    // std::cout << "Upper angle: " << upper_angle << std::endl;
    // std::cout << "Lower angle: " << lower_angle << std::endl;
    // std::cout << "Angle to relative v: " << angle_to_relative_v << std::endl;

    // If we are too close, immediately get a velocity to avoid collision
    std::cout << "Distance: " << relative_pos.norm() << std::endl;
    if (relative_pos.norm() <= sum_r)
    {
      std::cout << "SOS we are too close" << std::endl;
    }

    if (std::abs(upper_angle - angle_to_relative_v) < std::abs(angle_to_relative_v - lower_angle))
    {
      double side_length = relative_v.norm() * cos(upper_angle - angle_to_relative_v);
      Eigen::Vector2d projected_v{
          side_length * cos(upper_angle),
          side_length * sin(upper_angle)};
      u = projected_v - relative_v;
      // std::cout << "Upper" << std::endl;
    }
    else
    {
      double side_length = relative_v.norm() * cos(angle_to_relative_v - lower_angle);
      Eigen::Vector2d projected_v{
          side_length * cos(lower_angle),
          side_length * sin(lower_angle)};
      u = projected_v - relative_v;
      // std::cout << "Lower" << std::endl;
    }

    Eigen::Vector2d weighted_u = weight * u;
    Eigen::Vector2d orca_point = desired_vel_A + weighted_u;

    // std::cout << "ORCA point: " << orca_point.transpose() << std::endl;
    // std::cout << "U: " << weighted_u.transpose() << std::endl;

    // Construct ORCA halfplane
    geometry::Line orca_line(weighted_u, orca_point);
    geometry::HalfPlane orca(orca_line, weighted_u);
    output_orca = orca;
    return ORCA::Result::COLLISION;
  }

}