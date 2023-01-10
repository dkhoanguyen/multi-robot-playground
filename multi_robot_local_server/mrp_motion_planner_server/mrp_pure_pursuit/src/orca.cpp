#include "mrp_pure_pursuit/orca.hpp"

namespace mrp_pure_pursuit
{
  ORCA::Result ORCA::localConstruct(
      const nav_msgs::msg::Odometry &odom_A,
      const nav_msgs::msg::Odometry &odom_B,
      const double &radius_A,
      const double &radius_B,
      const double &delta_tau,
      const double &weight,
      mrp_pure_pursuit::geometry::HalfPlane &output_orca)
  {
    // Get vel vector of A
    Eigen::Vector2d A_vel_vector = mrp_common::GeometryUtils::projectToXY(
        odom_A.twist.twist.linear.x, 0);

    // Position vector for robot B in robot A frame
    Eigen::Vector2d B_2d_pose{
        odom_B.pose.pose.position.x,
        odom_B.pose.pose.position.y};
    double theta_B = tf2::getYaw(odom_B.pose.pose.orientation);

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
    Eigen::Vector2d relative_v = A_vel_vector - B_vel_vector;

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
      if ((relative_v - bottom_circle).norm() > bottom_radius &&
          relative_v.norm() < bottom_circle.norm())
      {
        return ORCA::Result::NO_IMMEDIATE_COLLISION;
      }
    }

    double angle_to_relative_v = atan2(relative_v(1), relative_v(0));

    // If we are too close, immediately get a velocity to avoid collision
    if (relative_pos.norm() <= sum_r)
    {
      std::cout << "SOS we are too close" << std::endl;
      return ORCA::Result::COLLISION;
    }

    if (std::abs(upper_angle - angle_to_relative_v) < std::abs(angle_to_relative_v - lower_angle))
    {
      double side_length = relative_v.norm() * cos(upper_angle - angle_to_relative_v);
      Eigen::Vector2d projected_v{
          side_length * cos(upper_angle),
          side_length * sin(upper_angle)};
      u = projected_v - relative_v;
    }
    else
    {
      double side_length = relative_v.norm() * cos(angle_to_relative_v - lower_angle);
      Eigen::Vector2d projected_v{
          side_length * cos(lower_angle),
          side_length * sin(lower_angle)};
      u = projected_v - relative_v;
    }

    Eigen::Vector2d weighted_u = weight * u;
    Eigen::Vector2d orca_point = A_vel_vector + weighted_u;

    // Construct ORCA halfplane
    geometry::Line orca_line(weighted_u, orca_point);
    geometry::HalfPlane orca(orca_line, weighted_u);
    output_orca = orca;
    return ORCA::Result::ON_COLLISION_COURSE;
  }

  RVO::Result RVO::construct(
      mrp_pure_pursuit::RVO &output_rvo,
      const nav_msgs::msg::Odometry &robot_odom,
      const nav_msgs::msg::Odometry &other_odom,
      const double &radius_A,
      const double &radius_B,
      const double &weight)
  {
    Eigen::Vector2d robot_2d_pose(
        robot_odom.pose.pose.position.x,
        robot_odom.pose.pose.position.y);
    double robot_theta = tf2::getYaw(robot_odom.pose.pose.orientation);

    Eigen::Vector2d other_2d_pose{
        other_odom.pose.pose.position.x,
        other_odom.pose.pose.position.y};
    double other_theta = tf2::getYaw(other_odom.pose.pose.orientation);

    Eigen::Vector2d robot_vel_vector = mrp_common::GeometryUtils::projectToXY(
        robot_odom.twist.twist.linear.x, robot_theta);

    // Velocity vector for robot B
    Eigen::Vector2d other_vel_vector = mrp_common::GeometryUtils::projectToXY(
        other_odom.twist.twist.linear.x, other_theta);

    double distance_to_other = (other_2d_pose - robot_2d_pose).norm();
    double angle_to_other = atan2(other_2d_pose(1) - robot_2d_pose(1),
                                  other_2d_pose(0) - robot_2d_pose(0));

    Eigen::Vector2d admissive_vel_vector = weight * (robot_2d_pose + other_2d_pose);

    Eigen::Vector2d rvo = robot_2d_pose + admissive_vel_vector;

    // Sum of radius
    double sum_r = radius_A + radius_B;

    if (distance_to_other < sum_r)
    {
      distance_to_other = sum_r;
    }

    double omega = asin(sum_r / distance_to_other);
    double upper_angle = angle_to_other + omega;
    double lower_angle = angle_to_other - omega;

    while (upper_angle > M_PI)
    {
      upper_angle = upper_angle - 2 * M_PI;
    }

    while (upper_angle < -M_PI)
    {
      upper_angle = upper_angle + 2 * M_PI;
    }

    while (lower_angle > M_PI)
    {
      lower_angle = lower_angle - 2 * M_PI;
    }

    while (lower_angle < -M_PI)
    {
      lower_angle = lower_angle + 2 * M_PI;
    }

    output_rvo.rvo = rvo;
    output_rvo.upper_angle = upper_angle;
    output_rvo.lower_angle = lower_angle;
    output_rvo.distance_to_other = distance_to_other;
    output_rvo.angle_to_other = angle_to_other;
  }

  bool RVO::checkCollision(
      const nav_msgs::msg::Odometry &robot_odom,
      const RVO &target_rvo)
  {
    Eigen::Vector2d robot_2d_pose(
        robot_odom.pose.pose.position.x,
        robot_odom.pose.pose.position.y);
    double robot_theta = tf2::getYaw(robot_odom.pose.pose.orientation);

    Eigen::Vector2d vel_vector = mrp_common::GeometryUtils::projectToXY(
        robot_odom.twist.twist.linear.x, robot_theta);

    Eigen::Vector2d local_rvo = target_rvo.rvo - robot_2d_pose;
    Eigen::Vector2d rel_vel_rvo = vel_vector - local_rvo;

    double rel_vel_rvo_theta = atan2(rel_vel_rvo(1), rel_vel_rvo(0));

    Eigen::Vector2d upper_vec(cos(target_rvo.upper_angle), sin(target_rvo.upper_angle));
    Eigen::Vector2d lower_vec(cos(target_rvo.lower_angle), sin(target_rvo.lower_angle));

    return mrp_common::GeometryUtils::vectorIsInside(rel_vel_rvo, lower_vec, upper_vec);
  }

  Eigen::Vector2d RVO::pickNewVelocity(
      const geometry_msgs::msg::Pose &robot_pose,
      const Eigen::Vector2d &desired_vel,
      const std::vector<RVO> &rvo_list)
  {
    Eigen::Vector2d robot_2d_pose(
        robot_pose.position.x,
        robot_pose.position.y);
    double theta = tf2::getYaw(robot_pose.orientation);
    double linear_vel = desired_vel.norm();

    double angular_res = 0.001;
    int size = (2 * M_PI) / angular_res;
    Eigen::VectorXd sampled_angle = Eigen::VectorXd::LinSpaced(
        size, -M_PI + theta, M_PI + theta);

    std::vector<double> cost_vector;
    for (int idx = 0; idx < sampled_angle.size(); idx++)
    {
      Eigen::Vector2d sampled_vel(
          linear_vel * cos(sampled_angle(idx)),
          linear_vel * sin(sampled_angle(idx)));

      for (RVO rvo : rvo_list)
      {
        Eigen::Vector2d local_rvo = rvo.rvo - robot_2d_pose;
        Eigen::Vector2d rel_sample_vel_rvo = sampled_vel - local_rvo;

        double rel_sample_vel_rvo_theta = atan2(rel_sample_vel_rvo(1), rel_sample_vel_rvo(0));

        Eigen::Vector2d upper_vec(cos(rvo.upper_angle), sin(rvo.upper_angle));
        Eigen::Vector2d lower_vec(cos(rvo.lower_angle), sin(rvo.lower_angle));

        if (mrp_common::GeometryUtils::vectorIsInside(
                rel_sample_vel_rvo, lower_vec, upper_vec))
        {
          // If this sampled is in one of the rvo then skip it
          break;
        }
      }

      double sampled_cost = (sampled_vel - desired_vel).norm();
      cost_vector.push_back(sampled_cost);
    }

    auto min_cost_it = std::min_element(std::begin(cost_vector), std::end(cost_vector));
    int min_index = std::distance(std::begin(cost_vector), min_cost_it);

    double min_angle = sampled_angle(min_index);
    return Eigen::Vector2d(
        linear_vel * cos(min_angle),
        linear_vel * sin(min_angle));
  }
}