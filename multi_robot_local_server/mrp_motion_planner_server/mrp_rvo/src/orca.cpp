#include "mrp_rvo/orca.hpp"
#include <iostream>

namespace mrp_orca
{
  ORCAVariables::ORCAVariables() : ifopt::VariableSet(2, "var_set1")
  {
    x_ = 0;
    y_ = 0;
  }

  ORCAVariables::~ORCAVariables()
  {
  }

  void ORCAVariables::SetVariables(const Eigen::VectorXd &x)
  {
    x_ = x(0);
    y_ = x(1);
  }

  Eigen::VectorXd ORCAVariables::GetValues() const
  {
    return Eigen::Vector2d(x_, y_);
  }

  ifopt::Component::VecBound ORCAVariables::GetBounds() const
  {
    VecBound bounds(ifopt::Component::GetRows());
    bounds.at(0) = ifopt::Bounds(-10.0, 10.0);
    bounds.at(1) = ifopt::Bounds(-10.0, 10.0);
    return bounds;
  }

  ORCAConstraint::ORCAConstraint() : ifopt::ConstraintSet(6, "constraints") {}

  ORCAConstraint::~ORCAConstraint() {}

  Eigen::VectorXd ORCAConstraint::GetValues() const
  {
    Eigen::VectorXd g(ifopt::Component::GetRows());
    Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    g(0) = 3 * x(0) + 2 * x(1);
    g(1) = -5 * x(0) + 3 * x(1);
    g(2) = -2 * x(0) - 11 * x(1);
    g(3) = -2 * x(0) + 1 * x(1);
    g(4) = x(0) - 2 * x(1);
    g(5) = -4 * x(0) + x(1);
    return g;
  }

  ifopt::Component::VecBound ORCAConstraint::GetBounds() const
  {
    ifopt::Component::VecBound b(ifopt::Component::GetRows());
    b.at(0) = ifopt::Bounds(-ifopt::inf, 20.0);
    b.at(1) = ifopt::Bounds(-ifopt::inf, 17.0);
    b.at(2) = ifopt::Bounds(-ifopt::inf, -51.0);
    b.at(3) = ifopt::Bounds(-ifopt::inf, 0.0);
    b.at(4) = ifopt::Bounds(-ifopt::inf, 0.0);
    b.at(5) = ifopt::Bounds(-ifopt::inf, 0.0);
    return b;
  }

  void ORCAConstraint::FillJacobianBlock(std::string var_set, Jacobian &jac_block) const
  {
    if (var_set == "var_set1")
    {
      Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac_block.coeffRef(0, 0) = 3.0; // derivative of first constraint w.r.t x0
      jac_block.coeffRef(0, 1) = 2.0; // derivative of first constraint w.r.t x1

      jac_block.coeffRef(1, 0) = -5.0; // derivative of first constraint w.r.t x0
      jac_block.coeffRef(1, 1) = 3.0;  // derivative of first constraint w.r.t x1

      jac_block.coeffRef(2, 0) = -2.0;  // derivative of first constraint w.r.t x0
      jac_block.coeffRef(2, 1) = -11.0; // derivative of first constraint w.r.t x1

      jac_block.coeffRef(3, 0) = -2.0; // derivative of first constraint w.r.t x0
      jac_block.coeffRef(3, 1) = 1.0;  // derivative of first constraint w.r.t x1

      jac_block.coeffRef(4, 0) = 1.0;  // derivative of first constraint w.r.t x0
      jac_block.coeffRef(4, 1) = -2.0; // derivative of first constraint w.r.t x1

      jac_block.coeffRef(5, 0) = -4.0; // derivative of first constraint w.r.t x0
      jac_block.coeffRef(5, 1) = 1.0;  // derivative of first constraint w.r.t x1
    }
  }

  ORCACost::ORCACost() : CostTerm("cost") {}
  ORCACost::~ORCACost() {}

  double ORCACost::GetCost() const
  {
    Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    return -std::pow(x(0), 2) - std::pow(x(1), 2);
  }

  void ORCACost::FillJacobianBlock(std::string var_set, Jacobian &jac) const
  {
    if (var_set == "var_set1")
    {
      Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = -2.0 * x(0); // derivative of cost w.r.t x0
      jac.coeffRef(0, 1) = -2.0 * x(1); // derivative of cost w.r.t x1
    }
  }

  ORCA::ORCA()
  {
  }

  ORCA::~ORCA()
  {
  }

  common::HalfPlane ORCA::construct(
      const nav_msgs::msg::Odometry &odom_A,
      const nav_msgs::msg::Odometry &odom_B,
      const double &radius_A, const double &radius_B,
      const double &delta_tau,
      const double &weight)
  {
    // Current position of each robot
    // x,y,theta
    Eigen::Vector2d A_2d_pose{
        odom_A.pose.pose.position.x,
        odom_A.pose.pose.position.y};
    double theta_A = mrp_common::GeometryUtils::yawFromPose(odom_A.pose.pose);

    Eigen::Vector2d B_2d_pose{
        odom_B.pose.pose.position.x,
        odom_B.pose.pose.position.y};
    double theta_B = mrp_common::GeometryUtils::yawFromPose(odom_B.pose.pose);

    // Get Linear velocity of each robot in the x direction for now
    // and convert that into a vector with x y component
    Eigen::Vector2d A_vel_vector = mrp_common::GeometryUtils::projectToXY(
        odom_A.twist.twist.linear.x, theta_A);

    Eigen::Vector2d B_vel_vector = mrp_common::GeometryUtils::projectToXY(
        odom_B.twist.twist.linear.x, theta_B);

    // Start onstructing the truncated collision cone
    // Distance from A to B
    double distance_A_to_B = mrp_common::GeometryUtils::euclideanDistance(
        odom_A.pose.pose, odom_B.pose.pose);
    // Angle between AB
    double angle_A_to_B = atan2(B_2d_pose(1) - A_2d_pose(1), B_2d_pose(0) - A_2d_pose(0));

    // Sum of radius
    double sum_r = radius_A + radius_B;
    double inner_angle = asin(sum_r / distance_A_to_B);
    double side_edge = sqrt(std::pow(distance_A_to_B, 2) - std::pow(sum_r, 2)) + sum_r;

    // Relative position between AB
    Eigen::Vector2d relative_pos = B_2d_pose - A_2d_pose;

    // Truncated cone bottom
    Eigen::Vector2d bottom_circle = (1 / delta_tau) * relative_pos;
    double bottom_radius = sum_r / delta_tau;

    double upper_angle = angle_A_to_B + inner_angle;
    double lower_angle = angle_A_to_B - inner_angle;

    // Relative velocity
    Eigen::Vector2d relative_v = A_vel_vector - B_vel_vector;
    Eigen::Vector2d u{0,0};

    // Check if relative v can be projected onto the circle
    if (bottom_circle.dot(relative_v - bottom_circle) < 0)
    {
      Eigen::Vector2d relative_to_center = relative_v - bottom_circle;
      u = bottom_circle + (relative_to_center/relative_to_center.norm()) * bottom_radius;
      u = u - relative_v;
    }
    // Relative v lies behind the circle center -> must be projected onto the 2 sides
    else
    {
      double angle_to_relative_v = atan2(relative_v(1),relative_v(0));
      if(std::abs(upper_angle - angle_to_relative_v) < std::abs(angle_to_relative_v - lower_angle))
      {
        double side_length = relative_v.norm() * cos(upper_angle - angle_to_relative_v);
        Eigen::Vector2d projected_v{
          side_length * cos(upper_angle),
          side_length * sin(upper_angle)
        };
        u = projected_v - relative_v;
      }
      else
      {
        double side_length = relative_v.norm() * cos(angle_to_relative_v - lower_angle);
        Eigen::Vector2d projected_v{
          side_length * cos(lower_angle),
          side_length * sin(lower_angle)
        };
        u = projected_v - relative_v;
      }
    }

    Eigen::Vector2d weighted_u = weight * u;
    Eigen::Vector2d orca_point = A_vel_vector + weighted_u;

    // Construct ORCA halfplane
    common::Line orca_line(weighted_u,orca_point);
    common::HalfPlane orca(orca_line,weighted_u);
    return orca;
  }
} // namespace mrp_orca
