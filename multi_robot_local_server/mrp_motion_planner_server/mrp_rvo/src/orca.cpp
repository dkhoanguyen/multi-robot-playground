#include "mrp_rvo/orca.hpp"

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

  mrp_orca::common::HalfPlane ORCA::construct(
      const nav_msgs::msg::Odometry &odom_A,
      const nav_msgs::msg::Odometry &odom_B,
      const double &radius_A, const double &radius_B,
      const double &delta_tau)
  {
    // Current position of each robot
    // x,y,theta
    Eigen::Vector3d A_2d_pose{
        odom_A.pose.pose.position.x,
        odom_A.pose.pose.position.y,
        mrp_common::GeometryUtils::yawFromPose(odom_A.pose.pose),
    };

    Eigen::Vector3d B_2d_pose{
        odom_B.pose.pose.position.x,
        odom_B.pose.pose.position.y,
        mrp_common::GeometryUtils::yawFromPose(odom_B.pose.pose),
    };

    // Get Linear velocity of each robot in the x direction for now
    // and convert that into a vector with x y component
    Eigen::Vector2d A_vel_vector = mrp_common::GeometryUtils::projectToXY(
        odom_A.twist.twist.linear.x, A_2d_pose(2));

    Eigen::Vector2d B_vel_vector = mrp_common::GeometryUtils::projectToXY(
        odom_B.twist.twist.linear.x, B_2d_pose(2));

    // Start constructing the ORCA half plane
    // First find the relative velocity
    Eigen::Vector2d rel_vel = A_vel_vector - B_vel_vector;

    // Then construct the truncated collision cone
    // Relative angle from A to B
    double relative_heading = atan2(rel_vel(1),rel_vel(0));

    // Distance from A to B
    double distance_A_to_B = mrp_common::GeometryUtils::euclideanDistance(
        odom_A.pose.pose, odom_B.pose.pose);
  }
} // namespace mrp_orca
