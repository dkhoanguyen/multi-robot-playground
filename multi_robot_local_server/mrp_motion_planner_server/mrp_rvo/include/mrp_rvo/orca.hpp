#ifndef MRP_RVO__ORCA_HPP_
#define MRP_RVO__ORCA_HPP_

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include "nav_msgs/msg/odometry.hpp"

#include "mrp_common/utils.hpp"

#include "mrp_rvo/geometry.hpp"

namespace mrp_orca
{
  class ORCAVariables : public ifopt::VariableSet
  {
  public:
    ORCAVariables();
    ~ORCAVariables();

    void SetVariables(const Eigen::VectorXd &x) override;
    Eigen::VectorXd GetValues() const override;

    VecBound GetBounds() const override;

  protected:
    double x_, y_;
  };

  class ORCAConstraint : public ifopt::ConstraintSet
  {
  public:
    ORCAConstraint();
    ~ORCAConstraint();

    VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override;
  };

  class ORCACost : public ifopt::CostTerm
  {
  public:
    ORCACost();
    ~ORCACost();

    double GetCost() const override;
    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override;
  };

  class ORCA
  {
  public:
    ORCA();
    ~ORCA();

    // Construct ORCA half plane of B induced by A, given
    // A current position and velocity vector
    // B current position and velocity vector
    common::HalfPlane construct(
        const nav_msgs::msg::Odometry &odom_A,
        const nav_msgs::msg::Odometry &odom_B,
        const double &radius_A, const double &radius_B,
        const double &delta_tau,
        const double &weight);

  protected:
  };
}

#endif