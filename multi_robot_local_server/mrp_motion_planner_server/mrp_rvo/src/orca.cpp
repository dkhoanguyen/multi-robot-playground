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

  ORCAConstraint::ORCAConstraint() : ifopt::ConstraintSet(3, "constraints") {}

  ORCAConstraint::~ORCAConstraint() {}

  Eigen::VectorXd ORCAConstraint::GetValues() const
  {
    Eigen::VectorXd g(ifopt::Component::GetRows());
    Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    g(0) = 3 * x(0) + 2 * x(1);
    g(1) = -5 * x(0) + 3 * x(1);
    g(2) = -2 * x(0) - 11 * x(1);
    return g;
  }

  ifopt::Component::VecBound ORCAConstraint::GetBounds() const
  {
    ifopt::Component::VecBound b(ifopt::Component::GetRows());
    b.at(0) = ifopt::Bounds(-ifopt::inf, 20.0);
    b.at(1) = ifopt::Bounds(-ifopt::inf, 17.0);
    b.at(2) = ifopt::Bounds(-ifopt::inf, -51.0);
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

      jac.coeffRef(0, 0) = -2.0 * x(0);   // derivative of cost w.r.t x0
      jac.coeffRef(0, 1) = -2.0 * x(1); // derivative of cost w.r.t x1
    }
  }
} // namespace mrp_orca
