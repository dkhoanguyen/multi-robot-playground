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
    x_ = x(1);
    y_ = x(1);
  }

  Eigen::VectorXd ORCAVariables::GetValues() const
  {
    return Eigen::Vector2d(x_, y_);
  }

  ifopt::Component::VecBound ORCAVariables::GetBounds() const
  {
    VecBound bounds(GetRows());
    bounds.at(0) = ifopt::Bounds(-1.0, 1.0);
    bounds.at(1) = ifopt::NoBound;
    return bounds;
  }

  ORCAConstraint::ORCAConstraint() : ifopt::ConstraintSet()

} // namespace mrp_orca
