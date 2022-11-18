#ifndef MRP_RVO__ORCA_HPP_
#define MRP_RVO__ORCA_HPP_

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

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
}

#endif