#ifndef MRP_ORCA__SOLVER_HPP_
#define MRP_ORCA__SOLVER_HPP_

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>

#include "mrp_orca/geometry.hpp"

namespace mrp_orca
{
  namespace solver
  {
    static const std::string VARSET_NAME = "opt_vel";
    static const std::string CONSTRAINTSET_NAME = "orca_constraints";
    static const std::string COST_NAME = "orca_cost";

    class Variables : public ifopt::VariableSet
    {
    public:
      Variables();
      ~Variables();

      void SetVariables(const Eigen::VectorXd &x) override;
      Eigen::VectorXd GetValues() const override;
      VecBound GetBounds() const override;

      void SetBounds(const Eigen::Vector2d &vel_lower_bound,
                     const Eigen::Vector2d &vel_upper_bound);

    protected:
      double vel_x_, vel_y_;
      Eigen::Vector2d vel_upper_bound_, vel_lower_bound_;
    };

    class Constraint : public ifopt::ConstraintSet
    {
    public:
      Constraint(int num);
      ~Constraint();

      VectorXd GetValues() const override;
      VecBound GetBounds() const override;
      void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override;

      void AddConstraint(const geometry::HalfPlane &orca_halfplane);
      void AddConstraints(const std::vector<geometry::HalfPlane> &orca_halfplanes);

    protected:
      int num_constraints_;
      std::vector<geometry::HalfPlane> orca_halfplanes_;
    };

    class Cost : public ifopt::CostTerm
    {
    public:
      Cost();
      ~Cost();

      double GetCost() const override;
      void FillJacobianBlock(std::string var_set, Jacobian &jac) const override;

      void SetOptimalVelocity(const Eigen::Vector2d &optimal_vel);

    protected:
      Eigen::Vector2d optimal_vel_;
    };

    class Solver
    {
    public:
      Solver();
      ~Solver();

      static Eigen::Vector2d solve(
          std::shared_ptr<mrp_orca::solver::Variables> variables,
          std::shared_ptr<mrp_orca::solver::Constraint> constraints,
          std::shared_ptr<mrp_orca::solver::Cost> cost);
    };
  }
}

#endif