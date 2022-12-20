#ifndef MRP_NMPC_ORCA__SOLVER_HPP_
#define MRP_NMPC_ORCA__SOLVER_HPP_

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>

namespace mrp_nmpc_orca
{
  namespace nmpc
  {
    // Implementation of single shooting
    static const int DEFAULT_MPC_STEPS = 10;
    static const int DEFAULT_DELTA_T = 0.1;

    static const std::string VARSET_NAME = "opt_vars";
    static const std::string CONSTRAINTSET_NAME = "nmpc_contrains";
    static const std::string COST_NAME = "nmpc_cost_func";

    class OptimisationVariables : public ifopt::VariableSet
    {
    public:
      OptimisationVariables(int mpc_steps);
      ~OptimisationVariables();

      void SetVariables(const Eigen::VectorXd &x) override;
      Eigen::VectorXd GetValues() const override;
      VecBound GetBounds() const override;

      void SetBounds(const Eigen::Vector2d &control_lower_bound,
                     const Eigen::Vector2d &control_upper_bound);

    protected:
      int mpc_steps_;

      Eigen::VectorXd control_u_;
      Eigen::VectorXd pose_x_;
      Eigen::VectorXd u_lower_bound_, u_upper_bound_;
    };

    class Constraints : public ifopt::ConstraintSet
    {
      Constraints(int mpc_steps, double dt);
      ~Constraints();

      VectorXd GetValues() const override;
      VecBound GetBounds() const override;
      void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override;
      void SetInitialState(const Eigen::Vector2d &initial_x);

    protected:
      int mpc_steps_;
      double dt_;
      Eigen::Vector2d initial_x_;
      Eigen::VectorXd X_;
    };

    class Cost : public ifopt::CostTerm
    {
    public:
      Cost(int mpc_steps);
      ~Cost();

      double GetCost() const override;
      void FillJacobianBlock(std::string var_set, Jacobian &jac) const override;

      // void SetTargetState(const Eigen::Vector2d &target_x);

    protected:
      Eigen::Vector3d target_x_;
      int mpc_steps_;
    };
  }
}

#endif