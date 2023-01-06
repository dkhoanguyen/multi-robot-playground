#include "mrp_pure_pursuit/solver.hpp"
#include <iostream>

namespace mrp_pure_pursuit
{
  namespace solver
  {
    Variables::Variables()
        : ifopt::VariableSet(2, VARSET_NAME),
          vel_x_(0),
          vel_y_(0)
    {
    }

    Variables::~Variables()
    {
    }

    void Variables::SetVariables(const Eigen::VectorXd &x)
    {
      vel_x_ = x(0);
      vel_y_ = x(1);
    }

    Eigen::VectorXd Variables::GetValues() const
    {
      return Eigen::Vector2d(vel_x_, vel_y_);
    }

    ifopt::Component::VecBound Variables::GetBounds() const
    {
      VecBound bounds(ifopt::Component::GetRows());
      bounds.at(0) = ifopt::Bounds(vel_lower_bound_(0), vel_upper_bound_(0));
      bounds.at(1) = ifopt::Bounds(vel_lower_bound_(1), vel_upper_bound_(1));
      return bounds;
    }

    void Variables::SetBounds(const Eigen::Vector2d &vel_lower_bound,
                              const Eigen::Vector2d &vel_upper_bound)
    {
      vel_lower_bound_ = vel_lower_bound;
      vel_upper_bound_ = vel_upper_bound;
    }


    // Constraints //
    Constraint::Constraint(int num)
        : ifopt::ConstraintSet(num, CONSTRAINTSET_NAME),
          num_constraints_(num)
    {
    }
    Constraint::~Constraint()
    {
    }

    Eigen::VectorXd Constraint::GetValues() const
    {
      Eigen::VectorXd constraints(ifopt::Component::GetRows());
      Eigen::Vector2d x = GetVariables()->GetComponent(VARSET_NAME)->GetValues();
      for (unsigned int i = 0; i < orca_halfplanes_.size(); i++)
      {
        constraints(i) = orca_halfplanes_.at(i).line().a() * x(0) + orca_halfplanes_.at(i).line().b() * x(1);
      }
      return constraints;
    }

    ifopt::Component::VecBound Constraint::GetBounds() const
    {
      ifopt::Component::VecBound bounds(ifopt::Component::GetRows());
      for (unsigned int i = 0; i < orca_halfplanes_.size(); i++)
      {
        bounds.at(i) = ifopt::Bounds(orca_halfplanes_.at(i).line().c(), ifopt::inf);
      }
      return bounds;
    }

    void Constraint::FillJacobianBlock(std::string var_set, Jacobian &jac_block) const
    {
      if (var_set == VARSET_NAME)
      {
        for (unsigned int i = 0; i < orca_halfplanes_.size(); i++)
        {
          jac_block.coeffRef(i, 0) = orca_halfplanes_.at(i).line().a(); // derivative of the ith constraint w.r.t x0
          jac_block.coeffRef(i, 1) = orca_halfplanes_.at(i).line().b(); // derivative of the ith constraint w.r.t x1
        }
      }
    }

    void Constraint::AddConstraint(const geometry::HalfPlane &orca_halfplane)
    {
      orca_halfplanes_.push_back(orca_halfplane);
    }

    void Constraint::AddConstraints(const std::vector<geometry::HalfPlane> &orca_halfplanes)
    {
      orca_halfplanes_ = orca_halfplanes;
    }

    Cost::Cost() : CostTerm(COST_NAME) {}
    Cost::~Cost() {}

    double Cost::GetCost() const
    {
      Eigen::Vector2d x = GetVariables()->GetComponent(VARSET_NAME)->GetValues();
      return std::pow(x(0) - optimal_vel_(0), 2) + std::pow(x(1) - optimal_vel_(1), 2);
    }

    void Cost::FillJacobianBlock(std::string var_set, Jacobian &jac) const
    {
      if (var_set == VARSET_NAME)
      {
        Eigen::Vector2d x = GetVariables()->GetComponent(VARSET_NAME)->GetValues();

        jac.coeffRef(0, 0) = 2.0 * (x(0) - optimal_vel_(0)); // derivative of cost w.r.t x0
        jac.coeffRef(0, 1) = 2.0 * (x(1) - optimal_vel_(1)); // derivative of cost w.r.t x1
      }
    }

    void Cost::SetOptimalVelocity(const Eigen::Vector2d &optimal_vel)
    {
      optimal_vel_ = optimal_vel;
    }


    // SOLVER //
    Solver::Solver()
    {
    }

    Solver::~Solver()
    {
    }

    Eigen::Vector2d Solver::solve(
        std::shared_ptr<mrp_pure_pursuit::solver::Variables> variables,
        std::shared_ptr<mrp_pure_pursuit::solver::Constraint> constraints,
        std::shared_ptr<mrp_pure_pursuit::solver::Cost> cost)
    {
      ifopt::Problem nlp;
      nlp.AddVariableSet(variables);
      nlp.AddConstraintSet(constraints);
      nlp.AddCostSet(cost);

      // 2. choose solver and options
      ifopt::IpoptSolver ipopt;
      ipopt.SetOption("print_level", 0);
      ipopt.SetOption("linear_solver", "mumps");
      ipopt.SetOption("jacobian_approximation", "exact");

      int status = ipopt.Solve(nlp);
      if (status == 0)
      {
        return nlp.GetOptVariables()->GetValues();
      }
      return Eigen::Vector2d(0, 0);
    }
  }
}