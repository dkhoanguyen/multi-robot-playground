#include "mrp_nmpc_orca/solver.hpp"

namespace mrp_nmpc_orca
{
  namespace nmpc
  {
    OptimisationVariables::OptimisationVariables(int mpc_steps)
        : ifopt::VariableSet(mpc_steps, VARSET_NAME),
          mpc_steps_(mpc_steps)
    {
      // Initialise control u to zero
      for (int i = 0; i < mpc_steps; i++)
      {
        control_u_(2 * i) = 0;
        control_u_(2 * i + 1) = 0;
      }
    }

    OptimisationVariables::~OptimisationVariables()
    {
    }

    void OptimisationVariables::SetVariables(const Eigen::VectorXd &u)
    {
      control_u_ = u;
    }

    Eigen::VectorXd OptimisationVariables::GetValues() const
    {
      return control_u_;
    }

    ifopt::Component::VecBound OptimisationVariables::GetBounds() const
    {
      VecBound bounds(ifopt::Component::GetRows());
      for (int i = 0; i < mpc_steps_; i++)
      {
        bounds.at(2 * i) = ifopt::Bounds(u_lower_bound_(0), u_upper_bound_(0));
        bounds.at(2 * i + 1) = ifopt::Bounds(u_lower_bound_(1), u_upper_bound_(1));
      }

      return bounds;
    }

    void OptimisationVariables::SetBounds(
        const Eigen::Vector2d &u_lower_bound,
        const Eigen::Vector2d &u_upper_bound)
    {
      u_lower_bound_ = u_lower_bound;
      u_upper_bound_ = u_upper_bound;
    }

    // At each time step, we calculate the following constraints
    // Next state given the current state and predicted control
    // And an initial constraint for the first state (x and y)
    // Therefore, num constraints should be 2 * mpc_steps + 1
    Constraints::Constraints(int mpc_steps, double dt)
        : ifopt::ConstraintSet(2 * mpc_steps + 1, CONSTRAINTSET_NAME),
          mpc_steps_(mpc_steps),
          dt_(dt)
    {
    }
    Constraints::~Constraints()
    {
    }

    Eigen::VectorXd Constraints::GetValues() const
    {
      Eigen::VectorXd g(ifopt::Component::GetRows());
      Eigen::VectorXd X(mpc_steps_ * 3);
      Eigen::VectorXd u = GetVariables()->GetComponent(VARSET_NAME)->GetValues();

      // Compute state X first
      X(0) = initial_x_(0);
      X(1) = initial_x_(1);
      X(2) = initial_x_(2);
      for (int k = 1; k < 3 * mpc_steps_; k++)
      {
        X(3 * k) = X(3 * (k - 1)) + u(2 * (k - 1)) * cos(X(3 * (k - 1) + 2)) * dt_;         // x(k) = x(k-1) + v(k-1)*cos(theta(k-1)) * dt
        X(3 * k + 1) = X(3 * (k - 1) + 1) + u(2 * (k - 1)) * sin(X(3 * (k - 1) + 2)) * dt_; // y(k) = y(k-1) + v(k-1)*sin(theta(k-1)) * dt
        X(3 * k + 2) = X(3 * (k - 1) + 2) + u(2 * (k - 1) + 1);                              // theta(k) = theta(k-1) + omega
      }

      // Initial constraint for initial state
      for (int i = 0; i < mpc_steps_ + 1; i++)
      {
        g(2 * i) = X(3 * i);
        g(2 * i + 1) = X(3 * i + 1);
      }
      return g;
    }

    ifopt::Component::VecBound Constraints::GetBounds() const
    {
      ifopt::Component::VecBound bounds(ifopt::Component::GetRows());

      for (int i = 0; i < mpc_steps_ + 1; i++)
      {
        bounds.at(2 * i) = ifopt::Bounds(-ifopt::inf, ifopt::inf);     // Constraint for x
        bounds.at(2 * i + 1) = ifopt::Bounds(-ifopt::inf, ifopt::inf); // Constrain for y
      }
      return bounds;
    }

    void Constraints::SetInitialState(const Eigen::Vector2d &initial_x)
    {
      initial_x_ = initial_x;
    }

    void Constraints::FillJacobianBlock(std::string var_set, Jacobian &jac_block) const
    {
      if (var_set == VARSET_NAME)
      {
        Eigen::VectorXd u = GetVariables()->GetComponent(var_set)->GetValues();
        Eigen::VectorXd X;
        for (unsigned int k = 0; k < mpc_steps_; k++)
        {
          X(0) = initial_x_(2); // Initial theta
          if (k == 0)
          {
            jac_block.coeffRef(k, 0) = 0;
            jac_block.coeffRef(k, 1) = 0;
          }
          else
          {
            jac_block.coeffRef(k, 0) = cos(X(k - 1)) * dt_;
            jac_block.coeffRef(k, 1) = sin(X(k - 1)) * dt_;
          }
        }
      }
    }

    Cost::Cost(int mpc_steps)
        : CostTerm(COST_NAME),
          mpc_steps_(mpc_steps)
    {
    }
    Cost::~Cost() {}

    double Cost::GetCost() const
    {
      Eigen::Vector2d x = GetVariables()->GetComponent(VARSET_NAME)->GetValues();
      Eigen::VectorXd cost;
      cost(0) = 0;

      for (int k = 0; k < mpc_steps_; k++)
      {
        // cost += 
      }
      return (double)cost(0);
    }

    void Cost::FillJacobianBlock(std::string var_set, Jacobian &jac) const
    {
      if (var_set == VARSET_NAME)
      {
        Eigen::Vector2d x = GetVariables()->GetComponent(VARSET_NAME)->GetValues();

        jac.coeffRef(0, 0) = 2.0 * (x(0)); // derivative of cost w.r.t x0
        jac.coeffRef(0, 1) = 2.0 * (x(1)); // derivative of cost w.r.t x1
      }
    }
  }
}