#include "mrp_nmpc_orca/mpc.hpp"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

namespace mrp_nmpc_orca
{
  PoseStablisingMPC::PoseStablisingMPC()
      : robot_radius_(0.14),
        observable_range_(1),
        delta_tau_(1.5),
        current_waypoint_indx_(0),
        max_linear_vel_(0.1),
        max_angular_vel_(2.0),
        linear_error_(0.01),
        angular_error_(0.05),
        at_position_(false),
        reach_goal_(false),
        moving_to_temp_(false)
  {
  }

  PoseStablisingMPC::~PoseStablisingMPC()
  {
  }

  void PoseStablisingMPC::initialise()
  {
  }
  void PoseStablisingMPC::start()
  {
  }
  void PoseStablisingMPC::stop()
  {
  }

  void PoseStablisingMPC::setPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    path_ = path;
    current_waypoint_indx_ = 0;
    at_position_ = false;
    reach_goal_ = false;
  }

  void PoseStablisingMPC::calculateVelocityCommand(
      const nav_msgs::msg::Odometry &current_odom,
      const std::vector<nav_msgs::msg::Odometry> &members_odom,
      const sensor_msgs::msg::LaserScan &scan,
      geometry_msgs::msg::Twist &vel_cmd)
  {
  }

  // For feedback
  double PoseStablisingMPC::getDistanceToGoal(const geometry_msgs::msg::Pose &current_pose)
  {
  }

  // For accessing
  bool PoseStablisingMPC::reachGoal()
  {
  }

  void PoseStablisingMPC::setParameter(const std::unordered_map<std::string, double> &param_map)
  {
  }

  class FG_eval
  {
  public:
    // Reference values
    double ref_x_, ref_y_, ref_theta_, ref_v_, ref_omega_;
    double dt_;
    int mpc_steps_, x_start_, y_start_, theta_start_, v_start_, omega_start_;

    double w_x_, w_y_, w_theta_, w_v_, w_omega_;
    // Minimise
    FG_eval()
    {
      mpc_steps_ = 40;
      x_start_ = 0;
      y_start_ = x_start_ + mpc_steps_;
      theta_start_ = y_start_ + mpc_steps_;
      v_start_ = theta_start_ + mpc_steps_ - 1;
      omega_start_ = v_start_ + mpc_steps_ - 1;
    }

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    // fg: function that evaluates the objective and constraints using the syntax
    void operator()(ADvector &fg, const ADvector &vars)
    {
      // fg[0] for cost function
      fg[0] = 0;
      for (int i = 0; i < mpc_steps_; i++)
      {
        fg[0] += w_x_ * CppAD::pow(vars[x_start_ + i] - ref_x_, 2);
        fg[0] += w_y_ * CppAD::pow(vars[y_start_ + i] - ref_y_, 2);
        fg[0] += w_theta_ * CppAD::pow(vars[theta_start_ + i] - ref_theta_, 2);
      }

      // Optimise for control signal
      for (int i = 0; i < mpc_steps_ - 1; i++)
      {
        fg[0] += w_v_ * CppAD::pow(vars[v_start_ + i], 2);
        fg[0] += w_omega_ * CppAD::pow(vars[omega_start_ + i], 2);
      }
    }
  };

  class FG_eval_1
  {
  public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    double _dt, _ref_cte, _ref_etheta, _ref_vel;
    double _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
    int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;

    CppAD::AD<double> cost_cte, cost_etheta, cost_vel;
    // Constructor
    FG_eval_1(Eigen::VectorXd coeffs)
    {
      this->coeffs = coeffs;

      // Set default value
      _dt = 0.1; // in sec
      _ref_cte = 0;
      _ref_etheta = 0;
      _ref_vel = 0.5; // m/s
      _w_cte = 100;
      _w_etheta = 100;
      _w_vel = 1;
      _w_angvel = 100;
      _w_accel = 50;
      _w_angvel_d = 0;
      _w_accel_d = 0;

      _mpc_steps = 40;
      _x_start = 0;
      _y_start = _x_start + _mpc_steps;
      _theta_start = _y_start + _mpc_steps;
      _v_start = _theta_start + _mpc_steps;
      _cte_start = _v_start + _mpc_steps;
      _etheta_start = _cte_start + _mpc_steps;
      _angvel_start = _etheta_start + _mpc_steps;
      _a_start = _angvel_start + _mpc_steps - 1;
    }

    // Load parameters for constraints
    void LoadParams(const std::map<std::string, double> &params)
    {
      _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
      _mpc_steps = params.find("STEPS") != params.end() ? params.at("STEPS") : _mpc_steps;
      _ref_cte = params.find("REF_CTE") != params.end() ? params.at("REF_CTE") : _ref_cte;
      _ref_etheta = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
      _ref_vel = params.find("REF_V") != params.end() ? params.at("REF_V") : _ref_vel;

      _w_cte = params.find("W_CTE") != params.end() ? params.at("W_CTE") : _w_cte;
      _w_etheta = params.find("W_EPSI") != params.end() ? params.at("W_EPSI") : _w_etheta;
      _w_vel = params.find("W_V") != params.end() ? params.at("W_V") : _w_vel;
      _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
      _w_accel = params.find("W_A") != params.end() ? params.at("W_A") : _w_accel;
      _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
      _w_accel_d = params.find("W_DA") != params.end() ? params.at("W_DA") : _w_accel_d;

      _x_start = 0;
      _y_start = _x_start + _mpc_steps;
      _theta_start = _y_start + _mpc_steps;
      _v_start = _theta_start + _mpc_steps;
      _cte_start = _v_start + _mpc_steps;
      _etheta_start = _cte_start + _mpc_steps;
      _angvel_start = _etheta_start + _mpc_steps;
      _a_start = _angvel_start + _mpc_steps - 1;

      // cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl;
    }

    // MPC implementation (cost func & constraints)
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    // fg: function that evaluates the objective and constraints using the syntax
    void operator()(ADvector &fg, const ADvector &vars)
    {
      // fg[0] for cost function
      fg[0] = 0;
      cost_cte = 0;
      cost_etheta = 0;
      cost_vel = 0;

      /*
      for (int i = 0; i < _mpc_steps; i++)
      {
          cout << i << endl;
          cout << "_x_start" << vars[_x_start + i] <<endl;
          cout << "_y_start" << vars[_y_start + i] <<endl;
          cout << "_theta_start" << vars[_theta_start + i] <<endl;
          cout << "_v_start" << vars[_v_start + i] <<endl;
          cout << "_cte_start" << vars[_cte_start + i] <<endl;
          cout << "_etheta_start" << vars[_etheta_start + i] <<endl;
      }*/

      for (int i = 0; i < _mpc_steps; i++)
      {
        fg[0] += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2);          // cross deviation error
        fg[0] += _w_etheta * CppAD::pow(vars[_etheta_start + i] - _ref_etheta, 2); // heading error
        fg[0] += _w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2);            // speed error

        cost_cte += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2);
        cost_etheta += (_w_etheta * CppAD::pow(vars[_etheta_start + i] - _ref_etheta, 2));
        cost_vel += (_w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2));
      }
      // cout << "-----------------------------------------------" << endl;
      // cout << "cost_cte, etheta, velocity: " << cost_cte << ", " << cost_etheta << ", " << cost_vel << endl;

      // Minimize the use of actuators.
      for (int i = 0; i < _mpc_steps - 1; i++)
      {
        fg[0] += _w_angvel * CppAD::pow(vars[_angvel_start + i], 2);
        fg[0] += _w_accel * CppAD::pow(vars[_a_start + i], 2);
      }
      // cout << "cost of actuators: " << fg[0] << endl;

      // Minimize the value gap between sequential actuations.
      for (int i = 0; i < _mpc_steps - 2; i++)
      {
        fg[0] += _w_angvel_d * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
        fg[0] += _w_accel_d * CppAD::pow(vars[_a_start + i + 1] - vars[_a_start + i], 2);
      }
      // cout << "cost of gap: " << fg[0] << endl;

      // fg[x] for constraints
      // Initial constraints
      fg[1 + _x_start] = vars[_x_start];
      fg[1 + _y_start] = vars[_y_start];
      fg[1 + _theta_start] = vars[_theta_start];
      fg[1 + _v_start] = vars[_v_start];
      fg[1 + _cte_start] = vars[_cte_start];
      fg[1 + _etheta_start] = vars[_etheta_start];

      // Add system dynamic model constraint
      for (int i = 0; i < _mpc_steps - 1; i++)
      {
        // The state at time t+1 .
        CppAD::AD<double> x1 = vars[_x_start + i + 1];
        CppAD::AD<double> y1 = vars[_y_start + i + 1];
        CppAD::AD<double> theta1 = vars[_theta_start + i + 1];
        CppAD::AD<double> v1 = vars[_v_start + i + 1];
        CppAD::AD<double> cte1 = vars[_cte_start + i + 1];
        CppAD::AD<double> etheta1 = vars[_etheta_start + i + 1];

        // The state at time t.
        CppAD::AD<double> x0 = vars[_x_start + i];
        CppAD::AD<double> y0 = vars[_y_start + i];
        CppAD::AD<double> theta0 = vars[_theta_start + i];
        CppAD::AD<double> v0 = vars[_v_start + i];
        CppAD::AD<double> cte0 = vars[_cte_start + i];
        CppAD::AD<double> etheta0 = vars[_etheta_start + i];

        // Only consider the actuation at time t.
        // CppAD::AD<double> angvel0 = vars[_angvel_start + i];
        CppAD::AD<double> w0 = vars[_angvel_start + i];
        CppAD::AD<double> a0 = vars[_a_start + i];

        // CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
        CppAD::AD<double> f0 = 0.0;
        for (int i = 0; i < coeffs.size(); i++)
        {
          f0 += coeffs[i] * CppAD::pow(x0, i);
        }

        // AD<double> trj_grad0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
        CppAD::AD<double> trj_grad0 = 0.0;
        for (int i = 1; i < coeffs.size(); i++)
        {
          trj_grad0 += i * coeffs[i] * CppAD::pow(x0, i - 1); // f'(x0)
        }
        trj_grad0 = CppAD::atan(trj_grad0);

        // Here's `x` to get you started.
        // The idea here is to constraint this value to be 0.
        //
        // NOTE: The use of `AD<double>` and use of `CppAD`!
        // This is also CppAD can compute derivatives and pass
        // these to the solver.
        // TODO: Setup the rest of the model constraints
        fg[2 + _x_start + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
        fg[2 + _y_start + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
        fg[2 + _theta_start + i] = theta1 - (theta0 + w0 * _dt);
        fg[2 + _v_start + i] = v1 - (v0 + a0 * _dt);

        fg[2 + _cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(etheta0) * _dt));
        fg[2 + _etheta_start + i] = etheta1 - ((theta0 - trj_grad0) + w0 * _dt);
      }
    }
  };
}