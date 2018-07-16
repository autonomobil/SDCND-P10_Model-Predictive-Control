#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <iomanip>      // std::setw

using CppAD::AD;

class FG_eval {
public:

// Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  Eigen::VectorXd parameter;

  FG_eval(Eigen::VectorXd coeffs, Eigen::VectorXd parameter) {
    this->coeffs = coeffs; this->parameter = parameter;
  }

  typedef CPPAD_TESTVECTOR (AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {

    // timestep length and duration
    // N * dt seconds to predict into the future
    const double N = parameter[0];
    const double dt = parameter[1];
    // reference speed
    const double ref_v = parameter[2];

    // weights parameters for the cost function
    const double factorCTE = parameter[3];
    const double factorErrorPsi = parameter[4];
    const double factorErrorV = parameter[5];
    const double factor_DeltaAndSpeed = parameter[6];
    const double factor_d_Delta = parameter[7];
    const double factor_d_Thrust = parameter[8];

    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    size_t x_start = 0;
    size_t y_start = x_start + N;
    size_t psi_start = y_start + N;
    size_t v_start = psi_start + N;
    size_t cte_start = v_start + N;
    size_t epsi_start = cte_start + N;

    size_t delta_start = epsi_start + N;
    size_t a_start = delta_start + N - 1;

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost is added to `fg[0]`.
    fg[0] = 0;

    for (size_t t = 0; t < N; t++) {
      // The part of the cost based on the state.
      fg[0] += factorCTE * CppAD::pow(vars[cte_start + t], 6);
      fg[0] += factorErrorPsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += factorErrorV * CppAD::pow((vars[v_start + t] - ref_v), 2);

      if(t < N - 1) {
        // Minimize the use of actuators.
        // fg[0] += factorDelta * CppAD::pow(vars[delta_start + t], 2); //unnecessary
        // fg[0] += factorThrust * (1 -  (CppAD::pow( vars[a_start + t], 2))); //unnecessary
        fg[0] += CppAD::pow(vars[v_start + t], 2) * CppAD::pow(vars[delta_start + t] * factor_DeltaAndSpeed, 2);
      }

      if(t < (N - 2)) {
        // Minimize the value gap between sequential actuations.
        fg[0] += factor_d_Delta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += factor_d_Thrust * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      }
    }

    // Setup Constraints
    // fg[0] = cost, so every other variable: 1+
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 0; t < N - 1; t++) {

      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t + 1];
      AD<double> y1 = vars[y_start + t + 1];
      AD<double> psi1 = vars[psi_start + t + 1];
      AD<double> v1 = vars[v_start + t + 1];
      AD<double> cte1 = vars[cte_start + t + 1];
      AD<double> epsi1 = vars[epsi_start + t + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + t];
      AD<double> y0 = vars[y_start + t];
      AD<double> psi0 = vars[psi_start + t];
      AD<double> v0 = vars[v_start + t];
      AD<double> cte0 = vars[cte_start + t];
      AD<double> epsi0 = vars[epsi_start + t];

      // Actuator constraints at time t
      AD<double> delta0 = vars[delta_start + t];
      AD<double> a0 = vars[a_start + t];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // physical model constraints
      fg[1 + x_start + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t + 1] = psi1 - (psi0 - v0 / LF * delta0 * dt);
      fg[1 + v_start + t + 1] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t + 1] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t + 1] = epsi1 - ((psi0 - psi_des0)  - v0 * delta0 / LF * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
}
MPC::~MPC() {
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

  bool ok = true;

  typedef CPPAD_TESTVECTOR (double) Dvector;

  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // timestep length and duration
  // N * dt seconds to predict into the future
  const double N = parameter[0];

  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we should to establish
  // when one variable starts and another ends to make our lifes easier.
  size_t x_start = 0;
  size_t y_start = x_start + N;
  size_t psi_start = y_start + N;
  size_t v_start = psi_start + N;
  size_t cte_start = v_start + N;
  size_t epsi_start = cte_start + N;

  size_t delta_start = epsi_start + N;
  size_t a_start = delta_start + N - 1;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 6 element vector, the actuators is a 2
  // element vector and there are 10 timesteps (N). The number of variables is:
  // 6 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N - 1);
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);

  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;


  // Lower and upper limits
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332 * Lf;
    vars_upperbound[i] = 0.436332 * Lf;
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, parameter);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.35\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
    options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << std::setw(8) << cost ;

  // Return the first actuator values.
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> mpcResult;

  mpcResult.push_back(solution.x[delta_start]);
  mpcResult.push_back(solution.x[a_start]);

  for ( size_t t = 0; t < N - 2; t++ ) {
    mpcResult.push_back(solution.x[x_start + t + 1]);
    mpcResult.push_back(solution.x[y_start + t + 1]);
  }


  return mpcResult;
}
