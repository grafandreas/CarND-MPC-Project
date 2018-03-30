#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <limits>

using CppAD::AD;

// TODO: Set the timestep length and duration

// Using size_t seems strange, we want semantic types
// and size_t is the sizeof an object, not an offset;
//
typedef unsigned long  ulong;
ulong const N = 10;       // Initial experimental value
double const  dt = 0.1;    // Initial experimental value

const ulong x_offs = 0;
const ulong y_offs = x_offs + N;
const ulong psi_offs = y_offs+N;
const ulong v_offs = psi_offs +N;
const ulong cte_offs = v_offs+N;
const ulong epsi_offs = cte_offs+N;
const ulong d1_offs = epsi_offs+N;
const ulong a_offs = d1_offs+(N-1);


const double angle_constraint = 0.436332; // Constraint for steering angle
const double acc_constraint = 1.0; // Constraint for acceleration

const ulong n_input_params = 6;
const ulong n_actuators = 2;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // calculate constants
  // Refactored to not use any hard coded constants.
  //
  ulong const n_vars = (n_input_params*N)+n_actuators*(N-1);
  ulong const n_constraints = n_input_params*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  vars[x_offs] = state[0];
  vars[y_offs] = state[1];
  vars[psi_offs] = state[2];
  vars[v_offs] = state[3];
  vars[cte_offs] = state[4];
  vars[epsi_offs] = state[5];

  // Initialize the bounds in a portable way for all platforms
  //
  for(unsigned int i = 0; i < d1_offs; i++) {
      vars_lowerbound[i] = std::numeric_limits<double>::lowest();
      vars_upperbound[i] = std::numeric_limits<double>::max();
  }

  // The upper and lower limits of delta are set to -25 and 25
  // (Values taken from the project lesson)
  for (unsigned int i = d1_offs; i < a_offs; i++) {
    vars_lowerbound[i] = - angle_constraint;
    vars_upperbound[i] = angle_constraint;
  }

  // Experimental values
  // AGR: Check for maximum acceleration and jerk
  //
  for (unsigned int i = a_offs; i < n_vars; i++) {
    vars_lowerbound[i] = -acc_constraint;
    vars_upperbound[i] = acc_constraint;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_offs] = state[0];
  constraints_lowerbound[y_offs] = state[1];
  constraints_lowerbound[psi_offs] = state[2];
  constraints_lowerbound[v_offs] = state[3];
  constraints_lowerbound[cte_offs] = state[4];
  constraints_lowerbound[epsi_offs] = state[5];

  constraints_upperbound[x_offs] = state[0];
  constraints_upperbound[y_offs] = state[1];
  constraints_upperbound[psi_offs] = state[2];
  constraints_upperbound[v_offs] = state[3];
  constraints_upperbound[cte_offs] = state[4];
  constraints_upperbound[epsi_offs] = state[5];


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
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
  options += "Numeric max_cpu_time          0.5\n";
  // AGR: Change CPU TIME!

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
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  std::vector<double> result = {solution.x[d1_offs], solution.x[a_offs] };
  for (unsigned int i = 0; i < N; ++i) {
    result.push_back(solution.x[x_offs + i]);
    result.push_back(solution.x[y_offs + i]);
  }
  return result;
}
