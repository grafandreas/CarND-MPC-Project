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


// Weights for the cost calculation
//const int cte_cost_weight = 2000;
//const int epsi_cost_weight = 2000;
//const int v_cost_weight = 1;
//const int delta_cost_weight = 10;
//const int a_cost_weight = 10;
//const int delta_change_cost_weight = 100;
//const int jerk_cost_weight = 10;

const int cte_cost_weight = 2000;
const int epsi_cost_weight = 2000;
const int v_cost_weight = 1;
const int delta_cost_weight = 5; // Inspired by Udacity video
const int a_cost_weight = 5;
const int delta_change_cost_weight = 100; // Was 200000
const int jerk_cost_weight = 10;



const int ref_v = 8; // 8 m/s is about 30 km/h

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

        // AGR check
      for (int i = 0; i < N; i++) {
        fg[0] += cte_cost_weight * CppAD::pow(vars[cte_offs + i], 2);  // Q&A has ref_cte
        fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_offs + i], 2);
        fg[0] += v_cost_weight * CppAD::pow(vars[v_offs + i] - ref_v, 2);
      }


      for (int i = 0; i < N-1; i++) {
        fg[0] += delta_cost_weight * CppAD::pow(vars[d1_offs + i], 2);
        fg[0] += a_cost_weight * CppAD::pow(vars[a_offs + i], 2);
      }


      for (int i = 0; i < N-2; i++) {
        fg[0] += delta_change_cost_weight * CppAD::pow(vars[d1_offs + i + 1] - vars[d1_offs + i], 2);
        fg[0] += jerk_cost_weight * CppAD::pow(vars[a_offs + i + 1] - vars[a_offs + i], 2);
      }


      fg[1 + x_offs] = vars[x_offs];
      fg[1 + y_offs] = vars[y_offs];
      fg[1 + psi_offs] = vars[psi_offs];
      fg[1 + v_offs] = vars[v_offs];
      fg[1 + cte_offs] = vars[cte_offs];
      fg[1 + epsi_offs] = vars[epsi_offs];

      // The rest of the constraints
      for (int t = 1; t < N; t++) {
        // State at time t + 1
        AD<double> x1 = vars[x_offs + t];
        AD<double> y1 = vars[y_offs + t];
        AD<double> psi1 = vars[psi_offs + t];
        AD<double> v1 = vars[v_offs + t];
        AD<double> cte1 = vars[cte_offs + t];
        AD<double> epsi1 = vars[epsi_offs + t];

        // State at next time
        AD<double> x0 = vars[x_offs + t - 1];
        AD<double> y0 = vars[y_offs + t - 1];
        AD<double> psi0 = vars[psi_offs + t - 1];
        AD<double> v0 = vars[v_offs + t - 1];
        AD<double> cte0 = vars[cte_offs + t - 1];
        AD<double> epsi0 = vars[epsi_offs + t - 1];

        // Actuator constraints
        AD<double> delta0 = vars[d1_offs + t - 1];
        AD<double> a0 = vars[a_offs + t - 1];

            // AG TODo: Check QA video does not use pwo
        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0 + coeffs[3] * x0*x0*x0; // Had POW here, probalby wrong pow?
        AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);

        // Setting up the rest of the model constraints
        fg[1 + x_offs + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_offs + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_offs + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
        fg[1 + v_offs + t] = v1 - (v0 + a0 * dt);
        fg[1 + cte_offs + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_offs + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
        }
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
    vars_lowerbound[i] = - angle_constraint ; // Experimental: this is from the Q&A
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
