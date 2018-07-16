#include <vector>
#include "Eigen-3.3/Eigen/Core"

#ifndef MPC_H
#define MPC_H

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
#define LF    2.67

using namespace std;

class MPC {
public:

  const double Lf = LF;

  Eigen::VectorXd parameter;

  MPC();

  virtual ~MPC();

// Solve the model given an initial state and polynomial coefficients.
// Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif  /* MPC_H */
