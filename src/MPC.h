#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"


#define NUM_STEPS 10
#define DT 0.1

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
#define LF 2.67

// reference speed (mph)
#define V_REF 80

// weights for the cost function
#define W_CTE 2
#define W_EPSILON 1
#define W_V 1
#define W_DELTA 3000
#define W_A 1
#define W_DDELTA 2000
#define W_DA 1

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd wp_poly);
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif /* MPC_H */
