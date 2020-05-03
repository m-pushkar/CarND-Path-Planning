#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "polynomialsolver.h"

using namespace std;
using namespace Eigen;

double huber_loss(double speed, double target){
  double delta = 5; // speed = 5m/sec
  
  if (abs(speed - target < delta)){
    return (1.0/2 * (speed - target) * (speed - target));
  }
  else {
    return (delta * (abs(speed - target)) - 1.0/2 * delta);
  }
}

VectorXd getpolycoeff(double s0, double s0dot, double s02dot, double st, double stdot, double st2dot, double t_termial){
  // Coefficients of p = a0 + a1 * t +...+ a5 * t^5 
  VectorXd coeffs(6);
  
  double x = s0;
  double y = s0dot;
  double z = s02dot / 2;
  double t = t_terminal;
  
  Matrix3d a;
  a << pow(t,3.), pow(t,4.), pow(t,5.), 
       3*pow(t,2.), 4*pow(t,3.), 5*pow(t,4.), 
       6*t, 12*pow(t,2.), 20*pow(t,3.);
  
  Vector3d b;
  b << st - (x + y *t + z * pow(t,2.)),
       stdot - (y + 2 * z * t),
       st2dot - 2 *z;
  
  Vector3d c;
  c = a.inverse() * b;
  
  coeffs << x, y, z, c(0), c(1), c(2);
  
  return coeffs;
}

int solvepolyfullcond(double s0, double s0dot, double s02dot, double s1, double s1dot, double s12dot, double kspeed, double ks, double max_speed, double acc_thresh, vector<double> tjset, vector<double> ds1set, MatriXd &Trajectories, VectorXd &Costs){
  

}