#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "polynomialsolver.h"

using namespace std;
using namespace Eigen;

double huber_loss(double speed, double target) {
  double delta = 5; // Speed in m/s

  if (abs(speed - target) < delta) { 
    return 1.0/2 *c(speedtarget)*(speed-target);
  }
  else {
    return delta * (abs(speed - target) - 1.0/2 * delta);
  }
}

VectorXd getpolycoeff(double s0, double s0_dot, double s0_2dot, double st, double st_dot, double st_2dot, double T_terminal) {

  // Coeffs p = a0 + a1 * t + ... a5 * t^5
  VectorXd coeffs(6);
  double a0 = s0;
  double a1 = s0_dot;
  double a2 = s0_2dot / 2.0;
  double T = T_terminal;

  Matrix3d c345;
  c345 << pow(T, 3.), pow(T, 4.), pow(T, 5.),
          3*pow(T, 2.), 4*pow(T, 3.), 5*pow(T,4.),
          6*T, 12*pow(T, 2.), 20*pow(T, 3.);

  Vector3d q345;
  q345 << st - (a0 + a1 * T + a2 * pow(T, 2.)),
          st_dot - (a1 + 2 * a2 * T),
          st_2dot - 2 * a2;

  Vector3d a345;
  a345 = c345.inverse() * q345;

  coeffs << a0, a1, a2, a345(0), a345(1), a345(2);

  return coeffs;
}

int solvepolyfullcond(double s0, double s0dot, double s02dot, double s1, double s1dot, double s12dot, double kspeed, double ks, double max_speed, double acc_thres, vector<double> Tjset, vector<double> ds1set, MatrixXd &Trajectories, VectorXd &Costs) {

  Matrix3d M1;
  M1 << 1, 0, 0,
        0, 1, 0,
        0, 0, 2;

  Vector3d zeta0;
  zeta0 << s0, s0dot, s02dot;

  Vector3d c012;
  c012 = M1.inverse() * zeta0;

  MatrixXd coeffs(6, ds1set.size() * Tjset.size());
  VectorXd costs(ds1set.size() * Tjset.size());

  double kj = 0.5; // Jerk weight
  double kt = 0; // Time weight
  int acc_cond_satisfy = 0;

  for (int i = 0; i < ds1set.size(); i++) {
    for (int j = 0; j < Tjset.size(); j++) {
      
      double ds1 = ds1set[i];
      double Tj = Tjset[j];
      double s1_target = s1 + ds1;

      // solve c3, c4, c5
      Matrix3d M1_T;
      Matrix3d M2_T;

      M1_T << 1, Tj, Tj*Tj,
                0, 1, 2* Tj,
                0, 0, 2;
      M2_T << Tj*Tj*Tj, Tj*Tj*Tj*Tj, Tj*Tj*Tj*Tj*Tj,
                3*Tj*Tj, 4*Tj*Tj*Tj, 5*Tj*Tj*Tj*Tj,
                6*Tj, 12*Tj*Tj, 20*Tj*Tj*Tj;

      Vector3d zeta1;
      zeta1 << s1_target, s1dot, s12dot; 
      Vector3d c345;
      c345 = M2_T.inverse() * (zeta1 - M1_T * c012);

      // Calculate accleration outsized
      double acc0 = abs(6 * c345(0));
      double acc1 = abs(6 * c345(0) + 24* c345(1) * Tj + 60 * c345(2) * Tj *Tj);
      double maxp = - c345(1) / (5.0 * c345(2));
      double acc2 = abs(6 * c345(0) + 24 * c345(1) * maxp + 60 * c345(2) * maxp*maxp);
      bool acc_outsize = (acc0 >= acc_thres) || (acc1 >= acc_thres) || (acc2 >= acc_thres);

      if (acc_outsize == false){

        // calculate cost
        double cost_Jerk = 36 * c345(0) * c345(0) * Tj \
                          + 144 * c345(0) * c345(1) * Tj * Tj \
                          + (192 * c345(1) * c345(1) + 240 * c345(0) *c345(2)) * Tj * Tj * Tj \
                          + 720 * c345(1) * c345(2) * Tj*Tj*Tj*Tj \
                          + 720 * c345(2) * c345(2) * Tj*Tj*Tj*Tj*Tj;

        double cost_terminal = ds1 * ds1;

        double speed_terminal = c012(1) + 2 * c012(2) * Tj \
                                + 3 * c345(0) * Tj * Tj + 4 * c345(1) * Tj * Tj * Tj \
                                + 5 * c345(2) * Tj * Tj * Tj * Tj;

        double cost_Total = kj * cost_Jerk \
                            + kt * Tj \
                            + ks * cost_terminal \
                            + kspeed * huber_loss(speed_terminal, max_speed);

        // Append coeffs and cost
        VectorXd c012345(6);
        c012345 << c012(0), c012(1), c012(2), c345(0), c345(1), c345(2);
        coeffs.col(acc_cond_satisfy) = c012345;
        costs(acc_cond_satisfy) = cost_Total;

        Trajectories.conservativeResize(6, Trajectories.cols() + 1);
        Trajectories.col(Trajectories.cols() - 1) = c012345;
        Costs.conservativeResize(Costs.size() + 1);
        Costs(Costs.size() - 1) = cost_Total;


        acc_cond_satisfy += 1;
      }
    }
  }
  return 0;
}

int solvepolytwocond(double s0, double s0dot, double s02dot, double s1dot, double s12dot, double kspeed, double max_speed, double acc_thres, vector<double> Tjset, vector<double> ds1dotset, MatrixXd &Trajectories, VectorXd &Costs) {

  Matrix3d M1;
  M1 << 1, 0, 0,
        0, 1, 0,
        0, 0, 2;

  Vector3d zeta0;
  zeta0 << s0, s0dot, s02dot;

  Vector3d c012;
  c012 = M1.inverse() * zeta0;

  Vector2d c12;
  c12 << c012(1), c012(2);

  MatrixXd coeffs(6, ds1dotset.size() * Tjset.size());
  VectorXd costs(ds1dotset.size() * Tjset.size());

  double kj = 0.5; // Jerk weight
  double kt = 0; // Time weight
  double ksdot = 0; // Terminal state weight
  int acc_cond_satisfy = 0;

  for (int i = 0; i < ds1dotset.size(); i++) {
    for (int j = 0; j < Tjset.size(); j++) {

      // Target state and terminal time
      double ds1dot = ds1dotset[i];
      double Tj = Tjset[j];
      double s1dot_target = s1dot + ds1dot;

      // Solve c3, c4 (c5 = 0 by transversality condition)
      Vector2d zeta1;
      zeta1 << s1dot_target, s12dot;

      Matrix2d M2;
      Matrix2d C2;

      M2 << 3 * Tj*Tj, 4 * Tj*Tj*Tj,
            6 * Tj, 12 * Tj*Tj;
      C2 << 1, 2*Tj,
            0, 2;

      Vector2d c34;
      c34 = M2.inverse() * (zeta1 - C2*c12);

      // Acceleration outsized
      double acc0 = abs(6 * c34(0));
      double acc1 = abs(6 * c34(0) + 24* c34(1) * Tj);
      bool acc_outsize = (acc0 >= acc_thres) || (acc1 >= acc_thres);

      if (acc_outsize == false){

        // Calculate cost
        double cost_Jerk = 36 * c34(0) * c34(0) * Tj \
                          + 144 * c34(0) * c34(1) * Tj * Tj \
                          + 192 * c34(1) * c34(1) * Tj * Tj * Tj;

        double cost_terminal = ds1dot * ds1dot;

        double speed_terminal = c012(1) + 2* c012(2) * Tj \
                                + 3* c34(0) * Tj * Tj + 4 * c34(1) * Tj * Tj * Tj;

        double cost_Total = kj * cost_Jerk \
                            + kt * Tj \
                            + ksdot * cost_terminal \
                            + kspeed * huber_loss(speed_terminal, max_speed);


        // Append coeffs and cost
        VectorXd c012345(6);
        c012345 << c012(0), c012(1), c012(2), c34(0), c34(1), 0;
        coeffs.col(acc_cond_satisfy) = c012345;
        costs(acc_cond_satisfy) = cost_Total;

        Trajectories.conservativeResize(6, Trajectories.cols()+1);
        Trajectories.col(Trajectories.cols() - 1) = c012345;
        Costs.conservativeResize(Costs.size() + 1);
        Costs(Costs.size() - 1) = cost_Total;


        acc_cond_satisfy += 1;
      }
    }
  }
  return 0;
}

int velocitykeeptrajectories(double s0, double s0dot, double s02dot, double s1dot, double max_speed, MatrixXd &s_trajectories, VectorXd &s_costs) {

    vector<double> ds1dotset;
    vector<double> ds1dotcand = {-3.0, -2.0, -1.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0};
    vector<double> Tjset = {3.0,3.5,4.0};
    double kspeed = 9.0;
    double acc_thres = 3.5;

    for (int i = 0; i < ds1dotcand.size(); i++){
      double keep_ds1dot = ds1dotcand[i];
      if (s1dot + keep_ds1dot <= max_speed) {
        if (s1dot + keep_ds1dot >= 0) ds1dotset.push_back(keep_ds1dot);
      }
      else {
        keep_ds1dot = max_speed - s1dot;
        ds1dotset.push_back(keep_ds1dot);
        break;
      }
    }
    int vel_keep = solvepolytwocond(s0, s0dot, s02dot, s1dot, 0, kspeed, max_speed, acc_thres, Tjset, ds1dotset, s_trajectories, s_costs);

    return 0;
}


int followtrajectories(double s0, double s0dot, double s02dot, double s_lv0, double s_lv0dot, double max_speed, MatrixXd &s_trajectories, VectorXd &s_costs) {
  
  // Tunning parameters : safety distance and CTG param
  double safe_dist = 15.0;
  double tau = 1.0; // constant time gap
  double kspeed = 9.0;
  double ks = 0.0;
  double acc_thres = 3.3;
  vector<double> Tjset = {3.0, 3.5, 4.0, 4.5};
  vector<double> ds1set = {-5.0, -3.0, 0, 5.0}
  double s12dot = 0.0;

  for (int i = 0; i < Tjset.size(); i++) {
    vector<double> follow_Tjset;
    double Tj = Tjset[i];
    follow_Tjset.push_back(Tj);

    // Calculate leading vehicle pos, vel, acc (CV Model in s-direction)
    double s_lv1 = s_lv0 + s_lv0dot * Tj;
    double s_lv1dot = s_lv0dot;
    double s_lv12dot = 0;

    // Calculate target pos, vel, acc
    double s_target = s_lv1 - (safe_dist + tau * s_lv1dot);
    double s_targetdot = s_lv1dot;
    double s_target2dot = 0;

    // Solve polynomials with full condition
    int follow_traj = solvepolyfullcond(s0, s0dot, s02dot, s_target, s_targetdot, s_target2dot, kspeed, ks, max_speed, acc_thres, follow_Tjset, ds1set, s_trajectories, s_costs);
  }
  return 0;
}

int lateraltrajectories(double d0, double d0dot, double d02dot, double d1, bool in_mylane, MatrixXd &d_trajectories, VectorXd &d_costs) {

    vector<double> dd1set;
    vector<double> Tjset;

    if (in_mylane) {
      dd1set = {-0.6, 0, 0.6};
      Tjset = {3.5, 3.7};
    }
    else {
      dd1set = {-1.2, -0.6, 0, 0.6, 1.2};
      Tjset = {3.2, 3.5, 3.7};
    }

    double max_speed = 15.0;
    double kspeed = 0.0;
    double ks = 2.0;
    double acc_thres = 4;

    int lateral_traj = solvePolynomialsFullTerminalCond(d0, d0dot, d02dot, d1, 0, 0, kspeed, ks, max_speed, acc_thres, Tjset, dd1set, d_trajectories, d_costs);
    return 0;
  }

vector<int> optimalcombo(VectorXd s_costs, VectorXd d_costs) {

  if ((s_costs.size() == 0) || (d_costs.size() == 0)) return {0, 0};

  // Cost weight for longitudinal / lateral
  double klon = 1.0;
  double klat = 2.0;

  // Sum matrix
  MatrixXd sd_sum(s_costs.size(), d_costs.size());
  for (int row = 0; row < s_costs.size(); row++){
    for (int col = 0; col < d_costs.size(); col++){
      sd_sum(row,col) = klon * s_costs(row) + klat * d_costs(col);
    }
  }

  // Find minimum
  int min_s_idx, min_d_idx;
  double minCost = sd_sum.minCoeff(&min_s_idx, &min_d_idx);
  return {min_s_idx, min_d_idx};
}

double getPosition(VectorXd coeffs, double t) {
  VectorXd tt(6);
  tt << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
  return coeffs.dot(tt);
}

double getVelocity(VectorXd coeffs, double t) {
  VectorXd tt(6);
  tt << 0, 1, 2*t, 3*t*t, 4*t*t*t, 5*t*t*t*t;
  return coeffs.dot(tt);
}

double getAcceleration(VectorXd coeffs, double t) {
  VectorXd tt(6);
  tt << 0, 0, 2, 3*2*t, 4*3*t*t, 5*4*t*t*t;
  return coeffs.dot(tt);
}

