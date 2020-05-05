#include <iostream>
#include <vector>
#include <math.h>

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
  b << st - (x + y * t + z * pow(t,2.)),
       stdot - (y + 2 * z * t),
       st2dot - 2 * z;
  
  Vector3d c;
  c = a.inverse() * b;
  
  coeffs << x, y, z, c(0), c(1), c(2);
  
  return coeffs;
}

int solvepolyfullcond(double s0, double s0dot, double s02dot, double s1, double s1dot, double s12dot, double kspeed, double ks, double max_speed, double acc_thresh, vector<double> tjset, vector<double> ds1set, MatrixXd &Trajectories, VectorXd &Costs){
    Matrix3d m1;
    m1 << 1, 0, 0,
          0, 1, 0,
          0, 0, 2;
    
    Vector3d z0;
    z0 << s0, s0dot, s02dot;
    
    Vector3d c0;
    c0 = m1.inverse() * z0;
    
    MatrixXd coeffs(6, ds1set.size() * tjset.size());
    VectorXd cost(ds1set.size() * tjset.size());
    
    double kj = 0.5; // Jerk weight
    double kt = 0; // Time weight
    int acc_cond = 0;
    
    for (int i = 0; i < ds1set.size(); ++i){
        for (int j = 0; j < tjset.size(); ++j){
            double ds1 = ds1set[i];
            double tj = tjset[j];
            double s1_target = s1 + ds1;
            
            // Solve c3, c4, c5
            Matrix3d m1_t;
            Matrix3d m2_t;
            
            m1_t << 1, tj, tj * tj,
                    0, 1, 2 * tj,
                    0, 0, 2;
            m2_t << pow(tj,3), pow(tj,4), pow(tj,5),
                    3 * pow(tj,2), 4 * pow(tj,3), 5 * pow(tj,4),
                    6 * tj, 12 * pow(tj,2), 20 * pow(tj,3);
            
            Vector3d z1;
            z1 << s1_target, s1dot, s12dot;
            
            Vector3d c3;
            c3 = m2_t.inverse() * (z1 - m1_t * c0);
            
            // Calculate acceleration outsized
            double acc0 = abs(6 * c3(0));
            double acc1 = abs(6 * c3(0) + 24 * c3(1) * tj + 60 * c3(2) * tj * tj);
            double maxp = -c3(1) / (5 * c3(2));
            double acc2 = abs(6 * c3(0) + 24 * c3(1) * maxp + 60 * c3(2) * maxp * maxp);
            bool acc_outsize = (acc0 >= acc_thresh) || (acc1 >= acc_thresh) || (acc2 >= acc_thresh);
            
            if (acc_outsize == false){
                
                // Calculate cost
                double cost_jerk = 36 * c3(0) * c3(0) * tj \
                + 144 * c3(0) * c3(1) * tj * tj \
                + (192 * c3(1) * c3(1) + 240 * c3(0) * c3(2)) * pow(tj,3) \
                + 720 * c3(1) * c3(2) * pow(tj,4) \
                + 720 * c3(2) * c3(2) * pow(tj,5);
                
                double cost_terminal = ds1 * ds1;
                
                double speed_terminal = c0(1) + 2 * c0(2) * tj + 3 * c3(0) * tj * tj + 4 * c3(1) * pow(tj,3) + 5 * c3(2) * pow(tj,4);
                
                double cost_total = kj * cost_jerk + kt * tj + ks * cost_terminal + kspeed * huber_loss(speed_terminal, max_speed);
                
                // Append coeffs and costs
                VectorXd c(6);
                c << c0(0), c0(1), c0(2), c3(0), c3(1), c3(2);
                coeffs.col(acc_cond) = c;
                cost(acc_cond) = cost_total;
                
                Trajectories.coservativeResize(6, Trajectories.cols() + 1);
                Trajectories.col(Trajectories.cols() - 1)  = c;
                Costs.conservativeResize(Costs.size() + 1);
                Costs(Costs.size() - 1) = cost_total;
                
                acc_cond += 1;
            }
        }
    }
    return 0;
}

int solvepolytwocond(double s0, double s0dot, double s02dot, double s1dot, double s12dot, double kspeed, double max_speed, double acc_thresh, vector<double> tjset, vector<double> ds1setdot, MatrixXd &Trajectories, VectorXd &Costs){
    Matrix3d m1;
    m1 << 1, 0, 0,
          0, 1, 0,
          0, 0, 2;
    
    Vector3d z0;
    z0 << s0, s0dot, s02dot;
    
    Vector3d c0;
    c0 = m1.inverse() * z0;
    
    Vector2d c1;
    c1 << c0(1), c0(2);
    
    MatrixXd coeffs(6, ds1setdot.size() * tjset.size());
    VectorXd cost(ds1setdot.size() * tjset.size());
    
    double kj = 0.5; // Jerk weight
    double kt = 0; // Time weight
    double ksdot = 0; // Terminal state weight
    int acc_cond = 0;
    
    for (int i = 0; i < ds1setdot.size(); ++i){
        for (int j = 0; j < tjset.size(); ++j){
            double ds1dot = ds1setdot[i];
            double tj = tjset[j];
            double s1dot_target = s1dot + s12dot;
            
            //Solve c3, c4, [c5 = 0, by Transversality condition]
            Vector2d z1;
            z1 << s1dot_target, s12dot;
            Matrix2d m2;
            m2 << 3 * tj * tj, 4 * pow(tj,3),
                  6 * tj, 12 * tj * tj;
            
            Matrix2d c2;
            c2 << 1, 2 * tj,
                  0, 2;
            
            Vector2d c3;
            c3 = m2.inverse() * (z1 - c2 * c1);
            
            // Calculate acceleration outsized
            double acc0 = abs(6 * c3(0));
            double acc1 = abs(6 * c3(0) + 24 * c3(1) * tj);
            bool acc_outsize = (acc0 >= acc_thresh) || (acc1 >= acc_thresh);
            
            if (acc_outsize == false){
                
                // Calculate cost
                double cost_jerk = 36 * c3(0) * c3(0) * tj \
                + 144 * c3(0) * c3(1) * tj * tj \
                + 192 * c3(1) * c3(1) * tj * tj * tj;
                
                double cost_terminal = ds1dot * ds1dot;
                
                double speed_terminal = c0(1) + 2 * c0(2) * tj + 3 * c3(0) * tj * tj + 4 * c3(1) * pow(tj,3);
                
                double cost_total = kj * cost_jerk + kt * tj + ksdot * cost_terminal + kspeed * huber_loss(speed_terminal, max_speed);
                
                VectorXd c(6);
                c << c0(0), c0(1), c0(2), c3(0), c3(1), 0;
                
                coeffs.col(acc_cond) = c;
                cost(acc_cond) = cost_total;
                
                Trajectories.coservativeResize(6, Trajectories.cols() + 1);
                Trajectories.col(Trajectories.cols() - 1)  = c;
                Costs.conservativeResize(Costs.size() + 1);
                Costs(Costs.size() - 1) = cost_total;
                
                acc_cond += 1;
            }
        }
    }
    return 0;
}

int velocitykeeptrajectories(double s0, double s0dot, double s02dot, double s1dot, double max_speed, MatrixXd &s_trajectories, VectorXd &s_costs){
    vector<double> ds1setdot;
    vector<double> ds1candot = {-3.0, -2.0, -1.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0};
    vector<double> tjset = {3.0, 3.5, 4.0};
    double kspeed = 9.0;
    double acc_thresh = 3.5; // Tuneable
    
    for (int i = 0; i < ds1candot.size(); ++i){
        double vel_ds1dot = ds1candot[i];
        if (s1dot + vel_ds1dot <= max_speed){
            if (s1dot + vel_ds1dot >= 0){
                ds1setdot.push_back(vel_ds1dot);
            }
        }
        else {
            vds1dot = max_speed - s1dot;
            ds1setdot.push_back(vel_ds1dot);
            break;
        }
    }
    int vel_keep = solvepolytwocond(s0, s0dot, s02dot, s1dot, 0, kspeed, max_speed, acc_thresh, tjset, ds1setdot, s_trajectories, s_costs);
    
    return 0;
}

int followtrajectories(double s0, double s0dot, double s02dot, double s_lv0, double s_lv0dot, double max_speed, MatrixXd &s_trajectories, VectorXd &s_costs){
    double safe_dist = 15.0;
    double t = 1.0; // Time gap
    double kspeed = 9.0;
    double ks = 0.0;
    double acc_thresh = 3.3; // Tuneable
    vector<double> tjset = {3.0, 3.5, 4.0, 4.5};
    vector<double> ds1set = {-5.0, -3.0, 0.0, 5.0};
    double s12dot = 0.0;
    
    for (int i = 0; i < tjset.size(); ++i){
        vector<double> follow_tjset;
        double tj = tjset[i];
        follow_tjset.push_back(tj);
        
        // Leading vehicle pos, vel, acc
        double s_lv1 = s_lv0 + s_lv0dot * tj;
        double s_lv1dot = s_lv0dot;
        double s_lv12dot = 0;
        
        // Target pos, vel, acc
        double s_target = s_lv1 - (safe_dist + t * s_lv1dot);
        double s_targetdot = s_lv1dot;
        double s_target2dot = 0;
        
        // Polynomial solve
        int follow_traj = solvepolyfullcond(s0, s0dot, s02dot, s_target, s_targetdot, s_target2dot, kspeed, ks, max_speed, acc_thresh,                                            follow_tjset, ds1set, s_trajectories, s_costs);
    }
    return 0;
}

int lateraltrajectories(double d0, double d0dot, double d02dot, double d1, bool mylane, MatrixXd &d_trajectories, VectorXd &d_costs){
    vector<double> dd1set;
    vector<double> tjset;
    
    if(mylane){
        dd1set = {-0.6, 0.0, 0.6};
        tjset = {3.5, 3.7};
    }
    else {
        dd1set = {-1.2, -0.6, 0.0, 0.6, 1.2};
        tjset = {3.2, 3.5, 3.7};
    }
    
    double max_speed = 15.0;
    double kspeed = 0.0;
    double ks = 2.0;
    double acc_thresh = 4.0;
    
    int lateral_traj = solvepolyfullcond(d0, d0dot, d02dot, d1, 0, 0, kspeed, ks, max_speed, acc_thresh, tjset, dd1set, d_trajectories, d_costs);
    return 0;
}

vector<int> optimalcombo(VectorXd s_costs, VectorXd d_costs){
    if ((s_costs.size() == 0) || (d_costs.size() == 0))
        return {0,0};
    
    // Cost weight for longitudinal and lateral
    double k_long = 1.0;
    double k_lat = 2.0;
    
    // Sum matrix
    MatrixXd sd_sum(s_costs.size(), d_costs.size());
    for (int row = 0; row < s_costs.size(); ++row){
        for (int col = 0; col < d_costs.size(); ++col){
            sd_sum(row,col) = k_long * s_costs(row) + k_lat * d_costs(col);
        }
    }
    
    // Minimum
    int min_s, min_d;
    double min_cost = sd_sum.minCoeff(&min_s, &min_d);
    return {min_s, min_d};
}

double position(VectorXd coeffs, double t){
    VectorXd tt(6);
    tt << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
    return (coeffs.dot(tt));
}

double velocity(VectorXd coeffs, double t){
    VectorXd tt(6);
    tt << 0, 1, 2*t, 3*t*t, 4*t*t*t, 5*t*t*t*t;
    return (coeffs.dot(tt));
}

double acceleration(VectorXd coeffs, double t){
    VectorXd tt(6);
    tt << 0, 0, 2, 6*t, 12*t*t, 20*t*t*t;
    return (coeffs.dot(tt));
}
