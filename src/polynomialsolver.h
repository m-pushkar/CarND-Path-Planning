#ifndef _POLYNOMIALSOLVER_H_
#define _POLYNOMIALSOLVER_H_

double huber_loss(double speed, double target);

VectorXd getpolycoeff(double s0, double s0_dot, double s0_2dot, double st, double st_dot, double st_2dot, double T_terminal);

int solvepolyfullcond(double s0, double s0dot, double s02dot, double s1, double s1dot, double s12dot, double kspeed, double ks, double max_speed, double acc_thres, vector<double> Tjset, vector<double> ds1set, MatrixXd &Trajectories, VectorXd &Costs);

int solvepolytwocond(double s0, double s0dot, double s02dot, double s1dot, double s12dot, double kspeed, double max_speed, double acc_thres, vector<double> Tjset, vector<double> ds1dotset, MatrixXd &Trajectories, VectorXd &Costs);

int velocitykeeptrajectories(double s0, double s0dot, double s02dot, double s1dot, double max_speed, MatrixXd &s_trajectories, VectorXd &s_costs);

int followtrajectories(double s0, double s0dot, double s02dot, double s_lv0, double s_lv0dot, double max_speed, MatrixXd &s_trajectories, VectorXd &s_costs);

int lateraltrajectories(double d0, double d0dot, double d02dot, double d1, bool in_mylane, MatrixXd &d_trajectories, VectorXd &d_costs);

vector<int> optimalcombo(VectorXd s_costs, VectorXd d_costs);

double position(VectorXd coeffs, double t);

double velocity(VectorXd coeffs, double t);

double acceleration(VectorXd coeffs, double t);


#endif

