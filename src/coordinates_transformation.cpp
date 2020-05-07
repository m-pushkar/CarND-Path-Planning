#include <math.h>
#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "coordinates_transformation.h"

using namespace std;
using namespace Eigen;

// Transforation between radians and degrees
double deg_rad(double x){
  return (x * M_PI / 180);
}

double rad_deg(double y){
  return (y * 180 / M_PI);
}

double distance(double x1, double y1, double x2, double y2){
  return (sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)));
}

int NearestWaypt(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y){
  double large_num = 100000;
  int nearest_waypt = 0;
  
  for (int i =0; i < maps_x.size(); ++i){
    double x_maps = maps_x[i];
    double y_maps = maps_y[i];
    double dist = distance(x, y, x_maps, y_maps);
    
    if (dist < large_num){
      large_num = dist;
      nearest_waypt = i;    
    }
  }
  return nearest_waypt;
}

vector <VectorXd> InterpolateWaypt(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s){
  int nearest_waypt = NearestWaypt(x, y, maps_x, maps_y);
  int last_waypt = maps_x.size();
  int interpolate = 4;
  
  // Waypoints for interpolation
  int interpolate_start = nearest_waypt - interpolate/2 + 1;
  if (interpolate_start < 0){
    interpolate_start = 0;
  }
  if (interpolate_start + interpolate > last_waypt){
    interpolate_start = last_waypt - interpolate;
  }
  
  VectorXd xs(interpolate); VectorXd ys(interpolate); VectorXd ss(interpolate);
  
  for (int i = 0; i < interpolate; ++i){
    int a = interpolate_start + i;
    xs(i) = maps_x[a];
    ys(i) = maps_y[a];
    ss(i) = maps_s[a]; 
  }
  
  // Ax = b construction
  MatrixXd A(interpolate, 4);
  for (int i = 0; i < interpolate; ++i){
    RowVectorXd sc(4);
    sc << 1, ss(i), ss(i) * ss(i), ss(i) * ss(i) * ss(i);
    A.row(i) = sc;
  }
  
  // Least square
  VectorXd x_cof(4);
  VectorXd y_cof(4);
  x_cof = A.colPivHouseholderQr().solve(xs);
  y_cof = A.colPivHouseholderQr().solve(ys);
  
  return {x_cof, y_cof};  
}

int NextWaypt(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y){
  int nearest_waypt = NearestWaypt(x, y, maps_x, maps_y);
  double x_maps = maps_x[nearest_waypt];
  double y_maps = maps_y[nearest_waypt];
  double heading = atan2((y_maps - y), (x_maps - x));
  double angle = abs(theta - heading);
  
  if (angle > M_PI/4){
    nearest_waypt++;
  }
  return nearest_waypt;
}

// Transformation from Cartesian to Frenet
vector<double> Frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y){
  int next_waypt = NextWaypt(x, y, theta, maps_x, maps_y);
  int prev_waypt = next_waypt - 1;
  if (next_waypt == 0){
    prev_waypt = maps_x.size() - 1;
  }
  
  double n_x = maps_x[next_waypt] - maps_x[prev_waypt];
  double n_y = maps_y[next_waypt] - maps_y[next_waypt];
  double x_x = x - maps_x[prev_waypt];
  double x_y = y - maps_y[prev_waypt];
  
  // Projection of x on n
  double proj_normal = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_normal * n_x;
  double proj_y = proj_normal * n_y;
  double frenet_d = distance(x_x, x_y, proj_x, proj_y);
  
  // Check frenet_dist is +ve or -ve by compairing with center point
  double center_x = 1000 - maps_x[prev_waypt];
  double center_y = 2000 - maps_y[prev_waypt];
  double center_pos = distance(center_x, center_y, x_x, x_y);
  double center_ref = distance(center_x, center_y, proj_x, proj_y);
  
  if (center_pos <= center_ref){
    frenet_d *= -1;
  }
  
  //s value calculation
  double frenet_s = 0;
    
  for (int i = 0; i < prev_waypt; ++i){
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
  }
  frenet_s += distance(0, 0, proj_x, proj_y);
  return {frenet_s, frenet_d};
}

// Transformation from Frenet to Cartesian
vector<double> Cartesian(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y){
  int prev_waypt = -1;
  
  while(s > maps_s[prev_waypt + 1] && (prev_waypt < (int)(maps_s.size() - 1))){
    prev_waypt++;
  }
  
  int waypt2 = (prev_waypt + 1) % maps_x.size();
  double heading = atan2((maps_y[waypt2] - maps_y[prev_waypt]) , (maps_x[waypt2] - maps_x[prev_waypt]));
  double seg_s = (s - maps_s[prev_waypt]);
  double seg_x = maps_x[prev_waypt] + seg_s * cos(heading);
  double seg_y = maps_y[prev_waypt] + seg_s * sin(heading);
  double perpend_heading = heading - M_PI/2;
  double x = seg_x + d * cos(perpend_heading);
  double y = seg_y + d * sin(perpend_heading);
  
  return {x, y};
}  
