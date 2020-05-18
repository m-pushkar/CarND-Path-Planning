#include <math.h>
#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "coordinates_transformation.h"

using namespace std;
using namespace Eigen;

// Transformation between radians and degrees.
constexpr double pi() {
    return M_PI;
}

double deg2rad(double x) {
    return x * pi() / 180;
}

double rad2deg(double x) {
    return x * 180 / pi();
}

double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int NearestWaypt(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

    double large_num = 100000; //large number
    int closest_waypoint = 0;

    for(int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < large_num) {
            large_num = dist;
            closest_waypoint = i;
        }
    }
    return closest_waypoint;
}

vector<VectorXd> InterpolateWaypt(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s) {
    
  int closest_waypoint = NearestWaypt(x, y, maps_x, maps_y);
  int end_waypoint = maps_x.size();
  int n_interpolate = 4;
    
  // Waypoints to interpolate x=x(s), y=y(s)
  int interpolate_start = closest_waypoint - n_interpolate/2 + 1;
  if (interpolat_start < 0) {interpolat_start = 0;}
  if (interpolat_start + n_interpolate > end_waypoint ) {interpolat_start = end_waypoint - n_interpolate;}

  VectorXd xs(n_interpolate);
  VectorXd ys(n_interpolate);
  VectorXd ss(n_interpolate);
  for (int i = 0; i < n_interpolate; i++) {
    int a = interpolateStartPoint + i;
    xs(i) = maps_x[a];
    ys(i) = maps_y[a];
    ss(i) = maps_s[a];
  }
    
  // Ax = b construction
  MatrixXd A(n_interpolate, 4);
  for (int i = 0; i < n_interpolate; i++) {
    RowVectorXd sc(4);
    sc << 1, ss(i), ss(i)*ss(i), ss(i)*ss(i)*ss(i);
    A.row(i) = sc;
  }
    
  // Solve least square
  VectorXd x_coeffs(4);
  VectorXd y_coeffs(4);
  x_coeffs = A.colPivHouseholderQr().solve(xs);
  y_coeffs = A.colPivHouseholderQr().solve(ys);

  return {x_coeffs, y_coeffs};
}

int NextWaypt(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closest_waypoint = NearestWaypt(x,y,maps_x,maps_y);
    double map_x = maps_x[closest_waypoint];
    double map_y = maps_y[closest_waypoint];
    double heading = atan2((map_y - y),(map_x - x));
    double angle = abs(theta - heading);

    if(angle > pi()/4)
    {
        closest_waypoint++;
    }
    return closest_waypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = NextWaypt(x,y, theta, maps_x,maps_y);
    int prev_wp;
    prev_wp = next_wp - 1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // Projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y)/(n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double center_pos = distance(center_x, center_y, x_x, x_y);
    double center_ref = distance(center_x, center_y, proj_x, proj_y);

    if(center_pos <= center_ref)
    {
        frenet_d *= -1;
    }

    // Calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
    }
    frenet_s += distance(0, 0, proj_x, proj_y);
    
    return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Cartesian(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();
    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]) , (maps_x[wp2] - maps_x[prev_wp]));
    
    // x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);
    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);
    double perp_heading = heading - pi()/2;
    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x,y};
}
