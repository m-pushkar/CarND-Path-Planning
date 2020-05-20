#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <algorithm>
#include <string>
#include <vector>
#include <uWS/uWS.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// for convenience
using std::string;
using std::vector;
using std::min;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

constexpr double pi() {
    return M_PI;
}

// Degree to radians conversion
double deg_rad(double x) {
    return x * pi() / 180;
}

// Radians to degree conversion
double rad_deg(double x) {
    return x * 180 / pi();
}

// Distance between 2 points
double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Find nearest waypoint
int NearestWaypt(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {
    double large = 100000; //Large number for comparison
    int nearest_waypt = 0;
    
    for (int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        
        if (dist < large) {
            large = dist;
            nearest_waypt = i;
        }
    }
    return nearest_waypt;
}

// Find next waypoint
int NextWaypt(double x, double y, double theta, const vector <double> &maps_x, const vector <double> &maps_y) {
    
    int nearest_waypt = NearestWaypt(x, y, maps_x, maps_y);
    double map_x = maps_x[nearest_waypt];
    double map_y = maps_y[nearest_waypt];
    double heading = atan2((map_y - y), (map_x - x));
    double angle = fabs(theta - heading);
    double angle_comp = min(2 * pi() - angle, angle);
    
    if (angle_comp > pi()/4) {
        nearest_waypt++;
        if (nearest_waypt == maps_x.size()) {
            nearest_waypt = 0;
        }
    }
    return nearest_waypt;
}

// Cartesian to frenet transformation
vector<double> Frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = NextWaypt(x, y, theta, maps_x, maps_y);
    int prev_wp = next_wp - 1;
    
    if (next_wp ==0) {
        prev_wp = maps_x.size() - 1;
    }
    
    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];
    
    // Projection of x on n
    double proj_norm = (x_x * n_x + x_y * n_y)/(n_x * n_x + n_y * n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    double frenet_d = distance(x_x, x_y, proj_x, proj_y);
    
    // Comapre d value with center to see if it is +ve or -ve
    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double center_pos = distance(center_x, center_y, x_x, x_y);
    double center_ref = distance(center_x, center_y, proj_x, proj_y);
    
    if (center_pos <= center_ref) {
        frenet_d *= -1;
    }
    
    // Get s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
    }
    frenet_s += distance(0, 0, proj_x, proj_y);
    return {frenet_s, frenet_d};
}

// Frenet to cartesian transformation
vector<double> Cartesian(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int prev_wp = -1;
    
    while(s > maps_s[prev_wp +1] && (prev_wp < (int)(maps_s.size() -1))) {
        prev_wp++;
    }
    
    int wp2 = (prev_wp + 1)%maps_x.size();
    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),(maps_x[wp2] - maps_x[prev_wp]));
    
    // x, y, s alomg the segment
    double seg_s = (s - maps_s[prev_wp]);
    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);
    double perp_heading = heading - pi()/2;
    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x,y};
}

#endif  // HELPERS_H