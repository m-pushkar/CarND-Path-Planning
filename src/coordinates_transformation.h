#ifndef _COORDINATES_TRANSFORMATION_H_
#define _COORDINATES_TRANSFORMATION_H_

constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int NearestWaypt(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

vector<VectorXd> InterpolateWaypt(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s);

int NextWaypt(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Cartesian(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif
