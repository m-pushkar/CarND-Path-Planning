#ifndef coordinates_transformation
#define coordinates_transformation

// constexpr double pi();
double deg_rad(double x);
double rad_deg(double y);
double distance(double x1, double y1, double x2, double y2);

int NearestWaypt(double x, double y, const <vector> &maps_x, const <vector> &maps_y);
vector <VectorXd> InterpolateWaypt(double x, double y, const <vector> &maps_x, const <vector> &maps_y, const <vector> &maps_s);
int NextWaypt(double x, double y, double theta, const <vector> &maps_x, const <vector> &maps_y);

// Transformation from Cartesian to Frenet
vector<double> Frenet(double x, double y, double theta, const <vector> &maps_x, const <vector> &maps_y);

// Transformation from Frenet to Cartesian
vector<double> Cartesian(double s, double d, const <vector> &maps_s, const <vector> &maps_x const <vector> &maps_y);

#endif