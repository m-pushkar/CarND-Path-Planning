#ifndef collisionavoid
#define collisionavoid

#include <vector>

using std::vector;
using namespace std;
using namespace Eigen;

int constains(double n, vector<double> range);
int overlap(vector<double> a, vector<double> b);
int CollisionAvoid(double s0, double d0, double theta0, double s1, double d1, double theta1);

#endif