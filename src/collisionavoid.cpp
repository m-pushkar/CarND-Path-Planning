#include <math.h>
#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using namespace Eigen;

#include "collisionavoid.h"

int contains(double n, vector<double> range) {
  float a = range[0];
  float b = range[1];
  if (b<a){
    a=b; b=range[0];
  }
  return (n >= a && n <= b);
}

int overlap(vector<double> a, vector<double> b) {
  if (contains(a[0], b)) return 1;
  if (contains(a[1], b)) return 1;
  if (contains(b[0], a)) return 1;
  if (contains(b[1], a)) return 1;
  return 0;
}

int CollisionAvoid(double s0, double d0, double theta0, double s1, double d1, double theta1) {
  // Implement Seperating Axis Theorem (SAT) for collision detection //

  // Set ego vehicle safe distance
  double safe_dist_lon = 5.5;
  double safe_dist_lat = 1.9;

  // Vehicle wrapper
  MatrixXd rec_wrapper(2,4);
  rec_wrapper << safe_dist_lon, safe_dist_lon, -safe_dist_lon, -safe_dist_lon,
                -safe_dist_lat, safe_dist_lat, safe_dist_lat, -safe_dist_lat;
  
  // Rotate wrapper by heading
  Matrix2d rot0, rot1;

  rot0 << cos(theta0), -sin(theta0), sin(theta0), cos(theta0);
  rot1 << cos(theta1), -sin(theta1), sin(theta1), cos(theta1);

  MatrixXd rec0(2,4);
  MatrixXd rec1(2,4);

  Vector2d trans0, trans1;

  trans0 << s0, d0;
  trans1 << s1, d1;

  for (int i = 0; i < rec_wrapper.cols(); i++){
    rec0.col(i) = rot0 * rec_wrapper.col(i) + trans0;
    rec1.col(i) = rot1 * rec_wrapper.col(i) + trans1;
  }

  // Set principal axis list
  MatrixXd axis(2,4);

  axis << cos(theta0), sin(theta0), cos(theta1), sin(theta1),
          sin(theta0), -cos(theta0), sin(theta1), -cos(theta1);

  for (int i = 0; i < axis.cols(); i++) {
    Vector2d principal_axis = axis.col(i);

    // Projection of rec0
    double min0 = principal_axis.dot(rec0.col(0));
    double max0 = min0;

    for (int j = 0; j < rec0.cols(); j++){
      double proj0 = principal_axis.dot(rec0.col(j));
      if (proj0 > max0) max0 = proj0;
      if (proj0 < min0) min0 = proj0;
    }

    // Projection of rec1
    double min1 = principal_axis.dot(rec1.col(0));
    double max1 = min1;

    for (int j = 0; j < rec1.cols(); j++){
      double proj1 = principal_axis.dot(rec1.col(j));
      if (proj1 > max1) max1 = proj1;
      if (proj1 < min1) min1 = proj1;
    }

    // Overlap checking
    if (!overlap({min0, max0}, {min1, max1})) return 0;
  }
  return 1;
}

